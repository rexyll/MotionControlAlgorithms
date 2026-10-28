#include "mca/simulation/gazebo_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace mca_gazebo_adapter {
class ScalarJointSystem final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate,
                                public gz::sim::ISystemPostUpdate,
                                public mca::simulation::IScalarGazeboSimulationAdapter {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    if (!sdf || !sdf->HasElement("joint_name")) return;
    const auto joint_name = sdf->Get<std::string>("joint_name");
    const gz::sim::Model model(entity);
    const auto joint = model.JointByName(ecm, joint_name);
    if (joint == gz::sim::kNullEntity) return;

    joint_ = joint;
    if (!ecm.Component<gz::sim::components::JointPosition>(joint_)) {
      ecm.CreateComponent(joint_, gz::sim::components::JointPosition({0.0}));
    }
    if (!ecm.Component<gz::sim::components::JointVelocity>(joint_)) {
      ecm.CreateComponent(joint_, gz::sim::components::JointVelocity({0.0}));
    }
    if (!ecm.Component<gz::sim::components::JointForceCmd>(joint_)) {
      ecm.CreateComponent(joint_, gz::sim::components::JointForceCmd({0.0}));
    }

    std::string prefix;
    if (sdf->HasElement("topic_prefix")) prefix = sdf->Get<std::string>("topic_prefix");
    if (prefix.empty()) {
      prefix = "/mca/" + model.Name(ecm) + "/joint/" + joint_name;
    }
    if (sdf->HasElement("command_timeout_s")) {
      command_timeout_s_ = sdf->Get<double>("command_timeout_s");
      if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0) {
        return;
      }
    }
    while (prefix.size() > 1 && prefix.back() == '/') prefix.pop_back();
    topic_prefix_ = prefix;
    command_topic_ = topic_prefix_ + "/cmd_effort";
    position_topic_ = topic_prefix_ + "/state/position";
    velocity_topic_ = topic_prefix_ + "/state/velocity";
    acceleration_topic_ = topic_prefix_ + "/state/acceleration";
    time_topic_ = topic_prefix_ + "/state/time";
    step_topic_ = topic_prefix_ + "/state/step_index";

    // Keep the subscriber handle alive for the entire system lifetime.
    // CreateSubscriber explicitly ties the subscription lifetime to this
    // stored handle, avoiding reliance on implicit node-owned subscription
    // lifetime across Gazebo system callbacks.
    command_subscriber_ = node_.CreateSubscriber<gz::msgs::Double>(
        command_topic_, &ScalarJointSystem::OnCommand, this);
    command_subscribed_ = static_cast<bool>(command_subscriber_);
    position_pub_ = node_.Advertise<gz::msgs::Double>(position_topic_);
    velocity_pub_ = node_.Advertise<gz::msgs::Double>(velocity_topic_);
    acceleration_pub_ = node_.Advertise<gz::msgs::Double>(acceleration_topic_);
    time_pub_ = node_.Advertise<gz::msgs::Double>(time_topic_);
    step_pub_ = node_.Advertise<gz::msgs::Double>(step_topic_);
    configured_ = command_subscribed_ && position_pub_ && velocity_pub_ &&
                  acceleration_pub_ && time_pub_ && step_pub_;
    const auto subscribed_topics = node_.SubscribedTopics();
    gzmsg << "MCA Gazebo adapter configured command topic=" << command_topic_
          << " subscribe_ok=" << (command_subscribed_ ? "true" : "false")
          << " configured=" << (configured_ ? "true" : "false")
          << " node_subscribed_topics=" << subscribed_topics.size() << std::endl;
    for (const auto& subscribed_topic : subscribed_topics) {
      gzmsg << "MCA Gazebo adapter transport subscription=" << subscribed_topic
            << std::endl;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (!configured_ || info.paused) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0) return;

    double effort_to_apply = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_sim_time_ && time_s < last_sim_time_) {
        pending_command_ = false;
        transport_command_active_ = false;
        state_valid_ = false;
        step_index_.store(0, std::memory_order_relaxed);
      }
      last_sim_time_ = time_s;
      last_dt_ = dt_s;
      has_sim_time_ = true;

      const auto current_step = step_index_.load(std::memory_order_relaxed);
      const bool step_command_valid =
          pending_command_ && pending_step_index_ == current_step && pending_command_value_.finite();
      const bool transport_command_fresh =
          transport_command_active_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         last_transport_command_wall_time_).count() <=
              command_timeout_s_;
      if (step_command_valid) {
        effort_to_apply = pending_command_value_.effort[0];
      } else if (transport_command_fresh && transport_command_value_.finite()) {
        effort_to_apply = transport_command_value_.effort[0];
      }
      if (transport_command_active_ && !transport_command_fresh) {
        transport_command_active_ = false;
      }
      pending_command_ = false;
      if (current_step != std::numeric_limits<std::size_t>::max()) {
        step_index_.store(current_step + 1, std::memory_order_relaxed);
      }
    }

    // Mutating Component::Data() in place does not reliably notify the ECS
    // that JointForceCmd changed. SetComponentData marks the component dirty
    // so the Gazebo physics system observes each new effort value.
    ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_, {effort_to_apply});
    if (std::abs(effort_to_apply) > 1e-12 &&
        !nonzero_effort_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter applied non-zero JointForceCmd="
            << effort_to_apply << " to joint entity " << joint_ << std::endl;
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (!configured_) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    const auto* position = ecm.Component<gz::sim::components::JointPosition>(joint_);
    const auto* velocity = ecm.Component<gz::sim::components::JointVelocity>(joint_);
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0 ||
        !position || !velocity || position->Data().empty() || velocity->Data().empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_valid_ = false;
      return;
    }

    mca::simulation::SimulationStep next_step{time_s, dt_s, step_index_.load(std::memory_order_relaxed)};
    mca::simulation::ScalarSimulationState next_state{};
    next_state.position[0] = position->Data()[0];
    next_state.velocity[0] = velocity->Data()[0];
    next_state.acceleration[0] = 0.0;
    if (!next_step.valid() || !next_state.finite()) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_valid_ = false;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cached_step_ = next_step;
      cached_state_ = next_state;
      state_valid_ = true;
    }

    PublishScalar(position_pub_, next_state.position[0]);
    PublishScalar(velocity_pub_, next_state.velocity[0]);
    PublishScalar(acceleration_pub_, next_state.acceleration[0]);
    PublishScalar(time_pub_, next_step.time_s);
    PublishScalar(step_pub_, static_cast<double>(next_step.step_index));
  }

  bool read_state(mca::simulation::SimulationStep& step,
                  mca::simulation::ScalarSimulationState& state) noexcept override {
    if (!configured_) return false;
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!state_valid_) return false;
      step = cached_step_;
      state = cached_state_;
      return true;
    } catch (...) {
      return false;
    }
  }

  bool write_command(const mca::simulation::SimulationStep& step,
                     const mca::simulation::ScalarActuatorCommand& command) noexcept override {
    if (!configured_ || !step.valid() || !command.finite() ||
        step.step_index != step_index_.load(std::memory_order_relaxed)) return false;
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if (step.step_index != step_index_.load(std::memory_order_relaxed)) return false;
      pending_step_index_ = step.step_index;
      pending_command_value_ = command;
      pending_command_ = true;
      return true;
    } catch (...) {
      return false;
    }
  }

 private:
  void OnCommand(const gz::msgs::Double& message,
                  const gz::transport::MessageInfo& message_info) {
    (void)message_info;
    const double effort = message.data();
    if (!std::isfinite(effort)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // Transport commands may arrive before the first simulation update. Keep
    // the latest finite command instead of dropping it; the wall-clock
    // watchdog still prevents an old command from being applied later.
    transport_command_value_.effort[0] = effort;
    last_transport_command_wall_time_ = std::chrono::steady_clock::now();
    transport_command_active_ = true;
    if (!transport_command_logged_) {
      gzmsg << "MCA Gazebo adapter received transport effort command=" << effort
            << " on " << command_topic_ << " (has_sim_time="
            << (has_sim_time_ ? "true" : "false") << ")" << std::endl;
      transport_command_logged_ = true;
    }
  }

  static void PublishScalar(gz::transport::Node::Publisher& publisher, double value) {
    if (!publisher || !std::isfinite(value)) return;
    gz::msgs::Double message;
    message.set_data(value);
    publisher.Publish(message);
  }

  gz::sim::Entity joint_{gz::sim::kNullEntity};
  bool configured_{false};
  bool command_subscribed_{false};
  // The simulator update callbacks and transport command ingress may execute on
  // different threads. Keep the mailbox/snapshot coherent.
  mutable std::mutex mutex_;
  bool state_valid_{false};
  bool pending_command_{false};
  std::atomic<std::size_t> step_index_{0};
  std::size_t pending_step_index_{0};
  double last_sim_time_{0.0};
  double last_dt_{0.0};
  double command_timeout_s_{0.1};
  std::chrono::steady_clock::time_point last_transport_command_wall_time_{};
  bool transport_command_active_{false};
  bool transport_command_logged_{false};
  std::atomic<bool> nonzero_effort_logged_{false};
  bool has_sim_time_{false};
  mca::simulation::SimulationStep cached_step_{};
  mca::simulation::ScalarSimulationState cached_state_{};
  mca::simulation::ScalarActuatorCommand pending_command_value_{};
  mca::simulation::ScalarActuatorCommand transport_command_value_{};

  gz::transport::Node node_;
  gz::transport::Node::Subscriber command_subscriber_;
  std::string topic_prefix_;
  std::string command_topic_;
  std::string position_topic_;
  std::string velocity_topic_;
  std::string acceleration_topic_;
  std::string time_topic_;
  std::string step_topic_;
  gz::transport::Node::Publisher position_pub_;
  gz::transport::Node::Publisher velocity_pub_;
  gz::transport::Node::Publisher acceleration_pub_;
  gz::transport::Node::Publisher time_pub_;
  gz::transport::Node::Publisher step_pub_;
};
}  // namespace mca_gazebo_adapter

GZ_ADD_PLUGIN(mca_gazebo_adapter::ScalarJointSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(mca_gazebo_adapter::ScalarJointSystem,
                    "mca_gazebo_adapter::ScalarJointSystem")
