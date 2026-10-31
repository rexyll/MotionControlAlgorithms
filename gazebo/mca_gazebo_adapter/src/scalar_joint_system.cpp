#include "mca/simulation/gazebo_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
namespace {
gz::transport::NodeOptions TransportNodeOptions() {
  gz::transport::NodeOptions options;
  const char* partition = std::getenv("GZ_PARTITION");
  if (partition != nullptr && partition[0] != '\0') {
    options.SetPartition(partition);
  }
  return options;
}
}  // namespace

class ScalarJointSystem final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate,
                                public gz::sim::ISystemPostUpdate,
                                public gz::sim::ISystemReset,
                                public mca::simulation::IScalarGazeboSimulationAdapter {
 public:
  ~ScalarJointSystem() override = default;

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
      if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0) return;
    }
    while (prefix.size() > 1 && prefix.back() == '/') prefix.pop_back();
    topic_prefix_ = prefix;
    command_topic_ = topic_prefix_ + "/cmd_effort";
    position_topic_ = topic_prefix_ + "/state/position";
    velocity_topic_ = topic_prefix_ + "/state/velocity";
    acceleration_topic_ = topic_prefix_ + "/state/acceleration";
    time_topic_ = topic_prefix_ + "/state/time";
    step_topic_ = topic_prefix_ + "/state/step_index";

    // Register exactly one subscription handler for this node/topic. Gazebo
    // Transport indexes subscription handlers by node and topic; registering
    // both typed and raw handlers for the same topic can cause them to
    // replace/conflict with each other. The standalone probe proves that the
    // typed Node::Subscribe API receives the bridged gz.msgs.Double payload.
    node_ = std::make_unique<gz::transport::Node>(TransportNodeOptions());
    // Use the same explicitly partitioned node for command subscription and
    // state publication. This avoids a second plugin-owned Transport node and
    // keeps both directions on one node callback dispatcher.
    command_subscribed_ = CreateCommandSubscriber();
    position_pub_ = node_->Advertise<gz::msgs::Double>(position_topic_);
    velocity_pub_ = node_->Advertise<gz::msgs::Double>(velocity_topic_);
    acceleration_pub_ = node_->Advertise<gz::msgs::Double>(acceleration_topic_);
    time_pub_ = node_->Advertise<gz::msgs::Double>(time_topic_);
    step_pub_ = node_->Advertise<gz::msgs::Double>(step_topic_);
    configured_ = command_subscribed_ && position_pub_ && velocity_pub_ &&
                  acceleration_pub_ && time_pub_ && step_pub_;
    const auto subscribed_topics = node_->SubscribedTopics();
    const char* environment_partition = std::getenv("GZ_PARTITION");
    gzmsg << "MCA Gazebo adapter configured command topic=" << command_topic_
          << " subscribe_ok=" << (command_subscribed_ ? "true" : "false")
          << " configured=" << (configured_ ? "true" : "false")
          << " command_subscription_mode=typed-subscribe-shared-node"
          << " node_subscribed_topics=" << subscribed_topics.size()
          << " environment_partition="
          << (environment_partition != nullptr ? environment_partition : "<unset>")
          << " node_partition=" << node_->Options().Partition() << std::endl;
    for (const auto& subscribed_topic : subscribed_topics) {
      gzmsg << "MCA Gazebo adapter transport subscription=" << subscribed_topic
            << std::endl;
    }
    // Keep this subscriber node and its callback alive for the complete
    // plugin lifetime. Transport subscriptions are discovery-driven: they
    // must remain registered while publishers appear later. Recreating the
    // node from a background thread races the transport callback dispatcher
    // and can leave a visible subscriber endpoint that never invokes the
    // plugin callback.
  }

  void Reset(const gz::sim::UpdateInfo& info,
             gz::sim::EntityComponentManager& ecm) override {
    bool had_transport_command = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      had_transport_command = transport_command_active_;
      state_valid_ = false;
      pending_command_ = false;
      pending_step_index_ = 0;
      pending_command_value_ = {};
      transport_command_active_ = false;
      transport_command_value_ = {};
      last_transport_command_wall_time_ = std::chrono::steady_clock::time_point{};
      last_sim_time_ = 0.0;
      last_dt_ = 0.0;
      has_sim_time_ = false;
      has_previous_velocity_ = false;
      previous_velocity_ = 0.0;
      cached_step_ = {};
      cached_state_ = {};
      step_index_.store(0, std::memory_order_relaxed);
    }

    bool force_component_available = false;
    if (joint_ != gz::sim::kNullEntity) {
      if (!ecm.Component<gz::sim::components::JointForceCmd>(joint_)) {
        ecm.CreateComponent(joint_, gz::sim::components::JointForceCmd({0.0}));
      }
      force_component_available =
          ecm.Component<gz::sim::components::JointForceCmd>(joint_) != nullptr;
      if (force_component_available) {
        ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_, {0.0});
      }
    }

    // Reset one-shot diagnostics so the post-reset command path is observable
    // independently from the pre-reset run. Keep the transport node and its
    // endpoints alive: Reset must not break publisher/subscriber discovery.
    transport_command_logged_.store(false, std::memory_order_relaxed);
    typed_command_logged_.store(false, std::memory_order_relaxed);
    typed_command_callback_count_.store(0, std::memory_order_relaxed);
    typed_command_callback_count_diagnostic_logged_.store(false,
                                                           std::memory_order_relaxed);
    nonzero_effort_logged_.store(false, std::memory_order_relaxed);
    transport_preupdate_diagnostic_logged_.store(false, std::memory_order_relaxed);
    transport_watchdog_expiry_logged_.store(false, std::memory_order_relaxed);
    pre_update_call_count_.store(0, std::memory_order_relaxed);
    pre_update_logged_.store(false, std::memory_order_relaxed);
    post_update_logged_.store(false, std::memory_order_relaxed);

    gzmsg << "MCA Gazebo adapter Reset completed sim_time_s="
          << std::chrono::duration<double>(info.simTime).count()
          << " had_transport_command=" << (had_transport_command ? "true" : "false")
          << " applied_effort=0 step_index=0 state_valid=false"
          << " force_component_available=" << (force_component_available ? "true" : "false")
          << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    const auto callback_count =
        typed_command_callback_count_.load(std::memory_order_relaxed);
    if (callback_count > 0 &&
        !typed_command_callback_count_diagnostic_logged_.exchange(
            true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter PreUpdate observed typed callback count="
            << callback_count << std::endl;
    }
    const auto pre_update_call =
        pre_update_call_count_.fetch_add(1, std::memory_order_relaxed);
    if (pre_update_call < 12) {
      gzmsg << "MCA Gazebo adapter PreUpdate heartbeat call=" << pre_update_call + 1
            << " paused=" << (info.paused ? "true" : "false")
            << " dt_s=" << std::chrono::duration<double>(info.dt).count()
            << " sim_time_s=" << std::chrono::duration<double>(info.simTime).count()
            << std::endl;
    }
    if (!pre_update_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter PreUpdate invoked paused="
            << (info.paused ? "true" : "false")
            << " dt_s=" << std::chrono::duration<double>(info.dt).count()
            << " sim_time_s=" << std::chrono::duration<double>(info.simTime).count()
            << std::endl;
    }
    if (!configured_ || info.paused) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0) return;

    double effort_to_apply = 0.0;
    bool transport_command_observed = false;
    bool transport_command_was_fresh = false;
    bool transport_command_expired = false;
    double transport_command_age_s = -1.0;
    double observed_transport_effort = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_sim_time_ && time_s < last_sim_time_) {
        pending_command_ = false;
        transport_command_active_ = false;
        state_valid_ = false;
        step_index_.store(0, std::memory_order_relaxed);
        // A time rollback starts a new simulation epoch. Do not calculate
        // acceleration across the discontinuity; the first sample is a baseline.
        has_previous_velocity_ = false;
      }
      last_sim_time_ = time_s;
      last_dt_ = dt_s;
      has_sim_time_ = true;

      const auto current_step = step_index_.load(std::memory_order_relaxed);
      const bool step_command_valid =
          pending_command_ && pending_step_index_ == current_step && pending_command_value_.finite();
      transport_command_age_s =
          transport_command_active_
              ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              last_transport_command_wall_time_).count()
              : -1.0;
      const bool transport_command_fresh =
          transport_command_active_ && transport_command_age_s <= command_timeout_s_;
      transport_command_observed = transport_command_active_;
      transport_command_was_fresh = transport_command_fresh;
      observed_transport_effort = transport_command_value_.effort[0];
      if (step_command_valid) {
        effort_to_apply = pending_command_value_.effort[0];
      } else if (transport_command_fresh && transport_command_value_.finite()) {
        effort_to_apply = transport_command_value_.effort[0];
      }
      if (transport_command_active_ && !transport_command_fresh) {
        transport_command_active_ = false;
        transport_command_expired = true;
        // Permit the next command after watchdog expiry to produce a fresh
        // ingress diagnostic. The workflow uses this acknowledgement to
        // ensure a transport command is active before invoking World Reset.
        transport_command_logged_.store(false, std::memory_order_relaxed);
      }
      pending_command_ = false;
      if (current_step != std::numeric_limits<std::size_t>::max()) {
        step_index_.store(current_step + 1, std::memory_order_relaxed);
      }
    }

    if (transport_command_observed &&
        !transport_preupdate_diagnostic_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter PreUpdate observed transport command="
            << observed_transport_effort
            << " fresh=" << (transport_command_was_fresh ? "true" : "false")
            << " age_s=" << transport_command_age_s
            << " timeout_s=" << command_timeout_s_
            << " applied_effort=" << effort_to_apply
            << " step_index=" << step_index_.load(std::memory_order_relaxed)
            << std::endl;
    }
    ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_, {effort_to_apply});
    if (transport_command_expired &&
        !transport_watchdog_expiry_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter transport watchdog expired age_s="
            << transport_command_age_s << " timeout_s=" << command_timeout_s_
            << " applied_effort=" << effort_to_apply << std::endl;
    }
    if (std::abs(effort_to_apply) > 1e-12 &&
        !nonzero_effort_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter applied non-zero JointForceCmd="
            << effort_to_apply << " to joint entity " << joint_ << std::endl;
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (!post_update_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter PostUpdate invoked paused="
            << (info.paused ? "true" : "false")
            << " dt_s=" << std::chrono::duration<double>(info.dt).count()
            << " sim_time_s=" << std::chrono::duration<double>(info.simTime).count()
            << std::endl;
    }
    if (!configured_) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    const auto* position = ecm.Component<gz::sim::components::JointPosition>(joint_);
    const auto* velocity = ecm.Component<gz::sim::components::JointVelocity>(joint_);
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0 ||
        !position || !velocity || position->Data().empty() || velocity->Data().empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_valid_ = false;
      has_previous_velocity_ = false;
      return;
    }

    mca::simulation::SimulationStep next_step{time_s, dt_s, step_index_.load(std::memory_order_relaxed)};
    mca::simulation::ScalarSimulationState next_state{};
    next_state.position[0] = position->Data()[0];
    next_state.velocity[0] = velocity->Data()[0];
    if (!next_step.valid() || !next_state.finite()) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_valid_ = false;
      has_previous_velocity_ = false;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Estimate joint acceleration from consecutive post-physics velocity
      // samples using the actual simulation dt. The first sample after startup
      // or a time reset is a baseline and intentionally reports zero.
      double acceleration = 0.0;
      if (has_previous_velocity_) {
        acceleration = (next_state.velocity[0] - previous_velocity_) / dt_s;
      }
      if (!std::isfinite(acceleration)) {
        state_valid_ = false;
        has_previous_velocity_ = false;
        return;
      }
      next_state.acceleration[0] = acceleration;
      previous_velocity_ = next_state.velocity[0];
      has_previous_velocity_ = true;
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

  bool CreateCommandSubscriber() {
    // Use the typed API proven by the standalone probe, sharing the node used
    // by the state publishers so command and state endpoints have one owner.
    return node_->Subscribe<gz::msgs::Double>(
        command_topic_, [this](const gz::msgs::Double& message) {
          typed_command_callback_count_.fetch_add(1, std::memory_order_relaxed);
          if (!typed_command_logged_.exchange(true, std::memory_order_relaxed)) {
            gzmsg << "MCA Gazebo adapter typed command callback invoked" << std::endl;
          }
          OnTransportCommand(message);
        });
  }

  void OnTransportCommand(const gz::msgs::Double& message) {
    OnCommand(message.data());
  }

  void OnCommand(double effort) {
    if (!std::isfinite(effort)) return;
    if (!transport_command_logged_.exchange(true, std::memory_order_relaxed)) {
      gzmsg << "MCA Gazebo adapter received typed transport effort command=" << effort
            << " on " << command_topic_ << std::endl;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    transport_command_value_.effort[0] = effort;
    last_transport_command_wall_time_ = std::chrono::steady_clock::now();
    transport_command_active_ = true;
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
  std::atomic<bool> transport_command_logged_{false};
  std::atomic<bool> typed_command_logged_{false};
  std::atomic<std::size_t> typed_command_callback_count_{0};
  std::atomic<bool> typed_command_callback_count_diagnostic_logged_{false};
  std::atomic<bool> nonzero_effort_logged_{false};
  std::atomic<bool> transport_preupdate_diagnostic_logged_{false};
  std::atomic<bool> transport_watchdog_expiry_logged_{false};
  std::atomic<std::size_t> pre_update_call_count_{0};
  std::atomic<bool> pre_update_logged_{false};
  std::atomic<bool> post_update_logged_{false};
  bool has_sim_time_{false};
  bool has_previous_velocity_{false};
  double previous_velocity_{0.0};
  mca::simulation::SimulationStep cached_step_{};
  mca::simulation::ScalarSimulationState cached_state_{};
  mca::simulation::ScalarActuatorCommand pending_command_value_{};
  mca::simulation::ScalarActuatorCommand transport_command_value_{};

  std::unique_ptr<gz::transport::Node> node_;
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
              gz::sim::ISystemPostUpdate,
              gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(mca_gazebo_adapter::ScalarJointSystem,
                    "mca_gazebo_adapter::ScalarJointSystem")
