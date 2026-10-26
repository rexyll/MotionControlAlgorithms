#include "mca/simulation/gazebo_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
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
    configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (!configured_ || info.paused) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0) return;

    // A command is consumed once. Invalid or mismatched commands never reach the simulator.
    if (pending_command_ && pending_step_index_ == step_index_ && pending_command_value_.finite()) {
      auto *force = ecm.Component<gz::sim::components::JointForceCmd>(joint_);
      if (force && !force->Data().empty()) {
        force->Data()[0] = pending_command_value_.effort[0];
      }
      pending_command_ = false;
    } else {
      auto *force = ecm.Component<gz::sim::components::JointForceCmd>(joint_);
      if (force && !force->Data().empty()) force->Data()[0] = 0.0;
      pending_command_ = false;
    }
    if (step_index_ != std::numeric_limits<std::size_t>::max()) ++step_index_;
  }

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (!configured_) return;
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    const double dt_s = std::chrono::duration<double>(info.dt).count();
    const auto *position = ecm.Component<gz::sim::components::JointPosition>(joint_);
    const auto *velocity = ecm.Component<gz::sim::components::JointVelocity>(joint_);
    if (!std::isfinite(time_s) || !std::isfinite(dt_s) || time_s < 0.0 || dt_s <= 0.0 ||
        !position || !velocity || position->Data().empty() || velocity->Data().empty()) {
      state_valid_ = false;
      return;
    }

    mca::simulation::SimulationStep next_step{time_s, dt_s, step_index_};
    mca::simulation::ScalarSimulationState next_state{};
    next_state.position[0] = position->Data()[0];
    next_state.velocity[0] = velocity->Data()[0];
    // Gazebo's generic joint components expose position and velocity; acceleration
    // is left at zero until a version-specific acceleration source is available.
    next_state.acceleration[0] = 0.0;
    if (!next_step.valid() || !next_state.finite()) {
      state_valid_ = false;
      return;
    }
    cached_step_ = next_step;
    cached_state_ = next_state;
    state_valid_ = true;
  }

  bool read_state(mca::simulation::SimulationStep& step,
                  mca::simulation::ScalarSimulationState& state) noexcept override {
    if (!configured_ || !state_valid_) return false;
    step = cached_step_;
    state = cached_state_;
    return true;
  }

  bool write_command(const mca::simulation::SimulationStep& step,
                    const mca::simulation::ScalarActuatorCommand& command) noexcept override {
    if (!configured_ || !step.valid() || !command.finite() ||
        step.step_index != step_index_) return false;
    pending_step_index_ = step.step_index;
    pending_command_value_ = command;
    pending_command_ = true;
    return true;
  }

 private:
  gz::sim::Entity joint_{gz::sim::kNullEntity};
  bool configured_{false};
  bool state_valid_{false};
  bool pending_command_{false};
  std::size_t step_index_{0};
  std::size_t pending_step_index_{0};
  mca::simulation::SimulationStep cached_step_{};
  mca::simulation::ScalarSimulationState cached_state_{};
  mca::simulation::ScalarActuatorCommand pending_command_value_{};
};
}  // namespace mca_gazebo_adapter

GZ_ADD_PLUGIN(mca_gazebo_adapter::ScalarJointSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(mca_gazebo_adapter::ScalarJointSystem,
                    "mca_gazebo_adapter::ScalarJointSystem")
