/*
 * Copyright (C) 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/JointForceCmd.hh"
#include <gz/plugin/Register.hh>

using namespace gz::sim;
namespace joint_control
{
class PulseJointVelocityCommand: public System,
                                 public ISystemConfigure,
                                 public ISystemPreUpdate
{
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &) override
  {
    this->model = Model(_entity);

    if (_sdf->HasElement("pulse_seconds"))
    {
      this->pulse_seconds = _sdf->Get<double>("pulse_seconds");
    }

    if (_sdf->HasElement("velocity_command"))
    {
      this->velocity_command = _sdf->Get<double>("velocity_command");
    }
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    double time = std::chrono::duration<double>(_info.simTime).count();
    bool pulse = static_cast<int>(std::floor(time / pulse_seconds)) % 2;

    for (auto jointEntity : this->model.Joints(_ecm))
    {
      Joint joint(jointEntity);
      if (pulse)
      {
        joint.SetVelocity(_ecm, {this->velocity_command});
        _ecm.RemoveComponent<components::JointForceCmd>(jointEntity);
      }
      else
      {
        // Setting the JointForceCmd is required to disengage the velocity
        // controller, otherwise the velocity controller remains engaged with a
        // JointVelocityCmd of 0.
        joint.SetForce(_ecm, {0.0});
      }
    }
  }

  private: Model model;
  private: double pulse_seconds = 2.0;
  // Internal variable stored in rad/s
  private: double velocity_command = 0.4;
};
}  // namespace joint_control

GZ_ADD_PLUGIN(joint_control::PulseJointVelocityCommand,
              gz::sim::System,
              joint_control::PulseJointVelocityCommand::ISystemConfigure,
              joint_control::PulseJointVelocityCommand::ISystemPreUpdate)
