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

#include <memory>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

using namespace gz::sim;
namespace joint_control
{
class ResetJointVelocityNearPosition: public System,
                                      public ISystemConfigure,
                                      public ISystemPreUpdate
{
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &) override
  {
    this->model = Model(_entity);

    for (auto jointEntity : this->model.Joints(_ecm))
    {
      Joint joint(jointEntity);
      joint.EnablePositionCheck(_ecm, true);
    }

    if (_sdf->HasElement("position_tolerance"))
    {
      this->position_tolerance = _sdf->Get<double>("position_tolerance");
    }

    if (_sdf->HasElement("trigger_position"))
    {
      this->trigger_position = _sdf->Get<double>("trigger_position");
    }

    if (_sdf->HasElement("velocity_after_reset"))
    {
      this->velocity_after_reset = _sdf->Get<double>("velocity_after_reset");
    }
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    auto jointEntities = this->model.Joints(_ecm);

    for (auto jointEntity : jointEntities)
    {
      Joint joint(jointEntity);

      auto optPositions = joint.Position(_ecm);
      if (!optPositions.has_value() || optPositions->size() != 1)
      {
        // error message
        continue;
      }
      double position = optPositions->at(0);

      if (fabs(position - this->trigger_position) < position_tolerance)
      {
        joint.ResetVelocity(_ecm, {this->velocity_after_reset});
      }
    }
  }

  private: Model model;
  // Internal variables stored in rad, rad/s
  private: double trigger_position = 0.0;
  private: double position_tolerance = 0.01;
  private: double velocity_after_reset = 0.25;
};
}  // namespace joint_control

GZ_ADD_PLUGIN(joint_control::ResetJointVelocityNearPosition,
              gz::sim::System,
              joint_control::ResetJointVelocityNearPosition::ISystemConfigure,
              joint_control::ResetJointVelocityNearPosition::ISystemPreUpdate)
