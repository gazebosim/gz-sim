/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ScalarVelocity.hh"
#include "ignition/gazebo/systems/DiffDrive.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::DiffDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Pose &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief EntityId of the left joint
  public: EntityId leftJointId = kNullEntity;

  /// \brief EntityId of the right joint
  public: EntityId rightJointId = kNullEntity;

  /// \brief
  public: std::string leftJointName = "left_wheel_joint";
  public: std::string rightJointName = "right_wheel_joint";
  public: double leftJointSpeed{0};
  public: double rightJointSpeed{0};
  public: double wheelSeparation{1.25};
  public: double wheelRadius{0.3};
  public: common::Time prevUpdateTime;
};

DiffDrive::DiffDrive()
  : dataPtr(std::make_unique<DiffDrivePrivate>())
{
  this->dataPtr->node.Subscribe("/cmd_vel",
                                &DiffDrivePrivate::OnCmdVel,
                                this->dataPtr.get());
  // TODO: Read params from SDF

  // TODO(future): Attach this to a model instead of the world
}

void DiffDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->leftJointId == kNullEntity ||
      this->dataPtr->rightJointId == kNullEntity)
  {
    _ecm.Each<components::Joint, components::Name>(
        [&](const EntityId &_entity, const components::Joint *,
            const components::Name *_name) -> bool
        {
          if (this->dataPtr->leftJointName == _name->Data())
          {
            this->dataPtr->leftJointId = _entity;
            igndbg << "Found joint [" << this->dataPtr->leftJointName << "] = ["
                   << this->dataPtr->leftJointId << "]" << std::endl;;
          }
          else if (this->dataPtr->rightJointName == _name->Data())
          {
            this->dataPtr->rightJointId = _entity;
            igndbg << "Found joint [" << this->dataPtr->rightJointName << "] = ["
                   << this->dataPtr->rightJointId << "]" << std::endl;;
          }

          return this->dataPtr->leftJointId != kNullEntity ||
                 this->dataPtr->rightJointId != kNullEntity;
        });
  }

  if (this->dataPtr->leftJointId == kNullEntity ||
      this->dataPtr->rightJointId == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update left wheel
  auto leftVel =
      _ecm.Component<components::ScalarVelocity>(this->dataPtr->leftJointId);

  if (leftVel == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->leftJointId,
        components::ScalarVelocity(this->dataPtr->leftJointSpeed));
  }
  else
  {
    *leftVel = components::ScalarVelocity(this->dataPtr->leftJointSpeed);
  }

  // Update right wheel
  auto rightVel =
      _ecm.Component<components::ScalarVelocity>(this->dataPtr->rightJointId);

  if (rightVel == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->rightJointId,
        components::ScalarVelocity(this->dataPtr->rightJointSpeed));
  }
  else
  {
    *rightVel = components::ScalarVelocity(this->dataPtr->rightJointSpeed);
  }
}

void DiffDrivePrivate::OnCmdVel(const msgs::Pose &_msg)
{
  auto linVel = _msg.position().x();
  auto angVel =  msgs::Convert(_msg.orientation()).Euler().Z();

  this->leftJointSpeed =
      (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  this->rightJointSpeed =
    (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::DiffDrive,
                    ignition::gazebo::System,
                    DiffDrive::ISystemPreUpdate)

