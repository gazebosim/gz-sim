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
#include <ignition/msgs/pose.pb.h>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/systems/DiffDrive.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::DiffDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief EntityId of the left joint
  public: EntityId leftJointId = kNullEntity;

  /// \brief EntityId of the right joint
  public: EntityId rightJointId = kNullEntity;

  /// \brief Name of left joint
  public: std::string leftJointName = "left_joint";

  /// \brief Name of right joint
  public: std::string rightJointName = "right_joint";

  /// \brief Calculated speed of left joint
  public: double leftJointSpeed{0};

  /// \brief Calculated speed of right joint
  public: double rightJointSpeed{0};

  /// \brief Distance between wheels
  public: double wheelSeparation{1.0};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
DiffDrive::DiffDrive()
  : dataPtr(std::make_unique<DiffDrivePrivate>())
{
}

//////////////////////////////////////////////////
void DiffDrive::Configure(const EntityId &_id,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_id);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "DiffDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->leftJointName = _sdf->Get<std::string>("left_joint",
      this->dataPtr->leftJointName).first;
  this->dataPtr->rightJointName = _sdf->Get<std::string>("right_joint",
      this->dataPtr->rightJointName).first;
  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
  this->dataPtr->node.Subscribe(topic, &DiffDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  ignmsg << "DiffDrive subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void DiffDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->leftJointId == kNullEntity ||
      this->dataPtr->rightJointId == kNullEntity)
  {
    this->dataPtr->leftJointId =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->leftJointName);
    this->dataPtr->rightJointId =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->rightJointName);
  }

  if (this->dataPtr->leftJointId == kNullEntity ||
      this->dataPtr->rightJointId == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update left wheel
  auto leftVel =
      _ecm.Component<components::JointVelocity>(this->dataPtr->leftJointId);

  if (leftVel == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->leftJointId,
        components::JointVelocity(this->dataPtr->leftJointSpeed));
  }
  else
  {
    *leftVel = components::JointVelocity(this->dataPtr->leftJointSpeed);
  }

  // Update right wheel
  auto rightVel =
      _ecm.Component<components::JointVelocity>(this->dataPtr->rightJointId);

  if (rightVel == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->rightJointId,
        components::JointVelocity(this->dataPtr->rightJointSpeed));
  }
  else
  {
    *rightVel = components::JointVelocity(this->dataPtr->rightJointSpeed);
  }
}

//////////////////////////////////////////////////
void DiffDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  auto linVel = _msg.linear().x();
  auto angVel = _msg.angular().z();

  this->rightJointSpeed =
      (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  this->leftJointSpeed =
    (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::DiffDrive,
                    ignition::gazebo::System,
                    DiffDrive::ISystemConfigure,
                    DiffDrive::ISystemPreUpdate)

