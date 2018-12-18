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
#include "Move3dSystem.hh"
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/ParentEntity.hh"

using namespace ignition;

using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::Move3dSystemPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnLinearVel(const ignition::msgs::Vector3d &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Current velocity command
  public: std::optional<math::Vector3d> currentCmd;

  /// \brief EntityId of the performer
  public: EntityId performerId = kNullEntity;
};

Move3dSystem::Move3dSystem() : dataPtr(std::make_unique<Move3dSystemPrivate>())
{
  this->dataPtr->node.Subscribe("/move3d/linear_vel",
                                &Move3dSystemPrivate::OnLinearVel,
                                this->dataPtr.get());
}

Move3dSystem::~Move3dSystem()
{
  // do nothing. This is needed because Move3dSystemPrivate is incomplete in
  // header file
}

void Move3dSystem::Configure(
    const EntityId &_id, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &, EventManager &)
{
  this->dataPtr->performerId = _id;
  igndbg << "Move3dSystem attached to: " << this->dataPtr->performerId  << "\n";
}

void Move3dSystem::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{

  if (this->dataPtr->performerId == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  if ((this->dataPtr->performerId != kNullEntity) &&
      (this->dataPtr->currentCmd.has_value()))
  {
    // update the next position of the model based on the commanded velocity
    auto linVelocity =
        _ecm.Component<components::LinearVelocity>(this->dataPtr->performerId);


    if (linVelocity != nullptr)
    {
      *linVelocity = components::LinearVelocity(*this->dataPtr->currentCmd);
    }
    else
    {
      _ecm.CreateComponent(
          this->dataPtr->performerId,
          components::LinearVelocity(*this->dataPtr->currentCmd));
    }
    // clear the command so that we only update the component when there's a new
    // command.
    this->dataPtr->currentCmd.reset();
  }

}

void Move3dSystemPrivate::OnLinearVel(const msgs::Vector3d &_msg)
{
  std::cout << "Got: " << _msg.x() << " " << _msg.y() << std::endl;
  this->currentCmd = msgs::Convert(_msg);
}

IGNITION_ADD_PLUGIN(Move3dSystem,
                    ignition::gazebo::System,
                    Move3dSystem::ISystemConfigure,
                    Move3dSystem::ISystemPreUpdate)
