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

  /// \brief Find an Entity by it's name
  /// \tparam[in] ComponentTypeT Component that identifies the entity. E.g., 
  /// components::Model, components::Link, components::Joint, etc.
  /// \param[in] _ecm Instance of the EntityComponentManager to use for the
  /// search
  /// \param[in] _name Name of the model to search
  /// \return EntityId of the model with the specified name if the model was
  /// found. Otherwise, returns kNullEntity
  public: template<typename ComponentTypeT> 
          EntityId EntityByName(const EntityComponentManager &_ecm,
                                const std::string &_name);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Current velocity command
  public: math::Vector3d currentCmd = math::Vector3d::Zero;

  /// \brief EntityId of the performer
  public: EntityId performerId = kNullEntity;

  /// \brief EntityId of the performer
  /// \todo(addisu) Get this from sdf when the system API adds support for it.
  public: std::string modelName = "ball";
};

Move3dSystem::Move3dSystem() : dataPtr(std::make_unique<Move3dSystemPrivate>())
{
  this->dataPtr->node.Subscribe("/move3d/linear_vel",
                                &Move3dSystemPrivate::OnLinearVel,
                                this->dataPtr.get());
}

void Move3dSystem::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // If the performer has not been identified yet, search through the models to 
  // find it.
  if (this->dataPtr->performerId == kNullEntity)
  {
    _ecm.Each<components::Performer, components::ParentEntity>(
        [&](const EntityId &/*_entity*/, const components::Performer *,
            const components::ParentEntity *_parent) -> bool
        {
          auto name = _ecm.Component<components::Name>(_parent->Data());
          if (this->dataPtr->modelName == name->Data())
          {
            this->dataPtr->performerId = _parent->Data();
            igndbg << "Found performer " << this->dataPtr->modelName << " = "
                   << this->dataPtr->performerId << "\n";
            return false;
          } 
          return true;
        });
  } 

  if (this->dataPtr->performerId == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  if (this->dataPtr->performerId != kNullEntity)
  {
    // update the next position of the model based on the commanded velocity
    auto pose = _ecm.Component<components::Pose>(this->dataPtr->performerId);
    if (pose)
    {
      math::Pose3d newPose(pose->Data());
      double dt = std::chrono::duration<double>(_info.dt).count();
      newPose.Pos() += dt * this->dataPtr->currentCmd;
      *pose = components::Pose(newPose);
    }
  }

}

template <typename ComponentTypeT>
EntityId Move3dSystemPrivate::EntityByName(const EntityComponentManager &_ecm,
                                           const std::string &_name)
{
  EntityId output = kNullEntity;
  _ecm.Each<ComponentTypeT, components::Name>(
      [&](const EntityId &_entity, const ComponentTypeT *,
          const components::Name *_nameComp) -> bool
      {
        if (_name == _nameComp->Data())
        {
          output = _entity;
          return false;
        }
        return true;
      });
  return output;
}

void Move3dSystemPrivate::OnLinearVel(const msgs::Vector3d &_msg)
{
  this->currentCmd = msgs::Convert(_msg);
  std::cout << "Got message: " << this->currentCmd << std::endl;
}

IGNITION_ADD_PLUGIN(Move3dSystem,
                    ignition::gazebo::System,
                    Move3dSystem::ISystemPreUpdate)
