/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/plugin/Register.hh>
#include <ignition/msgs/model.pb.h>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "StatePublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
StatePublisher::StatePublisher()
    : System()
{
}

//////////////////////////////////////////////////
void StatePublisher::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm, EventManager &)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "The StatePublisher system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void StatePublisher::PostUpdate(const UpdateInfo & /*_info*/,
                                const EntityComponentManager &_ecm)
{
  // Create the model state publisher. This can't be done in ::Configure
  // because the World is not guaranteed to be accessible.
  if (!this->modelPub)
  {
    std::string worldName = "default";

    // Get the parent entity, which is the world.
    const components::ParentEntity *parentEntity =
      _ecm.Component<components::ParentEntity>(this->model.Entity());
    if (parentEntity)
    {
      worldName = _ecm.Component<components::Name>(
          parentEntity->Data())->Data();

      std::string topic = std::string("/world/") + worldName + "/model/"
        + this->model.Name(_ecm) + "/state";
      this->modelPub = std::make_unique<transport::Node::Publisher>(
          this->node.Advertise<msgs::Model>(topic));
    }
  }

  msgs::Model msg;
  msg.set_name(this->model.Name(_ecm));
  msg.set_id(this->model.Entity());

  std::vector<Entity> childJoints = _ecm.ChildrenByComponents(
      this->model.Entity(), components::Joint());
  for (const Entity &joint : childJoints)
  {
    msgs::Joint *jointMsg = msg.add_joint();
    jointMsg->set_name(_ecm.Component<components::Name>(joint)->Data());
    const components::JointPosition *jointPositions  =
      _ecm.Component<components::JointPosition>(joint);
    if (jointPositions)
    {
      for (const double &pos : jointPositions->Data())
      {
        jointMsg->add_position(pos);
      }
    }
  }

  this->modelPub->Publish(msg);
}

IGNITION_ADD_PLUGIN(StatePublisher,
                    ignition::gazebo::System,
                    StatePublisher::ISystemConfigure,
                    StatePublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(StatePublisher,
    "ignition::gazebo::systems::StatePublisher")
