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

#include <ignition/msgs/model.pb.h>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "JointStatePublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
JointStatePublisher::JointStatePublisher()
    : System()
{
}

//////////////////////////////////////////////////
void JointStatePublisher::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm, EventManager &)
{
  // Get the model.
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "The JointStatePublisher system should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Create the position, velocity, and force components for the joint.
  std::vector<Entity> childJoints = _ecm.ChildrenByComponents(
      this->model.Entity(), components::Joint());
  for (const Entity &joint : childJoints)
  {
    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(joint,
          components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(joint, components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(joint,
          components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(joint, components::JointVelocity());
    }

    // Create joint force component if one doesn't exist
    if (!_ecm.EntityHasComponentType(joint, components::JointForce().TypeId()))
    {
      _ecm.CreateComponent(joint, components::JointForce());
    }
  }
}

//////////////////////////////////////////////////
void JointStatePublisher::PostUpdate(const UpdateInfo & /*_info*/,
                                const EntityComponentManager &_ecm)
{
  // Create the model state publisher. This can't be done in ::Configure
  // because the World is not guaranteed to be accessible.
  if (!this->modelPub)
  {
    std::string worldName;

    // Get the parent entity, which is the world.
    const components::ParentEntity *parentEntity =
      _ecm.Component<components::ParentEntity>(this->model.Entity());
    if (parentEntity)
    {
      worldName = _ecm.Component<components::Name>(
          parentEntity->Data())->Data();

      // Advertise the state topic
      std::string topic = std::string("/world/") + worldName + "/model/"
        + this->model.Name(_ecm) + "/joint_state";
      this->modelPub = std::make_unique<transport::Node::Publisher>(
          this->node.Advertise<msgs::Model>(topic));
    }
  }

  // Skip if we couldn't create the publisher.
  if (!this->modelPub)
    return;

  // Create the message
  msgs::Model msg;

  // Set the name and ID.
  msg.set_name(this->model.Name(_ecm));
  msg.set_id(this->model.Entity());

  // Set the model pose
  const components::Pose *pose = _ecm.Component<components::Pose>(
      this->model.Entity());
  if (pose)
    msgs::Set(msg.mutable_pose(), pose->Data());

  // Process each joint
  std::vector<Entity> childJoints = _ecm.ChildrenByComponents(
      this->model.Entity(), components::Joint());
  for (const Entity &joint : childJoints)
  {
    // Add a joint message.
    msgs::Joint *jointMsg = msg.add_joint();
    jointMsg->set_name(_ecm.Component<components::Name>(joint)->Data());
    jointMsg->set_id(joint);

    // Set the joint pose
    pose = _ecm.Component<components::Pose>(joint);
    if (pose)
      msgs::Set(jointMsg->mutable_pose(), pose->Data());

    // Set the joint position
    const components::JointPosition *jointPositions  =
      _ecm.Component<components::JointPosition>(joint);
    if (jointPositions)
    {
      for (size_t i = 0; i < jointPositions->Data().size(); ++i)
      {
        if (i == 0)
          jointMsg->mutable_axis1()->set_position(jointPositions->Data()[i]);
        else if (i == 1)
          jointMsg->mutable_axis2()->set_position(jointPositions->Data()[i]);
        else
          ignwarn << "Joint state publisher only supports two joint axis\n";
      }
    }

    // Set the joint velocity
    const components::JointVelocity *jointVelocity  =
      _ecm.Component<components::JointVelocity>(joint);
    if (jointVelocity)
    {
      for (size_t i = 0; i < jointVelocity->Data().size(); ++i)
      {
        if (i == 0)
          jointMsg->mutable_axis1()->set_velocity(jointVelocity->Data()[i]);
        else if (i == 1)
          jointMsg->mutable_axis2()->set_velocity(jointVelocity->Data()[i]);
        else
          ignwarn << "Joint state publisher only supports two joint axis\n";
      }
    }

    // Set the joint force
    const components::JointForce *jointForce  =
      _ecm.Component<components::JointForce>(joint);
    if (jointForce)
    {
      for (size_t i = 0; i < jointForce->Data().size(); ++i)
      {
        if (i == 0)
          jointMsg->mutable_axis1()->set_force(jointForce->Data()[i]);
        else if (i == 1)
          jointMsg->mutable_axis2()->set_force(jointForce->Data()[i]);
        else
          ignwarn << "Joint state publisher only supports two joint axis\n";
      }
    }
  }

  // Publish the message.
  this->modelPub->Publish(msg);
}

IGNITION_ADD_PLUGIN(JointStatePublisher,
                    ignition::gazebo::System,
                    JointStatePublisher::ISystemConfigure,
                    JointStatePublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(JointStatePublisher,
    "ignition::gazebo::systems::JointStatePublisher")
