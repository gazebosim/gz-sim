/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include "LinkPlot3DPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
LinkPlot3DPlugin::LinkPlot3DPlugin()
    : System()
{
}

//////////////////////////////////////////////////
void LinkPlot3DPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "LinkPlot3DPlugin plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");

  if (this->dataPtr->jointName == "")
  {
    ignerr << "JointPositionController found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (_sdf->HasElement("joint_index"))
  {
    this->dataPtr->jointIndex = _sdf->Get<unsigned int>("joint_index");
  }
}

//////////////////////////////////////////////////
void LinkPlot3DPlugin::PostUpdate(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm)
{
  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity);

  jointPosComp->Data().at(this->dataPtr->jointIndex)

  // -------------------------------------------

  ignition::msgs::Marker markerMsg;

  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.0, 1.0, 1.0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, 0.5));
  node.Request("/marker", markerMsg);

  // --------------------------------------------









  // Create the model state publisher. This can't be done in ::Configure
  // because the World is not guaranteed to be accessible.
  if (!this->modelPub)
  {
    std::string worldName;

    // Get the parent entity, which is the world.
    const auto *parentEntity =
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
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the name and ID.
  msg.set_name(this->model.Name(_ecm));
  msg.set_id(this->model.Entity());

  // Set the model pose
  const auto *pose = _ecm.Component<components::Pose>(
      this->model.Entity());
  if (pose)
    msgs::Set(msg.mutable_pose(), pose->Data());

  // Process each joint
  for (const Entity &joint : this->joints)
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
    const auto *jointPositions  =
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
    const auto *jointVelocity  =
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
    const auto *jointForce  =
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

IGNITION_ADD_PLUGIN(LinkPlot3DPlugin,
                    ignition::gazebo::System,
                    LinkPlot3DPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinkPlot3DPlugin,
    "ignition::gazebo::systems::LinkPlot3DPlugin")