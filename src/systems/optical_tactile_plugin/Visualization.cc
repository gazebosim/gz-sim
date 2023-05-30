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

#include <gz/msgs/marker.pb.h>
#include <gz/msgs/contact.pb.h>
#include <gz/transport/Node.hh>

#include "Visualization.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace optical_tactile_sensor;

//////////////////////////////////////////////////
OpticalTactilePluginVisualization::OpticalTactilePluginVisualization(
  std::string &_modelName,
  gz::math::Vector3d &_sensorSize,
  double &_forceLength,
  float &_cameraUpdateRate,
  gz::math::Pose3f &_depthCameraOffset) :
  modelName(_modelName),
  sensorSize(_sensorSize),
  forceLength(_forceLength),
  cameraUpdateRate(_cameraUpdateRate),
  depthCameraOffset(_depthCameraOffset)
{
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::InitializeSensorMarkerMsg(
  gz::msgs::Marker &_sensorMarkerMsg)
{
  // Reset all fields
  _sensorMarkerMsg = gz::msgs::Marker();

  // Initialize the marker for visualizing the sensor as a grey transparent box
  _sensorMarkerMsg.set_ns("sensor_" + this->modelName);
  _sensorMarkerMsg.set_id(1);
  _sensorMarkerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  _sensorMarkerMsg.set_type(gz::msgs::Marker::BOX);
  _sensorMarkerMsg.set_visibility(gz::msgs::Marker::GUI);
  gz::msgs::Set(_sensorMarkerMsg.mutable_scale(), this->sensorSize);

  // Set material properties
  gz::msgs::Set(_sensorMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0.5, 0.5, 0.5, 0.75));
  gz::msgs::Set(_sensorMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0.5, 0.5, 0.5, 0.75));
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RequestSensorMarkerMsg(
  gz::math::Pose3f const &_sensorPose)
{
  gz::msgs::Marker sensorMarkerMsg;

  this->InitializeSensorMarkerMsg(sensorMarkerMsg);

  gz::msgs::Set(sensorMarkerMsg.mutable_pose(),
    gz::math::Pose3d(_sensorPose.Pos().X(),
      _sensorPose.Pos().Y(), _sensorPose.Pos().Z(),
      _sensorPose.Rot().W(), _sensorPose.Rot().X(),
      _sensorPose.Rot().Y(), _sensorPose.Rot().Z()));

  this->node.Request("/marker", sensorMarkerMsg);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::InitializeContactsMarkerMsg(
  gz::msgs::Marker &_contactsMarkerMsg)
{
  // Reset all fields
  _contactsMarkerMsg = gz::msgs::Marker();

  // Initialize the marker for visualizing the physical contacts as red lines
  _contactsMarkerMsg.set_ns("contacts_" + this->modelName);
  _contactsMarkerMsg.set_id(1);
  _contactsMarkerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  _contactsMarkerMsg.set_type(gz::msgs::Marker::LINE_LIST);
  _contactsMarkerMsg.set_visibility(gz::msgs::Marker::GUI);

  gz::msgs::Set(_contactsMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(1, 0, 0, 1));
  gz::msgs::Set(_contactsMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(1, 0, 0, 1));
  _contactsMarkerMsg.mutable_lifetime()->set_sec(1);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::AddContactToMarkerMsg(
  gz::msgs::Contact const &_contact,
  gz::msgs::Marker &_contactMarkerMsg)
{
  // todo(anyone) once available, use normal field in the Contact message
  gz::math::Vector3d contactNormal(0, 0, 0.03);

  // For each contact, add a line marker starting from the contact position,
  // ending at the endpoint of the normal.
  for (auto const &position : _contact.position())
  {
    gz::math::Vector3d startPoint = gz::msgs::Convert(position);
    gz::math::Vector3d endPoint = startPoint + contactNormal;

    gz::msgs::Set(_contactMarkerMsg.add_point(), startPoint);
    gz::msgs::Set(_contactMarkerMsg.add_point(), endPoint);
  }
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RequestContactsMarkerMsg(
  const components::ContactSensorData *_contacts)
{
  gz::msgs::Marker contactsMarkerMsg;
  this->InitializeContactsMarkerMsg(contactsMarkerMsg);

  for (const auto &contact : _contacts->Data().contact())
  {
    this->AddContactToMarkerMsg(contact, contactsMarkerMsg);
  }

  this->node.Request("/marker", contactsMarkerMsg);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::InitializeNormalForcesMarkerMsgs(
  gz::msgs::Marker &_positionMarkerMsg,
  gz::msgs::Marker &_forceMarkerMsg)
{
  _positionMarkerMsg = gz::msgs::Marker();
  _forceMarkerMsg = gz::msgs::Marker();

  // Initialize marker messages for position and force of the contacts

  // Positions computed from camera
  _positionMarkerMsg.set_ns("positions_" + this->modelName);
  _positionMarkerMsg.set_id(1);
  _positionMarkerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  _positionMarkerMsg.set_type(gz::msgs::Marker::POINTS);
  _positionMarkerMsg.set_visibility(gz::msgs::Marker::GUI);
  gz::msgs::Set(_positionMarkerMsg.mutable_scale(),
    gz::math::Vector3d(1, 1, 1));

  _forceMarkerMsg.set_ns("forces_" + this->modelName);
  _forceMarkerMsg.set_id(1);
  _forceMarkerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  _forceMarkerMsg.set_type(gz::msgs::Marker::LINE_LIST);
  _forceMarkerMsg.set_visibility(gz::msgs::Marker::GUI);

  // Set material properties and lifetime
  // Blue points for positions
  gz::msgs::Set(_positionMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0, 0, 1, 1));
  gz::msgs::Set(_positionMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0, 0, 1, 1));
  _positionMarkerMsg.mutable_lifetime()->set_nsec(
    static_cast<int>(this->cameraUpdateRate * 1000000000));

  // Green lines for forces
  gz::msgs::Set(_forceMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0, 1, 0, 1));
  gz::msgs::Set(_forceMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0, 1, 0, 1));
  _forceMarkerMsg.mutable_lifetime()->set_sec(
    static_cast<int>(this->cameraUpdateRate * 1000000000));
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::AddNormalForceToMarkerMsgs(
  gz::msgs::Marker &_positionMarkerMsg,
  gz::msgs::Marker &_forceMarkerMsg,
  gz::math::Vector3f &_position,
  gz::math::Vector3f &_normalForce,
  gz::math::Pose3f &_sensorWorldPose)
{
  // Check if messages have been initialized
  if (!this->normalForcesMsgsAreInitialized)
  {
    this->InitializeNormalForcesMarkerMsgs(_positionMarkerMsg,
      _forceMarkerMsg);
    this->normalForcesMsgsAreInitialized = true;
  }

  // We need to compute the two points that form a normal force (start and end
  // points) with reference to the simulation origin
  gz::math::Vector3f normalForcePositionFromSensor(
    _position.X(), _position.Y(), _position.Z());

  gz::math::Quaternionf normalForceOrientationFromSensor;
  normalForceOrientationFromSensor.SetFrom2Axes(
    gz::math::Vector3f(0, 0, 1), _normalForce);

  gz::math::Pose3f normalForcePoseFromSensor(
    normalForcePositionFromSensor, normalForceOrientationFromSensor);

  gz::math::Pose3f normalForcePoseFromWorld = _sensorWorldPose *
    this->depthCameraOffset * normalForcePoseFromSensor;
  normalForcePoseFromWorld.Correct();

  // Get the start point of the normal force
  gz::math::Vector3f startPointFromWorld =
    normalForcePoseFromWorld.Pos();

  // Move the normal force pose a distance of forceLength along the direction
  // of _normalForce and get the end point
  normalForcePoseFromSensor.Set(normalForcePositionFromSensor +
    _normalForce * this->forceLength, normalForceOrientationFromSensor);

  normalForcePoseFromWorld = _sensorWorldPose * this->depthCameraOffset *
    normalForcePoseFromSensor;
  normalForcePoseFromWorld.Correct();

  gz::math::Vector3f endPointFromWorld =
    normalForcePoseFromWorld.Pos();

  // Check invalid points to avoid data transfer overhead
  if (abs(startPointFromWorld.Distance(endPointFromWorld)) < 1e-6)
    return;

  // Position
  gz::msgs::Set(_positionMarkerMsg.add_point(),
    gz::math::Vector3d(startPointFromWorld.X(),
      startPointFromWorld.Y(), startPointFromWorld.Z()));

  // Normal
  gz::msgs::Set(_forceMarkerMsg.add_point(),
    gz::math::Vector3d(startPointFromWorld.X(),
      startPointFromWorld.Y(), startPointFromWorld.Z()));
  gz::msgs::Set(_forceMarkerMsg.add_point(),
    gz::math::Vector3d(endPointFromWorld.X(),
      endPointFromWorld.Y(), endPointFromWorld.Z()));
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RequestNormalForcesMarkerMsgs(
  gz::msgs::Marker &_positionMarkerMsg,
  gz::msgs::Marker &_forceMarkerMsg)
{
  this->node.Request("/marker", _positionMarkerMsg);
  this->node.Request("/marker", _forceMarkerMsg);

  // Let the messages be initialized again
  this->normalForcesMsgsAreInitialized = false;
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RemoveNormalForcesAndContactsMarkers()
{
  gz::msgs::Marker positionMarkerMsg;
  gz::msgs::Marker forceMarkerMsg;
  gz::msgs::Marker contactMarkerMsg;

  positionMarkerMsg.set_ns("positions_" + this->modelName);
  positionMarkerMsg.set_action(gz::msgs::Marker::DELETE_ALL);

  forceMarkerMsg.set_ns("forces_" + this->modelName);
  forceMarkerMsg.set_action(gz::msgs::Marker::DELETE_ALL);

  contactMarkerMsg.set_ns("contacts_" + this->modelName);
  contactMarkerMsg.set_action(gz::msgs::Marker::DELETE_ALL);

  this->node.Request("/marker", positionMarkerMsg);
  this->node.Request("/marker", forceMarkerMsg);
  this->node.Request("/marker", contactMarkerMsg);
}
