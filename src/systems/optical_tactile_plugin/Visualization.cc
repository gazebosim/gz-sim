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

#include <ignition/transport/Node.hh>

#include "Visualization.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
namespace optical_tactile_sensor
{

//////////////////////////////////////////////////
OpticalTactilePluginVisualization::OpticalTactilePluginVisualization(
  std::string &_modelName,
  ignition::math::Vector3d &_sensorSize,
  double &_forceLength,
  float &_cameraUpdateRate,
  ignition::math::Pose3f &_depthCameraOffset,
  int &_visualizationResolution) :
  modelName(_modelName),
  sensorSize(_sensorSize),
  forceLength(_forceLength),
  cameraUpdateRate(_cameraUpdateRate),
  depthCameraOffset(_depthCameraOffset),
  visualizationResolution(_visualizationResolution)
{
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::InitializeSensorMarkerMsg(
  ignition::msgs::Marker &_sensorMarkerMsg)
{
  // Initialize the marker for visualizing the sensor as a grey transparent box

  _sensorMarkerMsg.set_ns("sensor_" + this->modelName);
  _sensorMarkerMsg.set_id(1);
  _sensorMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  _sensorMarkerMsg.set_type(ignition::msgs::Marker::BOX);
  _sensorMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);
  ignition::msgs::Set(_sensorMarkerMsg.mutable_scale(), this->sensorSize);

  // Set material properties
  ignition::msgs::Set(_sensorMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0.5, 0.5, 0.5, 0.75));
  ignition::msgs::Set(_sensorMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0.5, 0.5, 0.5, 0.75));
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RequestSensorMarkerMsg(
  ignition::math::Pose3f const &_sensorPose)
{
  ignition::msgs::Marker sensorMarkerMsg;

  this->InitializeSensorMarkerMsg(sensorMarkerMsg);

  ignition::msgs::Set(sensorMarkerMsg.mutable_pose(),
    ignition::math::Pose3d(_sensorPose.Pos().X(),
      _sensorPose.Pos().Y(), _sensorPose.Pos().Z(),
      _sensorPose.Rot().W(), _sensorPose.Rot().X(),
      _sensorPose.Rot().Y(), _sensorPose.Rot().Z()));

  this->node.Request("/marker", sensorMarkerMsg);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::InitializeNormalForcesMarkerMsgs(
  ignition::msgs::Marker &_positionMarkerMsg,
  ignition::msgs::Marker &_forceMarkerMsg)
{
  // Initialize marker messages for position and force of the contacts

  // Blue points for positions
  // Green lines for forces

  _positionMarkerMsg.set_ns("positions_" + this->modelName);
  _positionMarkerMsg.set_id(1);
  _positionMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  _positionMarkerMsg.set_type(ignition::msgs::Marker::POINTS);
  _positionMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);
  ignition::msgs::Set(_positionMarkerMsg.mutable_scale(),
    ignition::math::Vector3d(1, 1, 1));

  _forceMarkerMsg.set_ns("forces_" + this->modelName);
  _forceMarkerMsg.set_id(1);
  _forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  _forceMarkerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  _forceMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

  // Set material properties and lifetime
  ignition::msgs::Set(_positionMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0, 0, 1, 1));
  ignition::msgs::Set(_positionMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0, 0, 1, 1));
  _positionMarkerMsg.mutable_lifetime()->set_nsec(
    this->cameraUpdateRate * 1000000000);

  ignition::msgs::Set(_forceMarkerMsg.mutable_material()->
    mutable_ambient(), math::Color(0, 1, 0, 1));
  ignition::msgs::Set(_forceMarkerMsg.mutable_material()->
    mutable_diffuse(), math::Color(0, 1, 0, 1));
  _forceMarkerMsg.mutable_lifetime()->set_sec(
    this->cameraUpdateRate * 1000000000);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::AddNormalForceToMarkerMsgs(
  ignition::msgs::Marker &_positionMarkerMsg,
  ignition::msgs::Marker &_forceMarkerMsg,
  ignition::math::Vector3f &_position,
  ignition::math::Vector3f &_normalForce,
  ignition::math::Pose3f &_sensorWorldPose)
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
  ignition::math::Vector3f normalForcePositionFromSensor(
    _position.X(), _position.Y(), _position.Z());

  ignition::math::Quaternionf normalForceOrientationFromSensor;
  normalForceOrientationFromSensor.From2Axes(
    ignition::math::Vector3f(0, 0, 1), _normalForce);

  ignition::math::Pose3f normalForcePoseFromSensor(
    normalForcePositionFromSensor, normalForceOrientationFromSensor);

  ignition::math::Pose3f normalForcePoseFromWorld =
    (normalForcePoseFromSensor + this->depthCameraOffset) + _sensorWorldPose;
  normalForcePoseFromWorld.Correct();

  // Get the first point of the normal force
  ignition::math::Vector3f firstPointFromWorld =
    normalForcePoseFromWorld.Pos();

  // Move the normal force pose a distance of forceLength along the direction
  // of _normalForce and get the second point
  normalForcePoseFromSensor.Set(normalForcePositionFromSensor +
    _normalForce * this->forceLength, normalForceOrientationFromSensor);

  normalForcePoseFromWorld =
    (normalForcePoseFromSensor + this->depthCameraOffset) + _sensorWorldPose;
  normalForcePoseFromWorld.Correct();

  ignition::math::Vector3f secondPointFromWorld =
    normalForcePoseFromWorld.Pos();

  // Check invalid points to avoid data transfer overhead
  if (firstPointFromWorld == ignition::math::Vector3f(0.0, 0.0, 0.0) ||
      secondPointFromWorld == ignition::math::Vector3f(0.0, 0.0, 0.0))
    return;

  // Add points to the messages

  ignition::msgs::Set(_positionMarkerMsg.add_point(),
    ignition::math::Vector3d(firstPointFromWorld.X(),
      firstPointFromWorld.Y(), firstPointFromWorld.Z()));

  ignition::msgs::Set(_forceMarkerMsg.add_point(),
    ignition::math::Vector3d(firstPointFromWorld.X(),
      firstPointFromWorld.Y(), firstPointFromWorld.Z()));

  ignition::msgs::Set(_forceMarkerMsg.add_point(),
    ignition::math::Vector3d(secondPointFromWorld.X(),
      secondPointFromWorld.Y(), secondPointFromWorld.Z()));
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::RequestNormalForcesMarkerMsgs(
  ignition::msgs::Marker &_positionMarkerMsg,
  ignition::msgs::Marker &_forceMarkerMsg)
{
  this->node.Request("/marker", _forceMarkerMsg);
  this->node.Request("/marker", _positionMarkerMsg);

  // Let the messages be initialized again
  this->normalForcesMsgsAreInitialized = false;
}

}
}
}
}
}
