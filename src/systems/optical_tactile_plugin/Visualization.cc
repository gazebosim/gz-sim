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
  double &_contactRadius, double &_forceRadius, double &_forceLength,
  float &_cameraUpdateRate, ignition::math::Pose3f &_depthCameraOffset,
  int &_visualizationResolution) :
  contactRadius(_contactRadius), forceRadius(_forceRadius),
  forceLength(_forceLength), cameraUpdateRate(_cameraUpdateRate),
  depthCameraOffset(_depthCameraOffset),
  visualizationResolution(_visualizationResolution)
{
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::CreateMarkersMsgs(
  std::string &_modelName,
  ignition::math::Vector3d &_sensorSize)
{
  // Configure marker messages for position and force of the contacts, as well
  // as the marker for visualizing the sensor

  // Blue spheres for positions
  // Green cylinders for forces
  // Grey transparent box for the sensor

  this->positionMarkerMsg.set_ns("positions_" + _modelName);
  this->positionMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->positionMarkerMsg.set_type(ignition::msgs::Marker::SPHERE);
  this->positionMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

  this->forceMarkerMsg.set_ns("forces_" + _modelName);
  this->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->forceMarkerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  this->forceMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

  this->sensorMarkerMsg.set_ns("sensor_" + _modelName);
  this->sensorMarkerMsg.set_id(1);
  this->sensorMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->sensorMarkerMsg.set_type(ignition::msgs::Marker::BOX);
  this->sensorMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

  // Set material properties
  this->positionMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
  this->positionMarkerMsg.mutable_material()->mutable_ambient()->set_g(0);
  this->positionMarkerMsg.mutable_material()->mutable_ambient()->set_b(1);
  this->positionMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
  this->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
  this->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_g(0);
  this->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_b(1);
  this->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
  this->positionMarkerMsg.mutable_lifetime()->set_sec(this->cameraUpdateRate);

  this->forceMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
  this->forceMarkerMsg.mutable_material()->mutable_ambient()->set_g(1);
  this->forceMarkerMsg.mutable_material()->mutable_ambient()->set_b(0);
  this->forceMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
  this->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
  this->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_g(1);
  this->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  this->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
  this->forceMarkerMsg.mutable_lifetime()->set_sec(this->cameraUpdateRate);

  this->sensorMarkerMsg.mutable_material()->mutable_ambient()->set_r(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_ambient()->set_g(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_ambient()->set_b(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_ambient()->set_a(0.75);
  this->sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_g(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_b(0.5);
  this->sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_a(0.75);
  this->sensorMarkerMsg.mutable_lifetime()->set_sec(0);
  this->sensorMarkerMsg.mutable_lifetime()->set_nsec(0);

  // Set scales
  ignition::msgs::Set(this->positionMarkerMsg.mutable_scale(),
    ignition::math::Vector3d(this->contactRadius, this->contactRadius,
    this->contactRadius));

  ignition::msgs::Set(this->forceMarkerMsg.mutable_scale(),
    ignition::math::Vector3d(this->forceRadius, this->forceRadius,
    this->forceLength));

  ignition::msgs::Set(this->sensorMarkerMsg.mutable_scale(), _sensorSize);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::VisualizeSensor(
  ignition::math::Pose3f &_sensorPose)
{
  this->sensorMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->sensorMarkerMsg.set_id(1);

  this->sensorMarkerMsg.mutable_pose()->mutable_position()->set_x(
    _sensorPose.Pos().X());
  this->sensorMarkerMsg.mutable_pose()->mutable_position()->set_y(
    _sensorPose.Pos().Y());
  this->sensorMarkerMsg.mutable_pose()->mutable_position()->set_z(
    _sensorPose.Pos().Z());
  this->sensorMarkerMsg.mutable_pose()->mutable_orientation()->set_x(
    _sensorPose.Rot().X());
  this->sensorMarkerMsg.mutable_pose()->mutable_orientation()->set_y(
    _sensorPose.Rot().Y());
  this->sensorMarkerMsg.mutable_pose()->mutable_orientation()->set_z(
    _sensorPose.Rot().Z());
  this->sensorMarkerMsg.mutable_pose()->mutable_orientation()->set_w(
    _sensorPose.Rot().W());

  this->node.Request("/marker", this->sensorMarkerMsg);
}

//////////////////////////////////////////////////
void OpticalTactilePluginVisualization::VisualizeNormalForce(
  uint64_t &_i, uint64_t &_j,
  ignition::math::Vector3f &_position,
  ignition::math::Vector3f &_normalForce,
  ignition::math::Pose3f &_sensorWorldPose)
{
  // Visualization is downsampled according to plugin parameter
  // <visualization_resolution> if <visualize_forces> is set to true

  // Check if _normalForce has to be visualized
  bool visualizeForce = (_i % this->visualizationResolution == 0 &&
    _j % this->visualizationResolution == 0) ||
    (_i % this->visualizationResolution == 0 && _j == 1) ||
    (_i == 1 && _j % this->visualizationResolution == 0) ||
    (_i == 1 && _j == 1);

  if (!visualizeForce)
    return;

  if (_i == 1 && _j == 1)
    this->markerID = 1;

  // Add new markers with specific lifetime
  this->positionMarkerMsg.set_id(this->markerID);
  this->positionMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->positionMarkerMsg.mutable_lifetime()->set_sec(this->cameraUpdateRate);
  this->positionMarkerMsg.mutable_lifetime()->set_nsec(0);

  this->forceMarkerMsg.set_id(this->markerID++);
  this->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->forceMarkerMsg.mutable_lifetime()->set_sec(this->cameraUpdateRate);
  this->forceMarkerMsg.mutable_lifetime()->set_nsec(0);

  // The pose of the  markers must be published with reference
  // to the simulation origin
  ignition::math::Vector3f forceMarkerSensorPosition(
    _position.X(), _position.Y(), _position.Z());

  ignition::math::Quaternionf forceMarkerSensorQuaternion;
  forceMarkerSensorQuaternion.From2Axes(
    ignition::math::Vector3f(0, 0, 1), _normalForce);

  // The position of the force marker is modified in order to place the
  // end of the cylinder in the surface, not its middle point
  ignition::math::Pose3f forceMarkerSensorPose(forceMarkerSensorPosition +
    _normalForce * (this->forceLength/2), forceMarkerSensorQuaternion);

  ignition::math::Pose3f forceMarkerWorldPose =
    (forceMarkerSensorPose + this->depthCameraOffset) + _sensorWorldPose;
  forceMarkerWorldPose.Correct();

  ignition::math::Pose3f positionMarkerSensorPose(
    forceMarkerSensorPosition, forceMarkerSensorQuaternion);
  ignition::math::Pose3f positionMarkerWorldPose =
    (positionMarkerSensorPose + this->depthCameraOffset) + _sensorWorldPose;
  positionMarkerWorldPose.Correct();

  // Set markers pose messages from previously computed poses
  this->forceMarkerMsg.mutable_pose()->mutable_position()->set_x(
    forceMarkerWorldPose.Pos().X());
  this->forceMarkerMsg.mutable_pose()->mutable_position()->set_y(
    forceMarkerWorldPose.Pos().Y());
  this->forceMarkerMsg.mutable_pose()->mutable_position()->set_z(
    forceMarkerWorldPose.Pos().Z());
  this->forceMarkerMsg.mutable_pose()->mutable_orientation()->set_x(
    forceMarkerWorldPose.Rot().X());
  this->forceMarkerMsg.mutable_pose()->mutable_orientation()->set_y(
    forceMarkerWorldPose.Rot().Y());
  this->forceMarkerMsg.mutable_pose()->mutable_orientation()->set_z(
    forceMarkerWorldPose.Rot().Z());
  this->forceMarkerMsg.mutable_pose()->mutable_orientation()->set_w(
    forceMarkerWorldPose.Rot().W());

  this->positionMarkerMsg.mutable_pose()->mutable_position()->set_x(
    positionMarkerWorldPose.Pos().X());
  this->positionMarkerMsg.mutable_pose()->mutable_position()->set_y(
    positionMarkerWorldPose.Pos().Y());
  this->positionMarkerMsg.mutable_pose()->mutable_position()->set_z(
    positionMarkerWorldPose.Pos().Z());
  this->positionMarkerMsg.mutable_pose()->mutable_orientation()->set_x(
    positionMarkerWorldPose.Rot().X());
  this->positionMarkerMsg.mutable_pose()->mutable_orientation()->set_y(
    positionMarkerWorldPose.Rot().Y());
  this->positionMarkerMsg.mutable_pose()->mutable_orientation()->set_z(
    positionMarkerWorldPose.Rot().Z());
  this->positionMarkerMsg.mutable_pose()->mutable_orientation()->set_w(
    positionMarkerWorldPose.Rot().W());

  this->node.Request("/marker", this->forceMarkerMsg);
  this->node.Request("/marker", this->positionMarkerMsg);
}
}
}
}
}
}
