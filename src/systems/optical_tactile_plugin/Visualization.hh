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

#ifndef IGNITION_GAZEBO_SYSTEMS_OPTICAL_TACTILE_PLUGIN_VISUALIZATION_HH_
#define IGNITION_GAZEBO_SYSTEMS_OPTICAL_TACTILE_PLUGIN_VISUALIZATION_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

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
  // This class handles the configuration and process of visualizing the
  // different elements needed for the Optical Tactile Sensor plugin
  class OpticalTactilePluginVisualization
  {
    /// \brief Constructor
    /// \param[in] _modelName Name of the model to which the sensor is attached
    /// \param[in] _sensorSize Size of the contact sensor
    /// \param[in] _forceLength Value of the forceLength attribute
    /// \param[in] _cameraUpdateRate Value of the cameraUpdateRate attribute
    /// \param[in] _depthCameraOffset Value of the depthCameraOffset attribute
    /// \param[in] _visualizationResolution Value of the
    /// visualizationResolution attribute
    public: OpticalTactilePluginVisualization(
      std::string &_modelName,
      ignition::math::Vector3d &_sensorSize,
      double &_forceLength,
      float &_cameraUpdateRate,
      ignition::math::Pose3f &_depthCameraOffset,
      int &_visualizationResolution);

    /// \brief Destructor
    public: ~OpticalTactilePluginVisualization();

    /// \brief Initialize the marker message representing the optical tactile
    /// sensor
    /// \param[out] _sensorMarkerMsg Message for visualizing the sensor
    private: void InitializeSensorMarkerMsg(
      ignition::msgs::Marker &_sensorMarkerMsg);

    /// \brief Request the "/marker" service for the sensor marker.
    /// This can be helpful when debbuging, given that there shouldn't be a
    /// visual tag in the plugin's model
    /// \param[in] _sensorPose Pose of the optical tactile sensor
    public: void RequestSensorMarkerMsg(
      ignition::math::Pose3f const &_sensorPose);

    /// \brief Initialize the marker messages representing the normal forces
    /// \param[out] _positionMarkerMsg Message for visualizing the contact
    /// positions
    /// \param[out] _forceMarkerMsg Message for visualizing the contact
    /// normal forces
    private: void InitializeNormalForcesMarkerMsgs(
      ignition::msgs::Marker &_positionMarkerMsg,
      ignition::msgs::Marker &_forceMarkerMsg);

    /// \brief Add a normal force to the marker messages representing the
    /// normal forces computed
    /// \param[out] _positionMarkerMsg Message for visualizing the contact
    /// positions
    /// \param[out] _forceMarkerMsg Message for visualizing the contact
    /// normal forces
    /// \param[in] _position Position of the normal force referenced to the
    /// depth camera's origin
    /// \param[in] _normalForce Value of the normal force referenced to
    /// the depth camera's origin
    /// \param[in] _sensorWorldPose Pose of the plugin's model referenced to
    /// the world's origin
    public: void AddNormalForceToMarkerMsgs(
      ignition::msgs::Marker &_positionMarkerMsg,
      ignition::msgs::Marker &_forceMarkerMsg,
      ignition::math::Vector3f &_position,
      ignition::math::Vector3f &_normalForce,
      ignition::math::Pose3f &_sensorWorldPose);

    /// \brief Request the "/marker" service for the marker messages
    /// representing the normal forces
    /// \param[in] _positionMarkerMsg Message for visualizing the contact
    /// positions
    /// \param[in] _forceMarkerMsg Message for visualizing the contact
    /// normal forces
    public: void RequestNormalForcesMarkerMsgs(
      ignition::msgs::Marker &_positionMarkerMsg,
      ignition::msgs::Marker &_forceMarkerMsg);

    /// \brief Transport node to request the /marker service
    private: ignition::transport::Node node;

    /// \brief Name of the model to which the sensor is attached
    private: std::string modelName;

    /// \brief Size of the contact sensor
    private: ignition::math::Vector3d sensorSize;

    /// \brief Length of the visualized force cylinder
    private: double forceLength;

    /// \brief Depth camera update rate
    private: float cameraUpdateRate;

    /// \brief Offset between depth camera pose and model pose
    private: ignition::math::Pose3f depthCameraOffset;

    /// \brief Resolution of the sensor in pixels to skip.
    private: int visualizationResolution;

    /// \brief Whether the normal forces messages are initialized or not
    private: bool normalForcesMsgsAreInitialized{false};
  };
}
}
}
}
}

#endif
