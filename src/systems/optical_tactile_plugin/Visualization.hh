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
    /// \param[in] _contactRadius Value of the contactRadius attribute
    /// \param[in] _forceRadius Value of the forceRadius attribute
    /// \param[in] _forceLength Value of the forceLength attribute
    /// \param[in] _cameraUpdateRate Value of the cameraUpdateRate attribute
    /// \param[in] _depthCameraOffset Value of the depthCameraOffset attribute
    /// \param[in] _visualizationResolution Value of the
    /// visualizationResolution attribute
    public: OpticalTactilePluginVisualization(double &_contactRadius,
      double &_forceRadius, double &_forceLength, float &_cameraUpdateRate,
      ignition::math::Pose3f &_depthCameraOffset,
      int &_visualizationResolution);

    /// \brief Destructor
    public: ~OpticalTactilePluginVisualization();

    /// \brief Initialize the messages needed to request the /marker service
    /// \param[in] _modelName Name of the model
    /// \param[in] _sensorSize Size of the contact sensor
    public: void CreateMarkersMsgs(std::string &_modelName,
      ignition::math::Vector3d &_sensorSize);

    /// \brief Visualize the collision geometry of the sensor. This can be
    /// helpful when debbuging, given that there shouldn't be a visual tag
    /// in the plugin's model
    /// \param[in] _sensorPose Pose of the optical tactile sensor
    public: void VisualizeSensor(ignition::math::Pose3f &_sensorPose);

    /// \brief Visualize the normal forces computed for a (i,j) pixel of the
    /// image returned by the depth camera
    /// \param[in] _i First coordinate of the pixel
    /// \param[in] _j Second coordinate of the pixel
    /// \param[in] _position Position of the normal force referenced to the
    /// depth camera's origin
    /// \param[in] _normalForce Value of the normal force referenced to
    /// the depth camera's origin
    /// \param[in] _sensorWorldPose Pose of the plugin's model referenced to
    /// the world's origin
    public: void VisualizeNormalForce(uint64_t &_i, uint64_t &_j,
      ignition::math::Vector3f &_position,
      ignition::math::Vector3f &_normalForce,
      ignition::math::Pose3f &_sensorWorldPose);

    /// \brief Message for visualizing the sensor
    private: ignition::msgs::Marker sensorMarkerMsg;

    /// \brief Message for visualizing contacts positions
    private: ignition::msgs::Marker positionMarkerMsg;

    /// \brief Message for visualizing contacts forces
    private: ignition::msgs::Marker forceMarkerMsg;

    /// \brief Transport node to request the /marker service
    private: ignition::transport::Node node;

    /// \brief Radius of the visualized contact sphere
    private: double contactRadius;

    /// \brief Radius of the visualized force cylinder
    private: double forceRadius;

    /// \brief Length of the visualized force cylinder
    private: double forceLength;

    /// \brief Depth camera update rate
    private: float cameraUpdateRate;

    /// \brief Offset between depth camera pose and model pose
    private: ignition::math::Pose3f depthCameraOffset;

    /// \brief Resolution of the sensor in pixels to skip.
    private: int visualizationResolution;

    /// \brief Variable for setting the markers id
    private: int markerID;
  };
}
}
}
}
}

#endif
