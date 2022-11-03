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

#ifndef GZ_SIM_SYSTEMS_OPTICAL_TACTILE_PLUGIN_VISUALIZATION_HH_
#define GZ_SIM_SYSTEMS_OPTICAL_TACTILE_PLUGIN_VISUALIZATION_HH_

#include <memory>
#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/marker.pb.h>

#include "gz/sim/components/ContactSensorData.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace optical_tactile_sensor
{
  // This class handles the configuration and process of visualizing the
  // different elements needed for the Optical Tactile Sensor plugin.
  // Terminology:
  // "Contacts" refers to data from the contact sensor based on physics.
  // "Normal forces" refers to post-processed data from the depth camera based
  // purely on imagery.
  // These two sets of data are currently disjoint and visualized separately.
  class OpticalTactilePluginVisualization
  {
    /// \brief Constructor
    /// \param[in] _modelName Name of the model to which the sensor is attached
    /// \param[in] _sensorSize Size of the contact sensor
    /// \param[in] _forceLength Value of the forceLength attribute
    /// \param[in] _cameraUpdateRate Value of the cameraUpdateRate attribute
    /// \param[in] _depthCameraOffset Value of the depthCameraOffset attribute
    public: OpticalTactilePluginVisualization(
        std::string &_modelName,
        gz::math::Vector3d &_sensorSize,
        double &_forceLength,
        float &_cameraUpdateRate,
        gz::math::Pose3f &_depthCameraOffset);

    /// \brief Initialize the marker message representing the optical tactile
    /// sensor
    /// \param[out] _sensorMarkerMsg Message for visualizing the sensor
    private: void InitializeSensorMarkerMsg(
        gz::msgs::Marker &_sensorMarkerMsg);

    /// \brief Request the "/marker" service for the sensor marker.
    /// This can be helpful when debbuging, given that there shouldn't be a
    /// visual tag in the plugin's model
    /// \param[in] _sensorPose Pose of the optical tactile sensor
    public: void RequestSensorMarkerMsg(
        gz::math::Pose3f const &_sensorPose);

    /// \brief Initialize the marker message representing the contacts from the
    /// contact sensor
    /// \param[out] _contactsMarkerMsg Message for visualizing the contacts
    private: void InitializeContactsMarkerMsg(
        gz::msgs::Marker &_contactsMarkerMsg);

    /// \brief Add a contact to the marker message representing the contacts
    /// from the contact sensor based on physics
    /// \param[in] _contact Contact to be added
    /// \param[out] _contactsMarkerMsg Message for visualizing the contacts
    public: void AddContactToMarkerMsg(
        gz::msgs::Contact const &_contact,
        gz::msgs::Marker &_contactsMarkerMsg);

    /// \brief Request the "/marker" service for the contacts marker.
    /// \param[in] _contacts Contacts to visualize
    public: void RequestContactsMarkerMsg(
        components::ContactSensorData const *_contacts);

    /// \brief Initialize the marker messages representing the normal forces
    /// \param[out] _positionMarkerMsg Message for visualizing the contact
    /// positions
    /// \param[out] _forceMarkerMsg Message for visualizing the contact
    /// normal forces
    private: void InitializeNormalForcesMarkerMsgs(
        gz::msgs::Marker &_positionMarkerMsg,
        gz::msgs::Marker &_forceMarkerMsg);

    /// \brief Create a marker messages representing the normal force computed
    /// from depth camera
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
        gz::msgs::Marker &_positionMarkerMsg,
        gz::msgs::Marker &_forceMarkerMsg,
        gz::math::Vector3f &_position,
        gz::math::Vector3f &_normalForce,
        gz::math::Pose3f &_sensorWorldPose);

    /// \brief Request the "/marker" service for the marker messages
    /// representing the normal forces
    /// \param[in] _positionMarkerMsg Message for visualizing the contact
    /// positions
    /// \param[in] _forceMarkerMsg Message for visualizing the contact
    /// normal forces
    public: void RequestNormalForcesMarkerMsgs(
        gz::msgs::Marker &_positionMarkerMsg,
        gz::msgs::Marker &_forceMarkerMsg);

    /// \brief Remove all normal forces and contact markers
    public: void RemoveNormalForcesAndContactsMarkers();

    /// \brief Transport node to request the /marker service
    private: gz::transport::Node node;

    /// \brief Name of the model to which the sensor is attached
    private: std::string modelName;

    /// \brief Size of the contact sensor
    private: gz::math::Vector3d sensorSize;

    /// \brief Length of the visualized force cylinder
    private: double forceLength;

    /// \brief Depth camera update rate
    private: float cameraUpdateRate;

    /// \brief Offset between depth camera pose and model pose
    private: gz::math::Pose3f depthCameraOffset;

    /// \brief Whether the normal forces messages are initialized or not
    private: bool normalForcesMsgsAreInitialized{false};
  };
}  // namespace optical_tactile_sensor
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
