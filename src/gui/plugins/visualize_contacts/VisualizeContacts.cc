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

#include "VisualizeContacts.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/marker.pb.h>

#include <string>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/rendering/RenderUtil.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for VisualizeContacts
  class VisualizeContactsPrivate
  {
    /// \brief Creates ContactSensorData for Collision components without a
    /// Contact Sensor by requesting the /enable_contact service
    /// \param[in] Reference to the GUI Entity Component Manager
    public: void CreateCollisionData(EntityComponentManager &_ecm);

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Current state of the checkbox
    public: bool checkboxState{false};

    /// \brief Previous state of the checkbox
    public: bool checkboxPrevState{false};

    /// \brief Message for visualizing contact positions
    public: gz::msgs::Marker positionMarkerMsg;

    /// \brief Radius of the visualized contact sphere
    public: double contactRadius{0.10};

    /// \brief Update time of the markers in milliseconds
    public: int64_t markerLifetime{200};

    /// \brief Simulation time for the last markers update
    public: std::chrono::steady_clock::duration lastMarkersUpdateTime{0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: checkboxState, contactRadius and markerLifetime
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;
  };
}
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
VisualizeContacts::VisualizeContacts()
  : GuiSystem(), dataPtr(new VisualizeContactsPrivate)
{
}

/////////////////////////////////////////////////
VisualizeContacts::~VisualizeContacts() = default;

/////////////////////////////////////////////////
void VisualizeContacts::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize contacts";

  // Configure Marker messages for position of the contacts

  // Blue spheres for positions

  // Create the marker message
  this->dataPtr->positionMarkerMsg.set_ns("positions");
  this->dataPtr->positionMarkerMsg.set_action(
    gz::msgs::Marker::ADD_MODIFY);
  this->dataPtr->positionMarkerMsg.set_type(
    gz::msgs::Marker::SPHERE);
  this->dataPtr->positionMarkerMsg.set_visibility(
    gz::msgs::Marker::GUI);
  this->dataPtr->
    positionMarkerMsg.mutable_lifetime()->
      set_sec(0);
  this->dataPtr->
    positionMarkerMsg.mutable_lifetime()->
      set_nsec(this->dataPtr->markerLifetime * 1000000);

  // Set material properties
  gz::msgs::Set(
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient(),
    gz::math::Color(0, 0, 1, 1));
  gz::msgs::Set(
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse(),
    gz::math::Color(0, 0, 1, 1));

  // Set contact position scale
  gz::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    gz::math::Vector3d(this->dataPtr->contactRadius,
    this->dataPtr->contactRadius,
    this->dataPtr->contactRadius));
}

/////////////////////////////////////////////////
void VisualizeContacts::OnVisualize(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->checkboxState = _checked;
}

//////////////////////////////////////////////////
void VisualizeContacts::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("VisualizeContacts::Update");

  if (!this->dataPtr->initialized)
  {
    // Get the name of the world
    if (this->dataPtr->worldName.empty())
    {
      _ecm.Each<components::World, components::Name>(
        [&](const Entity &,
            const components::World *,
            const components::Name *_name) -> bool
        {
          // We assume there's only one world
          this->dataPtr->worldName = _name->Data();
          return false;
        });
    }

    // Enable collisions
    this->dataPtr->CreateCollisionData(_ecm);
    this->dataPtr->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (this->dataPtr->checkboxPrevState && !this->dataPtr->checkboxState)
    {
      // Remove the markers
      this->dataPtr->positionMarkerMsg.set_action(
        gz::msgs::Marker::DELETE_ALL);

      gzdbg << "Removing markers..." << std::endl;
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->positionMarkerMsg);

      // Change action in case checkbox is checked again
      this->dataPtr->positionMarkerMsg.set_action(
        gz::msgs::Marker::ADD_MODIFY);
    }

    this->dataPtr->checkboxPrevState = this->dataPtr->checkboxState;
    if (!this->dataPtr->checkboxState)
      return;
  }

  // Only publish markers if enough time has passed
  auto timeDiff =
    std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime -
    this->dataPtr->lastMarkersUpdateTime);

  if (timeDiff.count() < this->dataPtr->markerLifetime)
    return;

  // Store simulation time
  this->dataPtr->lastMarkersUpdateTime = _info.simTime;

  // Get the contacts and publish them
  // Since we are setting a lifetime for the markers, we get all the
  // contacts instead of getting new and removed ones

  // Variable for setting the markers id through the iteration
  int markerID = 1;
  _ecm.Each<components::ContactSensorData>(
    [&](const Entity &,
        const components::ContactSensorData *_contacts) -> bool
    {
      for (const auto &contact : _contacts->Data().contact())
      {
        for (int i = 0; i < contact.position_size(); ++i)
        {
          // Set marker id, poses and request service
          this->dataPtr->positionMarkerMsg.set_id(markerID++);
          gz::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_pose(),
            gz::math::Pose3d(contact.position(i).x(),
              contact.position(i).y(), contact.position(i).z(),
              0, 0, 0));

          this->dataPtr->node.Request(
            "/marker", this->dataPtr->positionMarkerMsg);
        }
      }
      return true;
    });
}

//////////////////////////////////////////////////
void VisualizeContactsPrivate::CreateCollisionData(
                              EntityComponentManager &_ecm)
{
  // Collisions can't be enabled with _ecm given that this is a GUI plugin and
  // it doesn't run in the same process as the physics.
  // We use the world/<name>/enable_collision service instead.
  _ecm.Each<components::Collision>(
    [&](const Entity &_entity,
        const components::Collision *) -> bool
    {
      // Check if ContactSensorData has already been created
      bool collisionHasContactSensor =
        _ecm.EntityHasComponentType(_entity,
          components::ContactSensorData::typeId);

      if (collisionHasContactSensor)
      {
        gzdbg << "ContactSensorData detected in collision [" << _entity << "]"
          << std::endl;
        return true;
      }

      // Request service for enabling collision
      msgs::Entity req;
      req.set_id(_entity);
      req.set_type(msgs::Entity::COLLISION);

      msgs::Boolean res;
      bool result;
      unsigned int timeout = 50;
      std::string service = "/world/" + this->worldName + "/enable_collision";

      this->node.Request(service, req, timeout, res, result);

      return true;
    });
}

//////////////////////////////////////////////////
void VisualizeContacts::UpdateRadius(double _radius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->contactRadius = _radius;

  // Set scale
  gz::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    gz::math::Vector3d(this->dataPtr->contactRadius,
    this->dataPtr->contactRadius,
    this->dataPtr->contactRadius));
}

//////////////////////////////////////////////////
void VisualizeContacts::UpdatePeriod(double _period)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->markerLifetime = _period;

  // Set markers lifetime
  this->dataPtr->
    positionMarkerMsg.mutable_lifetime()->set_nsec(_period * 1000000);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeContacts,
                    gz::gui::Plugin)
