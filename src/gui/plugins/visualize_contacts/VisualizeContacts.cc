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

#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/contact.pb.h>

#include <string>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Uuid.hh>
#include <ignition/common/VideoEncoder.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"

#include "VisualizeContacts.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
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
    public: ignition::msgs::Marker positionMarkerMsg;

    /// \brief Radius of the visualized contact sphere
    public: double contactRadius{0.10};

    /// \brief Message for visualizing contact forces
    public: ignition::msgs::Marker forceMarkerMsg;

    /// \brief Scale of the visualized force cylinder. By scale we mean how
    /// many meters per newton the marker will be long
    public: double forceScale{0.40};

    /// \brief Update time of the markers in milliseconds
    public: int64_t markerLifetime{200};

    /// \brief Simulation time for the last markers update
    public: std::chrono::steady_clock::duration lastMarkersUpdateTime{0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: checkboxState, contactRadius, forceScale and
    /// markerLifetime
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

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

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);

  // Configure Marker messages for position and force of the contacts

  // Blue spheres for positions
  // Green cylinders for forces

  // Create the marker message
  this->dataPtr->positionMarkerMsg.set_ns("positions");
  this->dataPtr->positionMarkerMsg.set_action(
    ignition::msgs::Marker::ADD_MODIFY);
  this->dataPtr->positionMarkerMsg.set_type(
    ignition::msgs::Marker::SPHERE);
  this->dataPtr->positionMarkerMsg.set_visibility(
    ignition::msgs::Marker::GUI);
  this->dataPtr->
    positionMarkerMsg.mutable_lifetime()->
      set_sec(0);
  this->dataPtr->
    positionMarkerMsg.mutable_lifetime()->
      set_nsec(this->dataPtr->markerLifetime * 1000000);

  this->dataPtr->forceMarkerMsg.set_ns("forces");
  this->dataPtr->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  this->dataPtr->forceMarkerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  this->dataPtr->forceMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);
  this->dataPtr->forceMarkerMsg.mutable_lifetime()->set_sec(0);
  this->dataPtr->forceMarkerMsg.mutable_lifetime()->
    set_nsec(this->dataPtr->markerLifetime * 1000000);

  // Set material properties
  ignition::msgs::Set(
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient(),
    ignition::math::Color(0, 0, 1, 1));
  ignition::msgs::Set(
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse(),
    ignition::math::Color(0, 0, 1, 1));

  ignition::msgs::Set(
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_ambient(),
    ignition::math::Color(0, 1, 0, 1));
  ignition::msgs::Set(
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_diffuse(),
    ignition::math::Color(0, 1, 0, 1));

  // Set contact position scale
  ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    ignition::math::Vector3d(this->dataPtr->contactRadius,
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
  IGN_PROFILE("VisualizeContacts::Update");

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
        ignition::msgs::Marker::DELETE_ALL);
      this->dataPtr->forceMarkerMsg.set_action(
        ignition::msgs::Marker::DELETE_ALL);

      igndbg << "Removing markers..." << std::endl;
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->positionMarkerMsg);
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->forceMarkerMsg);

      // Change action in case checkbox is checked again
      this->dataPtr->positionMarkerMsg.set_action(
        ignition::msgs::Marker::ADD_MODIFY);
      this->dataPtr->forceMarkerMsg.set_action(
        ignition::msgs::Marker::ADD_MODIFY);
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

  // todo(anyone) Get the contacts of the links that don't have a
  // contact sensor

  // Get the contacts and publish them
  // Since we are setting a lifetime for the markers, we get all the
  // contacts instead of getting news and removed ones

  // Variable for setting the markers id through the iteration
  int markerID = 1;
  _ecm.Each<components::ContactSensorData>(
    [&](const Entity &,
        const components::ContactSensorData *_contacts) -> bool
    {
      for (const auto &contact : _contacts->Data().contact())
      {
        // todo(anyone) add information about contact normal, depth
        // and wrench
        for (int i = 0; i < contact.position_size(); ++i)
        {
          // Skip dummy data set by physics
          bool dataIsDummy = std::fabs(
            contact.position(i).x() -
            static_cast<float>(ignition::math::NAN_I)) < 0.001;
          if (dataIsDummy)
            return true;

          // Set marker id, poses and request service
          this->dataPtr->positionMarkerMsg.set_id(markerID);
          ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_pose(),
            ignition::math::Pose3d(contact.position(i).x(),
              contact.position(i).y(), contact.position(i).z(),
              0, 0, 0));

          // Placeholder for the force value (see todo comment above)
          double force = 1;

          double forceLength = this->dataPtr->forceScale * force;
          ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_scale(),
            ignition::math::Vector3d(0.02, 0.02, forceLength));

          // The position of the force marker is modified in order to place the
          // end of the cylinder in the contact point, not its middle point
          this->dataPtr->forceMarkerMsg.set_id(markerID++);
          ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_pose(),
            ignition::math::Pose3d(contact.position(i).x(),
              contact.position(i).y(),
              contact.position(i).z() + forceLength/2,
              0, 0, 0));

          this->dataPtr->node.Request(
            "/marker", this->dataPtr->positionMarkerMsg);
          this->dataPtr->node.Request(
            "/marker", this->dataPtr->forceMarkerMsg);
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
        igndbg << "ContactSensorData detected in collision [" << _entity << "]"
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
void VisualizeContacts::UpdateScale(double _scale)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->forceScale = _scale;
}

//////////////////////////////////////////////////
void VisualizeContacts::UpdateRadius(double _radius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->contactRadius = _radius;

  // Set scale
  ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    ignition::math::Vector3d(this->dataPtr->contactRadius,
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

  this->dataPtr->
    forceMarkerMsg.mutable_lifetime()->set_nsec(_period * 1000000);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeContacts,
                    ignition::gui::Plugin)
