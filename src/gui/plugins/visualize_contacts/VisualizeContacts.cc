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
#include <gz/msgs/marker_v.pb.h>

#include <algorithm>
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
namespace
{
//////////////////////////////////////////////////
void OnMarkerArrayResponse(const gz::msgs::Boolean &, const bool)
{
}

constexpr gz::math::Color kContactColor =
    gz::math::Color::UnclampedColor(0.0f, 0.0f, 1.0f, 1.0f);

constexpr gz::math::Color kForceArrowAmbient =
    gz::math::Color::UnclampedColor(1.0f, 0.8f, 0.0f, 1.0f);
constexpr gz::math::Color kForceArrowDiffuse =
    gz::math::Color::UnclampedColor(1.0f, 0.9f, 0.1f, 1.0f);
constexpr gz::math::Color kForceArrowEmissive =
    gz::math::Color::UnclampedColor(0.9f, 0.5f, 0.0f, 1.0f);

constexpr double kMinimumForceToVisualize = 1e-2;

//////////////////////////////////////////////////
/// \brief Helper function to add force arrow body and head markers to a marker
/// array
/// \param[out] _markerMsgs Marker array to append markers to
/// \param[in] _forceMarkerId ID for the force arrow markers
/// \param[in] _pos Contact position in world coordinates
/// \param[in] _force Contact force vector in world coordinates
/// \param[in] _forceScale Scale factor to convert force magnitude to arrow length
/// \param[in] _arrowRadius Radius of the arrow body cylinder
/// \param[in] _bodyMarkerMsg Template marker message for arrow body
/// \param[in] _headMarkerMsg Template marker message for arrow head
void AddForceArrowMarkers(
    gz::msgs::Marker_V &_markerMsgs,
    int _forceMarkerId,
    const gz::math::Vector3d &_pos,
    const gz::math::Vector3d &_force,
    double _forceScale,
    double _arrowRadius,
    const gz::msgs::Marker &_bodyMarkerMsg,
    const gz::msgs::Marker &_headMarkerMsg)
{
  double fNorm = _force.Length();
  if (fNorm <= kMinimumForceToVisualize)
    return;

  double L = fNorm * _forceScale;
  gz::math::Vector3d u = _force / fNorm;

  double bodyDiam = 2.0 * _arrowRadius;
  double headDiam = 2.2 * bodyDiam;
  // Clamp head and body lens to prevent small scale rendering artifacts.
  double headLen = std::min(0.4 * L, std::max(0.01, 2.5 * bodyDiam));
  double bodyLen = std::max(0.001, L - headLen);

  // Default cylinder and cone marker shapes are aligned with +Z.
  // Compute rotation from +Z to the unit force direction vector u.
  gz::math::Quaterniond rot;
  rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, u);

  // 1. Body (Cylinder)
  gz::math::Vector3d pBody = _pos + (0.5 * bodyLen) * u;
  auto bodyMarker = _markerMsgs.add_marker();
  bodyMarker->CopyFrom(_bodyMarkerMsg);
  bodyMarker->set_id(_forceMarkerId);
  gz::msgs::Set(bodyMarker->mutable_pose(),
    gz::math::Pose3d(pBody, rot));
  gz::msgs::Set(bodyMarker->mutable_scale(),
    gz::math::Vector3d(bodyDiam, bodyDiam, bodyLen));

  // 2. Head (Cone)
  gz::math::Vector3d pHead = _pos + (bodyLen + 0.5 * headLen) * u;
  auto headMarker = _markerMsgs.add_marker();
  headMarker->CopyFrom(_headMarkerMsg);
  headMarker->set_id(_forceMarkerId);
  gz::msgs::Set(headMarker->mutable_pose(),
    gz::math::Pose3d(pHead, rot));
  gz::msgs::Set(headMarker->mutable_scale(),
    gz::math::Vector3d(headDiam, headDiam, headLen));
}
}  // namespace

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

    /// \brief State of the show forces checkbox
    public: bool showForcesState{false};

    /// brief Scale of force vectors in m/N
    public: double forceScale{0.01};

    /// \brief Radius of the force arrow in meters
    public: double arrowRadius{0.01};

    /// \brief Message template for contact positions (spheres)
    public: gz::msgs::Marker positionMarkerMsg;

    /// \brief Message template for cylinder of force arrows
    public: gz::msgs::Marker arrowBodyMarkerMsg;

    /// \brief Message template for cone heads of force arrows
    public: gz::msgs::Marker arrowHeadMarkerMsg;

    /// \brief Radius of the visualized contact sphere in meters
    public: double sphereRadius{0.10};

    /// \brief Update time of the markers in milliseconds
    public: int64_t markerLifetime{200};

    /// \brief Simulation time for the last markers update
    public: std::chrono::steady_clock::duration lastMarkersUpdateTime{0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: checkboxState, showForcesState, sphereRadius,
    /// forceScale and markerLifetime
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

  // Configure Marker messages for position and forces of contacts.
  // Spheres for contact positions, cylinders for force arrow body and
  // cone for force arrow head.

  // Create the contact sphere marker message
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
    kContactColor);
  gz::msgs::Set(
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse(),
    kContactColor);

  // Set contact position scale
  gz::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    gz::math::Vector3d(this->dataPtr->sphereRadius,
    this->dataPtr->sphereRadius,
    this->dataPtr->sphereRadius));

  // Create the force arrow body marker message.
  // Scale is not assigned here, it is updated dynamically in `Update`.
  this->dataPtr->arrowBodyMarkerMsg.set_ns("force_arrow_bodies");
  this->dataPtr->arrowBodyMarkerMsg.set_action(
    gz::msgs::Marker::ADD_MODIFY);
  this->dataPtr->arrowBodyMarkerMsg.set_type(
    gz::msgs::Marker::CYLINDER);
  this->dataPtr->arrowBodyMarkerMsg.set_visibility(
    gz::msgs::Marker::GUI);
  this->dataPtr->
    arrowBodyMarkerMsg.mutable_lifetime()->
      set_sec(0);
  this->dataPtr->
    arrowBodyMarkerMsg.mutable_lifetime()->
      set_nsec(this->dataPtr->markerLifetime * 1000000);

  // Set material properties
  gz::msgs::Set(
    this->dataPtr->arrowBodyMarkerMsg.mutable_material()->mutable_ambient(),
    kForceArrowAmbient);
  gz::msgs::Set(
    this->dataPtr->arrowBodyMarkerMsg.mutable_material()->mutable_diffuse(),
    kForceArrowDiffuse);
  gz::msgs::Set(
    this->dataPtr->arrowBodyMarkerMsg.mutable_material()->mutable_emissive(),
    kForceArrowEmissive);

  // Create the force arrow head marker message.
  // Scale is not assigned here, it is updated dynamically in `Update`.
  this->dataPtr->arrowHeadMarkerMsg.set_ns("force_arrow_heads");
  this->dataPtr->arrowHeadMarkerMsg.set_action(
    gz::msgs::Marker::ADD_MODIFY);
  this->dataPtr->arrowHeadMarkerMsg.set_type(
    gz::msgs::Marker::CONE);
  this->dataPtr->arrowHeadMarkerMsg.set_visibility(
    gz::msgs::Marker::GUI);
  this->dataPtr->
    arrowHeadMarkerMsg.mutable_lifetime()->
      set_sec(0);
  this->dataPtr->
    arrowHeadMarkerMsg.mutable_lifetime()->
      set_nsec(this->dataPtr->markerLifetime * 1000000);

  this->dataPtr->arrowHeadMarkerMsg.mutable_material()->CopyFrom(
    this->dataPtr->arrowBodyMarkerMsg.material());
}

/////////////////////////////////////////////////
void VisualizeContacts::OnVisualize(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->checkboxState = _checked;
}

/////////////////////////////////////////////////
void VisualizeContacts::OnVisualizeForces(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->showForcesState = _checked;
}

/////////////////////////////////////////////////
void VisualizeContacts::UpdateForceScale(double _scale)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->forceScale = _scale;
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
    if (this->dataPtr->checkboxState && !this->dataPtr->checkboxPrevState)
    {
      // Re-scan collisions to ensure any newly added models are enabled
      this->dataPtr->CreateCollisionData(_ecm);
    }
    else if (this->dataPtr->checkboxPrevState && !this->dataPtr->checkboxState)
    {
      gzdbg << "Removing markers..." << std::endl;

      // Remove position markers
      this->dataPtr->positionMarkerMsg.set_action(
        gz::msgs::Marker::DELETE_ALL);
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->positionMarkerMsg);

      // Change action in case checkbox is checked again
      this->dataPtr->positionMarkerMsg.set_action(
        gz::msgs::Marker::ADD_MODIFY);

      // Remove force arrow body markers
      this->dataPtr->arrowBodyMarkerMsg.set_action(
        gz::msgs::Marker::DELETE_ALL);
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->arrowBodyMarkerMsg);
      this->dataPtr->arrowBodyMarkerMsg.set_action(
        gz::msgs::Marker::ADD_MODIFY);

      // Remove force arrow head markers
      this->dataPtr->arrowHeadMarkerMsg.set_action(
        gz::msgs::Marker::DELETE_ALL);
      this->dataPtr->node.Request(
        "/marker", this->dataPtr->arrowHeadMarkerMsg);
      this->dataPtr->arrowHeadMarkerMsg.set_action(
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
  gz::msgs::Marker_V markerMsgs;

  // Marker ID for position spheres and force arrows.
  // Note that the position spheres, arrow bodies and arrow heads 
  // are in separate marker namespaces.
  int posMarkerID = 1;
  int forceMarkerID = 1;

  _ecm.Each<components::ContactSensorData>(
    [&](const Entity &,
        const components::ContactSensorData *_contacts) -> bool
    {
      for (const auto &contact : _contacts->Data().contact())
      {
        for (int i = 0; i < contact.position_size(); ++i)
        {
          const auto &pos = contact.position(i);
          gz::math::Vector3d p(pos.x(), pos.y(), pos.z());

          // Add contact point position sphere marker
          auto markerMsg = markerMsgs.add_marker();
          markerMsg->CopyFrom(this->dataPtr->positionMarkerMsg);

          markerMsg->set_id(posMarkerID++);
          gz::msgs::Set(markerMsg->mutable_pose(),
            gz::math::Pose3d(p, gz::math::Quaterniond::Identity));

          // If 3D force arrows are enabled and wrench data is available
          if (this->dataPtr->showForcesState && i < contact.wrench_size())
          {
            const auto &forceMsg = contact.wrench(i).body_1_wrench().force();
            gz::math::Vector3d force(forceMsg.x(), forceMsg.y(), forceMsg.z());
            if (force.Length() > kMinimumForceToVisualize)
            {
              AddForceArrowMarkers(
                  markerMsgs,
                  forceMarkerID++,
                  p,
                  force,
                  this->dataPtr->forceScale,
                  this->dataPtr->arrowRadius,
                  this->dataPtr->arrowBodyMarkerMsg,
                  this->dataPtr->arrowHeadMarkerMsg);
            }
          }
        }
      }
      return true;
    });

  if (markerMsgs.marker_size() > 0)
  {
    this->dataPtr->node.Request(
        "/marker_array", markerMsgs, &OnMarkerArrayResponse);
  }
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
void VisualizeContacts::UpdateSphereRadius(double _radius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->sphereRadius = _radius;

  // Set scale
  gz::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
    gz::math::Vector3d(this->dataPtr->sphereRadius,
    this->dataPtr->sphereRadius,
    this->dataPtr->sphereRadius));
}

//////////////////////////////////////////////////
void VisualizeContacts::UpdateArrowRadius(double _radius)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->arrowRadius = _radius;
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
    arrowBodyMarkerMsg.mutable_lifetime()->set_nsec(_period * 1000000);
  this->dataPtr->
    arrowHeadMarkerMsg.mutable_lifetime()->set_nsec(_period * 1000000);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeContacts,
                    gz::gui::Plugin)

