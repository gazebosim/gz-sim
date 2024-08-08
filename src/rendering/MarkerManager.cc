/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <functional>
#include <list>
#include <map>
#include <mutex>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Events.hh"
#include "gz/sim/Conversions.hh"
#include "gz/common/Console.hh"
#include "gz/rendering/Marker.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

#include "gz/sim/rendering/MarkerManager.hh"

using namespace gz;
using namespace sim;

/// Private data for the MarkerManager class
class gz::sim::MarkerManagerPrivate
{
  /// \brief Processes a marker message.
  /// \param[in] _msg The message data.
  /// \return True if the marker was processed successfully.
  public: bool ProcessMarkerMsg(const msgs::Marker &_msg);

  /// \brief Converts a Gazebo msg render type to Gazebo Sim Rendering
  /// \param[in] _msg The message data
  /// \return Converted rendering type, if any.
  public: rendering::MarkerType MsgToType(
                    const msgs::Marker &_msg);

  /// \brief Converts a Gazebo msg material to Gazebo Sim Rendering
  //         material.
  //  \param[in] _msg The message data.
  //  \return Converted rendering material, if any.
  public: rendering::MaterialPtr MsgToMaterial(
                    const msgs::Marker &_msg);

  /// \brief Updates the markers.
  public: void Update();

  /// \brief Callback that receives marker messages.
  /// \param[in] _req The marker message.
  public: void OnMarkerMsg(const msgs::Marker &_req);

  /// \brief Callback that receives multiple marker messages.
  /// \param[in] _req The vector of marker messages
  /// \param[in] _res Response data
  /// \return True if the request is received
  public: bool OnMarkerMsgArray(const msgs::Marker_V &_req,
              msgs::Boolean &_res);

  /// \brief Services callback that returns a list of markers.
  /// \param[out] _rep Service reply
  /// \return True on success.
  public: bool OnList(msgs::Marker_V &_rep);

  /// \brief Sets Marker from marker message.
  /// \param[in] _msg The message data.
  /// \param[out] _markerPtr The message pointer to set.
  public: void SetMarker(const msgs::Marker &_msg,
                         const rendering::MarkerPtr &_markerPtr);

  /// \brief Sets Visual from marker message.
  /// \param[in] _msg The message data.
  /// \param[out] _visualPtr The visual pointer to set.
  public: void SetVisual(const msgs::Marker &_msg,
                         const rendering::VisualPtr &_visualPtr);

  /// \brief Sets sim time from time.
  /// \param[in] _time The time data.
  public: void SetSimTime(const std::chrono::steady_clock::duration &_time);

  /// \brief Previous sim time received
  public: std::chrono::steady_clock::duration lastSimTime;

  /// \brief Mutex to protect message list.
  public: std::mutex mutex;

  /// \brief Map of visuals
  public: std::map<std::string,
      std::map<uint64_t, rendering::VisualPtr>> visuals;

  /// \brief List of marker message to process.
  public: std::list<msgs::Marker> markerMsgs;

  /// \brief Pointer to the scene
  public: rendering::ScenePtr scene;

  /// \brief Gazebo node
  public: transport::Node node;

  /// \brief Sim time according to UpdateInfo in RenderUtil
  public: std::chrono::steady_clock::duration simTime;

  /// \brief The last marker message received
  public: msgs::Marker msg;

  /// \brief Topic name for the marker service
  public: std::string topicName = "/marker";

  /// \brief Topic that publishes marker updates.
  public: gz::transport::Node::Publisher markerPub;
};

/////////////////////////////////////////////////
MarkerManager::MarkerManager() : dataPtr(
    std::make_unique<MarkerManagerPrivate>())
{
}

/////////////////////////////////////////////////
MarkerManager::~MarkerManager() = default;

/////////////////////////////////////////////////
void MarkerManager::SetScene(rendering::ScenePtr _scene)
{
  this->dataPtr->scene = std::move(_scene);
}

/////////////////////////////////////////////////
rendering::ScenePtr MarkerManager::Scene() const
{
  return this->dataPtr->scene;
}

/////////////////////////////////////////////////
void MarkerManager::Update()
{
  return this->dataPtr->Update();
}

/////////////////////////////////////////////////
bool MarkerManager::Init(const rendering::ScenePtr &_scene)
{
  if (!_scene)
  {
    gzerr << "Scene pointer is invalid\n";
    return false;
  }

  this->dataPtr->scene = _scene;

  if (this->dataPtr->topicName.empty())
  {
    gzerr << "Unable to advertise marker service. Topic name empty.\n";
    return false;
  }

  // Advertise the list service
  if (!this->dataPtr->node.Advertise(this->dataPtr->topicName + "/list",
      &MarkerManagerPrivate::OnList, this->dataPtr.get()))
  {
    gzerr << "Unable to advertise to the " << this->dataPtr->topicName
           << "/list service.\n";
  }

  // Advertise to the marker service
  if (!this->dataPtr->node.Advertise(this->dataPtr->topicName,
        &MarkerManagerPrivate::OnMarkerMsg, this->dataPtr.get()))
  {
    gzerr << "Unable to advertise to the " << this->dataPtr->topicName
           << " service.\n";
  }

  // Advertise to the marker_array service
  if (!this->dataPtr->node.Advertise(this->dataPtr->topicName + "_array",
        &MarkerManagerPrivate::OnMarkerMsgArray, this->dataPtr.get()))
  {
    gzerr << "Unable to advertise to the " << this->dataPtr->topicName
           << "_array service.\n";
  }

  this->dataPtr->markerPub =
    this->dataPtr->node.Advertise<msgs::Marker>(this->dataPtr->topicName);

  return true;
}

/////////////////////////////////////////////////
void MarkerManager::SetTopic(const std::string &_name)
{
  this->dataPtr->topicName = _name;
}

/////////////////////////////////////////////////
void MarkerManager::SetSimTime(
    const std::chrono::steady_clock::duration &_time)
{
  this->dataPtr->SetSimTime(_time);
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::Update()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Process the marker messages.
  for (auto markerIter = this->markerMsgs.begin();
       markerIter != this->markerMsgs.end();)
  {
    this->ProcessMarkerMsg(*markerIter);
    this->markerPub.Publish(*markerIter);
    this->markerMsgs.erase(markerIter++);
  }


  // Erase any markers that have a lifetime.
  for (auto mit = this->visuals.begin();
       mit != this->visuals.end();)
  {
    for (auto it = mit->second.cbegin();
         it != mit->second.cend(); ++it)
    {
      if (it->second->GeometryCount() == 0u)
        continue;

      rendering::MarkerPtr markerPtr =
            std::dynamic_pointer_cast<rendering::Marker>
            (it->second->GeometryByIndex(0u));
      if (markerPtr != nullptr)
      {
        if (markerPtr->Lifetime().count() != 0 &&
            (markerPtr->Lifetime() <= simTime ||
            this->simTime < this->lastSimTime))
        {
          this->scene->DestroyVisual(it->second);
          it = mit->second.erase(it);
          break;
        }
      }
    }

    // Erase a namespace if it's empty
    if (mit->second.empty())
      mit = this->visuals.erase(mit);
    else
      ++mit;
  }
  this->lastSimTime = this->simTime;
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::SetSimTime(
    const std::chrono::steady_clock::duration &_time)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->simTime = _time;
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::SetVisual(const msgs::Marker &_msg,
                           const rendering::VisualPtr &_visualPtr)
{
  // Set Visual Scale
  if (_msg.has_scale())
  {
    _visualPtr->SetLocalScale(_msg.scale().x(),
                              _msg.scale().y(),
                              _msg.scale().z());
  }

  // Set Visual Pose
  if (_msg.has_pose())
    _visualPtr->SetLocalPose(convert<math::Pose3d>(_msg.pose()));

  // Set Visual Parent
  if (!_msg.parent().empty())
  {
    if (_visualPtr->HasParent())
    {
      _visualPtr->Parent()->RemoveChild(_visualPtr);
    }

    rendering::VisualPtr parent = this->scene->VisualByName(_msg.parent());

    if (parent)
    {
      parent->AddChild(_visualPtr);
    }
    else
    {
      gzerr << "No visual with the name[" << _msg.parent() << "]\n";
    }
  }

  // todo(anyone) Update Marker Visibility
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::SetMarker(const msgs::Marker &_msg,
                           const rendering::MarkerPtr &_markerPtr)
{
  _markerPtr->SetLayer(_msg.layer());

  // Set Marker Lifetime
  std::chrono::steady_clock::duration lifetime =
    convert<std::chrono::steady_clock::duration>(_msg.lifetime());
  if (lifetime.count() != 0)
  {
    _markerPtr->SetLifetime(lifetime + this->simTime);
  }
  else
  {
    _markerPtr->SetLifetime(std::chrono::seconds(0));
  }
  // Set Marker Render Type
  rendering::MarkerType markerType = MsgToType(_msg);
  _markerPtr->SetType(markerType);

  // Set Marker Material
  if (_msg.has_material())
  {
    rendering::MaterialPtr materialPtr = MsgToMaterial(_msg);
    _markerPtr->SetMaterial(materialPtr, true /* clone */);

    // clean up material after clone
    this->scene->DestroyMaterial(materialPtr);
  }

  // Assume the presence of points means we clear old ones
  if (_msg.point().size() > 0)
  {
    _markerPtr->ClearPoints();
  }

  math::Color color(
      _msg.material().diffuse().r(),
      _msg.material().diffuse().g(),
      _msg.material().diffuse().b(),
      _msg.material().diffuse().a());

  // Set Marker Points
  for (int i = 0; i < _msg.point().size(); ++i)
  {
    math::Vector3d vector(
        _msg.point(i).x(),
        _msg.point(i).y(),
        _msg.point(i).z());

    _markerPtr->AddPoint(vector, color);
  }
}

/////////////////////////////////////////////////
rendering::MarkerType MarkerManagerPrivate::MsgToType(
                          const msgs::Marker &_msg)
{
  msgs::Marker_Type marker = this->msg.type();
  if (marker != _msg.type() && _msg.type() != msgs::Marker::NONE)
  {
    marker = _msg.type();
    this->msg.set_type(_msg.type());
  }
  switch (marker)
  {
    case msgs::Marker::BOX:
      return rendering::MarkerType::MT_BOX;
    case msgs::Marker::CAPSULE:
      return rendering::MarkerType::MT_CAPSULE;
    case msgs::Marker::CONE:
      return rendering::MarkerType::MT_CONE;
    case msgs::Marker::CYLINDER:
      return rendering::MarkerType::MT_CYLINDER;
    case msgs::Marker::LINE_STRIP:
      return rendering::MarkerType::MT_LINE_STRIP;
    case msgs::Marker::LINE_LIST:
      return rendering::MarkerType::MT_LINE_LIST;
    case msgs::Marker::POINTS:
      return rendering::MarkerType::MT_POINTS;
    case msgs::Marker::SPHERE:
      return rendering::MarkerType::MT_SPHERE;
    case msgs::Marker::TEXT:
      return rendering::MarkerType::MT_TEXT;
    case msgs::Marker::TRIANGLE_FAN:
      return rendering::MarkerType::MT_TRIANGLE_FAN;
    case msgs::Marker::TRIANGLE_LIST:
      return rendering::MarkerType::MT_TRIANGLE_LIST;
    case msgs::Marker::TRIANGLE_STRIP:
      return rendering::MarkerType::MT_TRIANGLE_STRIP;
    default:
      gzerr << "Unable to create marker of type[" << _msg.type() << "]\n";
      break;
  }
  return rendering::MarkerType::MT_NONE;
}

/////////////////////////////////////////////////
rendering::MaterialPtr MarkerManagerPrivate::MsgToMaterial(
                              const msgs::Marker &_msg)
{
  rendering::MaterialPtr material = this->scene->CreateMaterial();

  material->SetAmbient(
      _msg.material().ambient().r(),
      _msg.material().ambient().g(),
      _msg.material().ambient().b(),
      _msg.material().ambient().a());

  material->SetDiffuse(
      _msg.material().diffuse().r(),
      _msg.material().diffuse().g(),
      _msg.material().diffuse().b(),
      _msg.material().diffuse().a());

  material->SetSpecular(
      _msg.material().specular().r(),
      _msg.material().specular().g(),
      _msg.material().specular().b(),
      _msg.material().specular().a());

  material->SetEmissive(
      _msg.material().emissive().r(),
      _msg.material().emissive().g(),
      _msg.material().emissive().b(),
      _msg.material().emissive().a());

  material->SetLightingEnabled(_msg.material().lighting());

  return material;
}

//////////////////////////////////////////////////
bool MarkerManagerPrivate::ProcessMarkerMsg(const msgs::Marker &_msg)
{
  // Get the namespace, if it exists. Otherwise, use the global namespace
  std::string ns;
  if (!_msg.ns().empty()) {
    ns = _msg.ns();
  }

  // Get the namespace that the marker belongs to
  auto nsIter = this->visuals.find(ns);

  // If an id is given
  size_t id;
  if (_msg.id() != 0)
  {
    id = _msg.id();
  }
  // Otherwise generate unique id
  else
  {
    id = math::Rand::IntUniform(0, math::MAX_I32);

    // Make sure it's unique if namespace is given
    if (nsIter != this->visuals.end())
    {
      while (nsIter->second.find(id) != nsIter->second.end())
        id = math::Rand::IntUniform(math::MIN_UI32,
                                              math::MAX_UI32);
    }
  }

  // Get visual for this namespace and id
  std::map<uint64_t, rendering::VisualPtr>::iterator visualIter;
  if (nsIter != this->visuals.end())
    visualIter = nsIter->second.find(id);

  // Add/modify a marker
  if (_msg.action() == msgs::Marker::ADD_MODIFY)
  {
    // Modify an existing marker, identified by namespace and id
    if (nsIter != this->visuals.end() &&
        visualIter != nsIter->second.end())
    {
      if (visualIter->second->GeometryCount() > 0u)
      {
        // TODO(anyone): Update so that multiple markers can
        //               be attached to one visual
        rendering::MarkerPtr markerPtr =
              std::dynamic_pointer_cast<rendering::Marker>
              (visualIter->second->GeometryByIndex(0));

        visualIter->second->RemoveGeometryByIndex(0);

        // Set the visual values from the Marker Message
        this->SetVisual(_msg, visualIter->second);

        // Set the marker values from the Marker Message
        this->SetMarker(_msg, markerPtr);

        visualIter->second->AddGeometry(markerPtr);
      }
    }
    // Otherwise create a new marker
    else
    {
      // Create the name for the marker
      std::string name = "__GZ_MARKER_VISUAL_" + ns + "_" +
                         std::to_string(id);

      // Create the new marker
      rendering::VisualPtr visualPtr = this->scene->CreateVisual(name);

      // Create and load the marker
      rendering::MarkerPtr markerPtr = this->scene->CreateMarker();

      // Set the visual values from the Marker Message
      this->SetVisual(_msg, visualPtr);

      // Set the marker values from the Marker Message
      this->SetMarker(_msg, markerPtr);

      // Add populated marker to the visual
      visualPtr->AddGeometry(markerPtr);

      // Add visual to root visual
      if (!visualPtr->HasParent())
      {
        this->scene->RootVisual()->AddChild(visualPtr);
      }

      // Store the visual
      this->visuals[ns][id] = visualPtr;
    }
  }
  // Remove a single marker
  else if (_msg.action() == msgs::Marker::DELETE_MARKER)
  {
    // Remove the marker if it can be found.
    if (nsIter != this->visuals.end() &&
        visualIter != nsIter->second.end())
    {
      this->scene->DestroyVisual(visualIter->second);
      this->visuals[ns].erase(visualIter);

      // Remove namespace if empty
      if (this->visuals[ns].empty())
        this->visuals.erase(nsIter);
    }
    else
    {
      gzwarn << "Unable to delete marker with id[" << id << "] "
        << "in namespace[" << ns << "]" << std::endl;
      return false;
    }
  }
  // Remove all markers, or all markers in a namespace
  else if (_msg.action() == msgs::Marker::DELETE_ALL)
  {
    // If given namespace doesn't exist
    if (!ns.empty() && nsIter == this->visuals.end())
    {
      gzwarn << "Unable to delete all markers in namespace[" << ns <<
          "], namespace can't be found." << std::endl;
      return false;
    }
    // Remove all markers in the specified namespace
    else if (nsIter != this->visuals.end())
    {
      for (auto it : nsIter->second)
      {
        this->scene->DestroyVisual(it.second);
      }
      nsIter->second.clear();
      this->visuals.erase(nsIter);
    }
    // Remove all markers in all namespaces.
    else
    {
      for (nsIter = this->visuals.begin();
           nsIter != this->visuals.end(); ++nsIter)
      {
        for (auto it : nsIter->second)
        {
          this->scene->DestroyVisual(it.second);
        }
      }
      this->visuals.clear();
    }
  }
  else
  {
    gzerr << "Unknown marker action[" << _msg.action() << "]\n";
    return false;
  }

  return true;
}


/////////////////////////////////////////////////
bool MarkerManagerPrivate::OnList(msgs::Marker_V &_rep)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  _rep.clear_marker();

  // Create the list of visuals
  for (auto mIter : this->visuals)
  {
    for (auto iter : mIter.second)
    {
      msgs::Marker *markerMsg = _rep.add_marker();
      markerMsg->set_ns(mIter.first);
      markerMsg->set_id(iter.first);
    }
  }

  return true;
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::OnMarkerMsg(const msgs::Marker &_req)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->markerMsgs.push_back(_req);
}

/////////////////////////////////////////////////
bool MarkerManagerPrivate::OnMarkerMsgArray(
    const msgs::Marker_V&_req, msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::copy(_req.marker().begin(), _req.marker().end(),
            std::back_inserter(this->markerMsgs));
  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void MarkerManager::Clear()
{
  this->dataPtr->visuals.clear();
  this->dataPtr->markerMsgs.clear();
  this->dataPtr->scene.reset();
}
