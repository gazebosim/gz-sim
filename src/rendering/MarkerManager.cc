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

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/common/Console.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/Marker.hh"

#include "ignition/gazebo/rendering/MarkerManager.hh"

using namespace ignition;
using namespace gazebo;

/// Private data for the MarkerManager class
class ignition::gazebo::MarkerManagerPrivate
{
  /// \def MarkerPtr
  /// \brief Shared pointer to Marker
  typedef std::shared_ptr<ignition::rendering::Visual> VisualPtr;

  /// \def Marker_M
  /// \brief Map of markers. The key is a marker namespace, the
  /// value is the map of markers in the namespace and their ids.
  typedef std::map<std::string, std::map<uint64_t, VisualPtr>> Visual_M;

  /// \def MarkerMsgs_L
  /// \brief List of marker messages.
  typedef std::list<ignition::msgs::Marker> MarkerMsgs_L;

  /// \brief Process a marker message.
  /// \param[in] _msg The message data.
  /// \return True if the marker was processed successfully.
  public: bool ProcessMarkerMsg(const ignition::msgs::Marker &_msg);

  /// \brief Convert an ignition msg render type to ignition rendering
  /// \param[in] _msg The message data
  /// \return Converted rendering type, if any.
  public: ignition::rendering::MarkerType MsgToType(
                    const ignition::msgs::Marker &_msg);
  
  /// \brief Convert an ignition msg material to ignition rendering
  //         material.
  //  \param[in] _msg The message data.
  //  \return Converted rendering material, if any.
  public: rendering::MaterialPtr MsgToMaterial(
                    const ignition::msgs::Marker &_msg);

  /// \brief Update the markers. This function is called on
  /// the PreRender event.
  public: void OnPreRender();

  /// \brief Callback that receives marker messages.
  /// \param[in] _req The marker message.
  public: void OnMarkerMsg(const ignition::msgs::Marker &_req);

  /// \brief Service callback that returns a list of markers.
  /// \param[out] _rep Service reply
  /// \return True on success.
  public: bool OnList(ignition::msgs::Marker_V &_rep);

  /// \brief Set Marker from marker message.
  /// \param[in] _msg The message data.
  /// \param[out] _markerPtr The message pointer to set.
  public: void SetMarker(const ignition::msgs::Marker &_msg,
                         rendering::MarkerPtr _markerPtr);

  /// \brief Set Visual from marker message.
  /// \param[in] _msg The message data.
  /// \param[out] _visualPtr The visual pointer to set.
  public: void SetVisual(const ignition::msgs::Marker &_msg,
                         rendering::VisualPtr _visualPtr);

  /// \brief Set sim time from time.
  /// \param[in] _time The time data.
  public: void SetSimTime(const std::chrono::steady_clock::duration &_time);

  /// \brief Receive messages from the world_stats topic
  /// \param[in] _msg The world stats message
  /// public: void OnStatsMsg(ConstWorldStatisticsPtr &_msg);

  /// \brief Previous sim time received
  public: std::chrono::steady_clock::duration lastSimTime;

  /// \brief Mutex to protect message list.
  public: std::mutex mutex;

  /// \brief Map of visuals
  public: Visual_M visuals;

  /// \brief List of marker message to process.
  public: MarkerMsgs_L markerMsgs;

  /// \brief Pointer to the scene
  public: rendering::ScenePtr scene;

  /// \brief Ignition node
  public: ignition::transport::Node node;

  /// \brief Sim time according to world_stats
  public: std::chrono::steady_clock::duration simTime;

  /// \brief The last marker message received
  public: ignition::msgs::Marker msg;
};

/////////////////////////////////////////////////
MarkerManager::MarkerManager()
: dataPtr(std::make_unique<MarkerManagerPrivate>())
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
void MarkerManager::PreRender() const
{
  return this->dataPtr->OnPreRender();
}

/////////////////////////////////////////////////
bool MarkerManager::Init(ignition::rendering::ScenePtr _scene)
{
  if (!_scene)
  {
    ignerr << "Scene pointer is invalid\n";
    return false;
  }

  this->dataPtr->scene = _scene;

  // Advertise the list service
  if (!this->dataPtr->node.Advertise("/marker/list",
      &MarkerManagerPrivate::OnList, this->dataPtr.get()))
  {
    ignerr << "Unable to advertise to the /marker/list service.\n";
  }

  // Advertise to the marker service
  if (!this->dataPtr->node.Advertise("/marker",
        &MarkerManagerPrivate::OnMarkerMsg, this->dataPtr.get()))
  {
    ignerr << "Unable to advertise to the /marker service.\n";
  }

  return true;
}

void MarkerManager::SetSimTime(const std::chrono::steady_clock::duration &_time){
  this->dataPtr->SetSimTime(_time);
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::OnPreRender()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Process the marker messages.
  for (auto markerIter = this->markerMsgs.begin();
       markerIter != this->markerMsgs.end();)
  {
    this->ProcessMarkerMsg(*markerIter);
    this->markerMsgs.erase(markerIter++);
  }


  // Erase any markers that have a lifetime.
  for (auto mit = this->visuals.begin();
       mit != this->visuals.end();)
  {
    for (auto it = mit->second.cbegin();
         it != mit->second.cend(); ++it)
    {
      ignition::rendering::MarkerPtr markerPtr =
            std::dynamic_pointer_cast<ignition::rendering::Marker>
            (it->second->GeometryByIndex(0));
      
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

void MarkerManagerPrivate::SetSimTime(const std::chrono::steady_clock::duration &_time)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  simTime = _time;
}

void MarkerManagerPrivate::SetVisual(const ignition::msgs::Marker &_msg,
                           rendering::VisualPtr _visualPtr)
{
  // Set Visual Scale
  _visualPtr->SetLocalScale(_msg.scale().x(),
                            _msg.scale().y(),
                            _msg.scale().z()
                            );

  // Set Visual Pose
  _visualPtr->SetLocalPose(convert<math::Pose3d>(_msg.pose()));

  // Set Visual Parent
  if (!_msg.parent().empty())
  {
    if (_visualPtr->HasParent())
    {
      _visualPtr->Parent()->RemoveChild(_visualPtr);
    }
    
    VisualPtr parent = this->scene->VisualByName(_msg.parent());
  
    if (parent)
    {
      parent->AddChild(_visualPtr);
    }
    else
    {
      ignerr << "No visual with the name[" << _msg.parent() << "]\n";
    }
  }
 
  // Update Marker Visibility
  _visualPtr->SetVisible(_msg.visibility());
}

void MarkerManagerPrivate::SetMarker(const ignition::msgs::Marker &_msg,
                           rendering::MarkerPtr _markerPtr)
{
  _markerPtr->SetLayer(_msg.layer());

  // Set Marker Lifetime
  std::chrono::steady_clock::duration lifetime =
    convert<std::chrono::steady_clock::duration>(_msg.lifetime());
  if (lifetime.count() != 0)
  {
    _markerPtr->SetLifetime(lifetime + this->simTime);
  }
  // Set Marker Render Type
  ignition::rendering::MarkerType markerType = MsgToType(_msg);
  _markerPtr->SetType(markerType);

  // Set Marker Material
  rendering::MaterialPtr materialPtr = MsgToMaterial(_msg);
  _markerPtr->SetMaterial(materialPtr, true /* clone */);

  // Assume the presence of points means we clear old ones
  if (_msg.point().size() > 0)
  {
    _markerPtr->ClearPoints();
  }

  math::Color color(
      _msg.material().ambient().r(),
      _msg.material().ambient().g(),
      _msg.material().ambient().b(),
      _msg.material().ambient().a()
      );

  // Set Marker Points
  for (int i = 0; i < _msg.point().size(); ++i)
  {
    math::Vector3d vector(
        _msg.point(i).x(),
        _msg.point(i).y(),
        _msg.point(i).z()
        );

    _markerPtr->AddPoint(vector, color);
  }
}

ignition::rendering::MarkerType MarkerManagerPrivate::MsgToType(
                          const ignition::msgs::Marker &_msg)
{
  ignition::msgs::Marker_Type marker = this->msg.type();
  if (marker != _msg.type() && _msg.type() != ignition::msgs::Marker::NONE)
  {
    marker = _msg.type();
    this->msg.set_type(_msg.type());
  }
  switch (marker)
  {
    case ignition::msgs::Marker::BOX:
      return ignition::rendering::MarkerType::BOX;
    case ignition::msgs::Marker::CYLINDER:
      return ignition::rendering::MarkerType::CYLINDER;
    case ignition::msgs::Marker::LINE_STRIP:
      return ignition::rendering::MarkerType::LINE_STRIP;
    case ignition::msgs::Marker::LINE_LIST:
      return ignition::rendering::MarkerType::LINE_LIST;
    case ignition::msgs::Marker::POINTS:
      return ignition::rendering::MarkerType::POINTS;
    case ignition::msgs::Marker::SPHERE:
      return ignition::rendering::MarkerType::SPHERE;
    case ignition::msgs::Marker::TEXT:
      return ignition::rendering::MarkerType::TEXT;
    case ignition::msgs::Marker::TRIANGLE_FAN:
      return ignition::rendering::MarkerType::TRIANGLE_FAN;
    case ignition::msgs::Marker::TRIANGLE_LIST:
      return ignition::rendering::MarkerType::TRIANGLE_LIST;
    case ignition::msgs::Marker::TRIANGLE_STRIP:
      return ignition::rendering::MarkerType::TRIANGLE_STRIP;
    default:
      ignerr << "Unable to create marker of type[" << _msg.type() << "]\n";
      break;
  }
  return ignition::rendering::MarkerType::NONE;
}

rendering::MaterialPtr MarkerManagerPrivate::MsgToMaterial(
                              const ignition::msgs::Marker &_msg)
{
  rendering::MaterialPtr material = this->scene->CreateMaterial();

  material->SetAmbient(
      _msg.material().ambient().r(),
      _msg.material().ambient().g(),
      _msg.material().ambient().b(),
      _msg.material().ambient().a()
      );
  
  material->SetDiffuse(
      _msg.material().diffuse().r(),
      _msg.material().diffuse().g(),
      _msg.material().diffuse().b(),
      _msg.material().diffuse().a()
      );
  
  material->SetSpecular(
      _msg.material().specular().r(),
      _msg.material().specular().g(),
      _msg.material().specular().b(),
      _msg.material().specular().a()
      );
  
  material->SetEmissive(
      _msg.material().emissive().r(),
      _msg.material().emissive().g(),
      _msg.material().emissive().b(),
      _msg.material().emissive().a()
      );
  
  material->SetLightingEnabled(_msg.material().lighting());

  return material;
}

//////////////////////////////////////////////////
bool MarkerManagerPrivate::ProcessMarkerMsg(const ignition::msgs::Marker &_msg)
{
  // Get the namespace, if it exists. Otherwise, use the global namespace
  std::string ns;
  ns = _msg.ns();

  // Get the namespace that the marker belongs to
  Visual_M::iterator nsIter = this->visuals.find(ns);

  // If an id is given
  size_t id;
  id = _msg.id();

  // Get visual for this namespace and id
  std::map<uint64_t, VisualPtr>::iterator visualIter;
  if (nsIter != this->visuals.end())
    visualIter = nsIter->second.find(id);

  // Add/modify a marker
  if (_msg.action() == ignition::msgs::Marker::ADD_MODIFY)
  {
    // Modify an existing marker, identified by namespace and id
    if (nsIter != this->visuals.end() &&
        visualIter != nsIter->second.end())
    {
      // TODO(anyone): Update so that multiple markers can
      //               be attached to one visual
      ignition::rendering::MarkerPtr markerPtr =
            std::dynamic_pointer_cast<ignition::rendering::Marker>
            (visualIter->second->GeometryByIndex(0));
      
      visualIter->second->RemoveGeometryByIndex(0);

      // Set the visual values from the Marker Message
      SetVisual(_msg, visualIter->second);
      
      // Set the marker values from the Marker Message
      SetMarker(_msg, markerPtr);

      visualIter->second->AddGeometry(markerPtr);
    }
    // Otherwise create a new marker
    else
    {
      // Create the name for the marker
      std::string name = "__IGN_MARKER_VISUAL_" + ns + "_" +
                         std::to_string(id);

      // Create the new marker
      rendering::VisualPtr visualPtr = this->scene->CreateVisual(name);

      // Create and load the marker
      rendering::MarkerPtr markerPtr = this->scene->CreateMarker();

      // Set the visual values from the Marker Message
      SetVisual(_msg, visualPtr);

      // Set the marker values from the Marker Message
      SetMarker(_msg, markerPtr);

      // Add populated marker to the visual
      visualPtr->AddGeometry(markerPtr);
      
      // Add visual to root visual
      this->scene->RootVisual()->AddChild(visualPtr);

      // Store the visual
      this->visuals[ns][id] = visualPtr;
    }
  }
  // Remove a single marker
  else if (_msg.action() == ignition::msgs::Marker::DELETE_MARKER)
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
      ignwarn << "Unable to delete marker with id[" << id << "] "
        << "in namespace[" << ns << "]" << std::endl;
      return false;
    }
  }
  // Remove all markers, or all markers in a namespace
  else if (_msg.action() == ignition::msgs::Marker::DELETE_ALL)
  {
    // If given namespace doesn't exist
    if (!ns.empty() && nsIter == this->visuals.end())
    {
      ignwarn << "Unable to delete all markers in namespace[" << ns <<
          "], namespace can't be found." << std::endl;
      return false;
    }
    // Remove all markers in the specified namespace
    else if (nsIter != this->visuals.end())
    {
      for (auto it = nsIter->second.begin(); it != nsIter->second.end(); ++it)
      {
        this->scene->DestroyVisual(it->second);
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
        for (auto it = nsIter->second.begin(); it != nsIter->second.end(); ++it)
        {
          this->scene->DestroyVisual(it->second);
        }
      }
      this->visuals.clear();
    }
  }
  else
  {
    ignerr << "Unknown marker action[" << _msg.action() << "]\n";
    return false;
  }

  return true;
}


/////////////////////////////////////////////////
bool MarkerManagerPrivate::OnList(ignition::msgs::Marker_V &_rep)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  _rep.clear_marker();

  // Create the list of visuals
  for (Visual_M::const_iterator mIter = this->visuals.begin();
       mIter != this->visuals.end(); ++mIter)
  {
    for (std::map<uint64_t, VisualPtr>::const_iterator iter =
        mIter->second.begin(); iter != mIter->second.end(); ++iter)
    {
      ignition::msgs::Marker *markerMsg = _rep.add_marker();
      markerMsg->set_ns(mIter->first);
      markerMsg->set_id(iter->first);
      //iter->second->FillMsg(*markerMsg);
    }
  }

  return true;
}


/////////////////////////////////////////////////
void MarkerManagerPrivate::OnMarkerMsg(const ignition::msgs::Marker &_req)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->markerMsgs.push_back(_req);
}
