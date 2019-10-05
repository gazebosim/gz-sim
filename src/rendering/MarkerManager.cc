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

  /// \brief Receive messages from the world_stats topic
  /// \param[in] _msg The world stats message
  /// public: void OnStatsMsg(ConstWorldStatisticsPtr &_msg);

  /// \brief Previous sim time received
  public: common::Time lastSimTime;

  /// \brief Mutex to protect message list.
  public: std::mutex mutex;

  /// \brief Map of visuals
  public: Visual_M visuals;

  /// \brief List of marker message to process.
  public: MarkerMsgs_L markerMsgs;

  /// \brief Pointer to the scene
  public: ignition::rendering::Scene *scene = nullptr;

  /// \brief Ignition node
  public: ignition::transport::Node node;

  /// \brief Gazebo transport node
  /// public: rendering::NodePtr ign_node;

  /// \brief Connect to the prerender signal
  // public: event::ConnectionPtr preRenderConnection;

  /// \brief Sim time according to world_stats
  public: common::Time simTime;

  /// \brief Subscribe to world_stats topic
  /// public: transport::SubscriberPtr statsSub;
};

/////////////////////////////////////////////////
MarkerManager::MarkerManager()
: dataPtr(new MarkerManagerPrivate)
{
}

/////////////////////////////////////////////////
MarkerManager::~MarkerManager()
{
  // this->dataPtr->statsSub.reset();
  // this->dataPtr->ign_node.reset();
}

/////////////////////////////////////////////////
bool MarkerManager::Init(ignition::rendering::Scene *_scene)
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

  // this->dataPtr->ign_node = transport::NodePtr(new transport::Node());
  // this->dataPtr->ign_node->Init();

  // this->dataPtr->statsSub = this->dataPtr->ign_node->Subscribe("~/world_stats",
  //    &MarkerManagerPrivate::OnStatsMsg, this->dataPtr.get());

  // Process markers on PreRender
  // this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
  //    std::bind(&MarkerManagerPrivate::OnPreRender, this->dataPtr.get()));

  return true;
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
         it != mit->second.cend();)
    {
      // Erase a marker if it has a lifetime and it's expired,
      // or if the world has reset
      /*
      if (it->second->Lifetime() != common::Time::Zero &&
          (it->second->Lifetime() <= this->simTime ||
          this->simTime < this->lastSimTime))
      {
        it->second->Fini();
        this->scene->RemoveVisual(it->second);
        it = mit->second.erase(it);
      }
      else
      */
      ++it;
    }

    // Erase a namespace if it's empty
    if (mit->second.empty())
      mit = this->visuals.erase(mit);
    else
      ++mit;
  }
  this->lastSimTime = this->simTime;
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

  // Get marker for this namespace and id
  std::map<uint64_t, VisualPtr>::iterator markerIter;
  if (nsIter != this->visuals.end())
    markerIter = nsIter->second.find(id);

  // Add/modify a marker
  if (_msg.action() == ignition::msgs::Marker::ADD_MODIFY)
  {
    // Modify an existing marker, identified by namespace and id
    if (nsIter != this->visuals.end() &&
        markerIter != nsIter->second.end())
    {
      // TODO(jshep1): set the properties for the marker here
      // markerIter->second->Load(_msg);
      
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
      markerPtr->SetLayer(_msg.layer());
      markerPtr->SetLifetime(convert<std::chrono::steady_clock::duration>(_msg.lifetime()));

      visualPtr->AddGeometry(markerPtr);
      this->scene->RootVisual()->AddChild(visualPtr);

      // Store the marker
      this->visuals[ns][id] = visualPtr;
    }
  }
  // Remove a single marker
  else if (_msg.action() == ignition::msgs::Marker::DELETE_MARKER)
  {
    // Remove the marker if it can be found.
    if (nsIter != this->visuals.end() &&
        markerIter != nsIter->second.end())
    {
      // TODO(jshep1): cleanup code here for Fini()
      // markerIter->second->Fini();
      this->scene->DestroyVisual(markerIter->second);
      this->visuals[ns].erase(markerIter);

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
        // TODO(jshep1): cleanup code here for Fini()
        // it->second->Fini();
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
          // TODO(jshep1): cleanup code here for Fini()
          // it->second->Fini();
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
void MarkerManagerPrivate::OnMarkerMsg(const ignition::msgs::Marker &_req)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->markerMsgs.push_back(_req);
}
