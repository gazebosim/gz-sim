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

#include "../../GuiRunner.hh"
#include "EntityContextMenu.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <iostream>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/Conversions.hh>
#include <gz/gui/Application.hh>
#include <gz/transport/Node.hh>

namespace gz::sim
{
  /// \brief Private data class for EntityContextMenu
  class EntityContextMenuPrivate
  {

    /// \brief Protects variable changed through services.
    public: std::mutex mutex;

    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief Move to service name
    public: std::string moveToService;

    /// \brief Track topic name
    public: std::string trackTopic;

    /// \brief Currently tracked topic name
    public: std::string currentTrackTopic;

    /// \brief Remove service name
    public: std::string removeService;

    /// \brief View as transparent service name
    public: std::string viewTransparentService;

    /// \brief View center of mass service name
    public: std::string viewCOMService;

    /// \brief View inertia service name
    public: std::string viewInertiaService;

    /// \brief View joints service name
    public: std::string viewJointsService;

    /// \brief View wireframes service name
    public: std::string viewWireframesService;

    /// \brief View collisions service name
    public: std::string viewCollisionsService;

    /// \brief View frames service name
    public: std::string viewFramesService;

    /// \brief Copy service name
    public: std::string copyService;

    /// \brief Paste service name
    public: std::string pasteService;

    /// \brief Name of world.
    public: std::string worldName;

    /// \brief Storing last follow target for look at.
    public: std::string followTargetLookAt;

    /// \brief Flag used to disable look at when not following target.
    public: bool followingTarget{false};

    /// \brief /gui/track publisher
    public: transport::Node::Publisher trackPub;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
void EntityContextMenu::OnCurrentlyTrackedSub(const msgs::CameraTrack &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->followingTarget = false;
  if (_msg.track_mode() == gz::msgs::CameraTrack::FOLLOW ||
      _msg.track_mode() == gz::msgs::CameraTrack::FOLLOW_LOOK_AT ||
      _msg.track_mode() == gz::msgs::CameraTrack::FOLLOW_FREE_LOOK)
  {
    this->dataPtr->followingTarget = true;
  }
    this->FollowingTargetChanged();

  return;
}

/////////////////////////////////////////////////
void GzSimPlugin::registerTypes(const char *_uri)
{
  // Register our 'EntityContextMenuItem' in qml engine
  qmlRegisterType<EntityContextMenu>(_uri, 1, 0,
      "EntityContextMenuItem");
}

/////////////////////////////////////////////////
EntityContextMenu::EntityContextMenu()
  : dataPtr(std::make_unique<EntityContextMenuPrivate>())
{
  this->dataPtr->currentTrackTopic = "/gui/currently_tracked";
  this->dataPtr->node.Subscribe(this->dataPtr->currentTrackTopic,
      &EntityContextMenu::OnCurrentlyTrackedSub, this);
  gzmsg << "Currently tracking topic on ["
         << this->dataPtr->currentTrackTopic << "]" << std::endl;

  // For move to service requests
  this->dataPtr->moveToService = "/gui/move_to";

  // For track topic message
  this->dataPtr->trackTopic = "/gui/track";

  // For remove service requests
  this->dataPtr->removeService = "/world/default/remove";

  // For view transparent service requests
  this->dataPtr->viewTransparentService = "/gui/view/transparent";

  // For view center of mass service requests
  this->dataPtr->viewCOMService = "/gui/view/com";

  // For view inertia service requests
  this->dataPtr->viewInertiaService = "/gui/view/inertia";

  // For view joints service requests
  this->dataPtr->viewJointsService = "/gui/view/joints";

  // For view wireframes service requests
  this->dataPtr->viewWireframesService = "/gui/view/wireframes";

  // For view collisions service requests
  this->dataPtr->viewCollisionsService = "/gui/view/collisions";

  // For view frames service requests
  this->dataPtr->viewFramesService = "/gui/view/frames";

  // For copy service requests
  this->dataPtr->copyService = "/gui/copy";

  // For paste service requests
  this->dataPtr->pasteService = "/gui/paste";

  this->dataPtr->trackPub =
    this->dataPtr->node.Advertise<msgs::CameraTrack>(this->dataPtr->trackTopic);
}

/////////////////////////////////////////////////
EntityContextMenu::~EntityContextMenu() = default;

/////////////////////////////////////////////////
void EntityContextMenu::SetFollowingTarget(bool &_followingTarget)
{
  this->dataPtr->followingTarget = _followingTarget;
  this->FollowingTargetChanged();
}

/////////////////////////////////////////////////
bool EntityContextMenu::FollowingTarget() const
{
  return this->dataPtr->followingTarget;
}

/////////////////////////////////////////////////
void EntityContextMenu::OnRemove(
  const QString &_data, const QString &_type)
{
  if (this->dataPtr->worldName.empty())
  {
    auto runners = gui::App()->findChildren<GuiRunner *>();
    if (runners.empty() || runners[0] == nullptr)
    {
      gzerr << "Internal error: no GuiRunner found." << std::endl;
      return;
    }

    this->dataPtr->worldName = "default";
    auto worldNameVariant = runners[0]->property("worldName");
    if (!worldNameVariant.isValid())
    {
      gzwarn << "GuiRunner's worldName not set, using["
              << this->dataPtr->worldName << "]" << std::endl;
    }
    else
    {
      this->dataPtr->worldName = worldNameVariant.toString().toStdString();
    }

    this->dataPtr->removeService =
      "/world/" + this->dataPtr->worldName + "/remove";
  }

  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &_rep, const bool _result)
  {
    if (!_result || !_rep.data())
      gzerr << "Error sending remove request" << std::endl;
  };

  msgs::Entity req;
  req.set_name(_data.toStdString());
  req.set_type(convert<msgs::Entity_Type>(_type.toStdString()));

  this->dataPtr->node.Request(this->dataPtr->removeService, req, cb);
}

/////////////////////////////////////////////////
void EntityContextMenu::OnRequest(const QString &_request, const QString &_data)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error sending move to request" << std::endl;
  };

  std::string request = _request.toStdString();
  if (request == "move_to")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->moveToService, req, cb);
  }
  else if (request == "follow")
  {
    msgs::CameraTrack followMsg;
    followMsg.mutable_follow_target()->set_name(_data.toStdString());
    followMsg.set_track_mode(msgs::CameraTrack::FOLLOW);
    this->dataPtr->followTargetLookAt = followMsg.follow_target().name();
    gzmsg << "Follow target: " << followMsg.follow_target().name() << std::endl;
    this->dataPtr->trackPub.Publish(followMsg);
  }
  else if (request == "free_look")
  {
    msgs::CameraTrack followMsg;
    followMsg.mutable_follow_target()->set_name(_data.toStdString());
    followMsg.set_track_mode(msgs::CameraTrack::FOLLOW_FREE_LOOK);
    this->dataPtr->followTargetLookAt = followMsg.follow_target().name();
    gzmsg << "Follow target: " << followMsg.follow_target().name() << std::endl;
    this->dataPtr->trackPub.Publish(followMsg);
  }
  else if (request == "look_at")
  {
    msgs::CameraTrack followMsg;
    followMsg.mutable_track_target()->set_name(_data.toStdString());
    followMsg.set_track_mode(msgs::CameraTrack::FOLLOW_LOOK_AT);
    followMsg.mutable_follow_target()->set_name(
        this->dataPtr->followTargetLookAt);
    gzmsg << "Follow target: " << followMsg.follow_target().name() << std::endl;
    gzmsg << "Look at target: " << followMsg.track_target().name() << std::endl;
    this->dataPtr->trackPub.Publish(followMsg);
  }
  else if (request == "track")
  {
    msgs::CameraTrack trackMsg;
    trackMsg.mutable_track_target()->set_name(_data.toStdString());
    trackMsg.set_track_mode(msgs::CameraTrack::TRACK);
    gzmsg << "Track target: " << trackMsg.track_target().name() << std::endl;
    this->dataPtr->trackPub.Publish(trackMsg);
  }
  else if (request == "view_transparent")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewTransparentService, req, cb);
  }
  else if (request == "view_com")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewCOMService, req, cb);
  }
  else if (request == "view_inertia")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewInertiaService, req, cb);
  }
  else if (request == "view_joints")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewJointsService, req, cb);
  }
  else if (request == "view_wireframes")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewWireframesService, req, cb);
  }
  else if (request == "view_collisions")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewCollisionsService, req, cb);
  }
  else if (request == "view_frames")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewFramesService, req, cb);
  }
  else if (request == "copy")
  {
    msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->copyService, req, cb);
  }
  else if (request == "paste")
  {
    msgs::Empty req;
    this->dataPtr->node.Request(this->dataPtr->pasteService, req, cb);
  }
  else
  {
    gzwarn << "Unknown request [" << request << "]" << std::endl;
  }
}
