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

#include "LogPlayback.hh"

#include <ignition/msgs/pose_v.pb.h>

#include <string>

#include <ignition/msgs/Utility.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/plugin/RegisterMore.hh>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Time.hh>
#include <ignition/transport/log/QueryOptions.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Message.hh>

#include <sdf/Root.hh>

#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/Pose.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private LogPlayback data class.
class ignition::gazebo::systems::LogPlaybackPrivate
{
  /// \brief Start log playback.
  /// \param[in] _logPath Path of recorded state to playback.
  /// \param[in] _worldEntity The world entity this plugin is attached to.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  /// \param[in] _eventMgr The EventManager of the given simulation
  /// instance.
  /// \return True if any playback has been started successfully.
  public: bool Start(const std::string &_logPath,
      const Entity &_worldEntity, EntityComponentManager &_ecm,
      EventManager &_eventMgr);

  /// \brief Updates the ECM according to the given message.
  /// \param[in] _ecm Mutable ECM.
  /// \param[in] _msg Message containing pose updates.
  public: void Parse(EntityComponentManager &_ecm, const msgs::Pose_V &_msg);

  /// \brief Updates the ECM according to the given message.
  /// \param[in] _ecm Mutable ECM.
  /// \param[in] _msg Message containing state updates.
  public: void Parse(EntityComponentManager &_ecm,
      const msgs::SerializedState &_msg);

  /// \brief A batch of data from log file, of all pose messages
  public: transport::log::Batch batch;

  /// \brief Iterator to go through messages in Batch
  public: transport::log::MsgIter iter;

  /// \brief Indicator of whether any playback instance has ever been started
  public: static bool started;

  /// \brief Indicator of whether this instance has been started
  public: bool instStarted{false};

  /// \brief Flag to print finish message once
  public: bool printedEnd{false};
};

bool LogPlaybackPrivate::started{false};

//////////////////////////////////////////////////
LogPlayback::LogPlayback()
  : System(), dataPtr(std::make_unique<LogPlaybackPrivate>())
{
}

//////////////////////////////////////////////////
LogPlayback::~LogPlayback() = default;

//////////////////////////////////////////////////
void LogPlaybackPrivate::Parse(EntityComponentManager &_ecm,
    const msgs::Pose_V &_msg)
{
  // Maps entity to pose recorded
  // Key: entity. Value: pose
  std::map <Entity, msgs::Pose> idToPose;

  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    msgs::Pose pose = _msg.pose(i);

    // Update entity pose in map
    idToPose.insert_or_assign(pose.id(), pose);
  }

  // Loop through entities in world
  _ecm.Each<components::Pose>(
      [&](const Entity &_entity, components::Pose *_poseComp) -> bool
  {
    // Check if we have an updated pose for this entity
    if (idToPose.find(_entity) == idToPose.end())
      return true;

    // Look for pose in log entry loaded
    msgs::Pose pose = idToPose.at(_entity);

    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(msgs::Convert(pose));

    return true;
  });
}

//////////////////////////////////////////////////
void LogPlaybackPrivate::Parse(EntityComponentManager &_ecm,
    const msgs::SerializedState &_msg)
{
  _ecm.SetState(_msg);
}

//////////////////////////////////////////////////
void LogPlayback::Configure(const Entity &_worldEntity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  // Get directory paths from SDF
  auto logPath = _sdf->Get<std::string>("path");

  // Enforce only one playback instance
  if (!LogPlaybackPrivate::started)
  {
    this->dataPtr->Start(logPath, _worldEntity, _ecm, _eventMgr);
  }
  else
  {
    ignwarn << "A LogPlayback instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogPlaybackPrivate::Start(const std::string &_logPath,
  const Entity &_worldEntity, EntityComponentManager &_ecm,
  EventManager &_eventMgr)
{
  if (LogPlaybackPrivate::started)
  {
    ignwarn << "A LogPlayback instance has already been started. "
      << "Will not start another.\n";
    return true;
  }

  if (_logPath.empty())
  {
    ignerr << "Unspecified log path to playback. Nothing to play.\n";
    return false;
  }

  if (!common::isDirectory(_logPath))
  {
    ignerr << "Specified log path [" << _logPath << "] must be a directory.\n";
    return false;
  }

  // Append file name
  std::string dbPath = common::joinPaths(_logPath, "state.tlog");
  ignmsg << "Loading log file [" + dbPath + "]\n";
  if (!common::exists(dbPath))
  {
    ignerr << "Log path invalid. File [" << dbPath << "] "
      << "does not exist. Nothing to play.\n";
    return false;
  }

  // Call Log.hh directly to load a .tlog file
  auto log = std::make_unique<transport::log::Log>();
  if (!log->Open(dbPath))
  {
    ignerr << "Failed to open log file [" << dbPath << "]" << std::endl;
  }

  // Find SDF string in .tlog file
  transport::log::TopicList sdfOpts("/" + common::basename(_logPath) + "/sdf");
  transport::log::Batch sdfBatch = log->QueryMessages(sdfOpts);
  transport::log::MsgIter sdfIter = sdfBatch.begin();
  if (sdfIter == sdfBatch.end())
  {
    // location of log may have changed from where it was recorded.
    // search through the topics available in the log and find the sdf topic
    sdfBatch = log->QueryMessages(
        transport::log::TopicPattern(std::regex(".*/sdf")));
    sdfIter = sdfBatch.begin();
    if (sdfIter == sdfBatch.end())
    {
      ignerr << "No SDF found in log file [" << dbPath << "]" << std::endl;
      return false;
    }
  }

  // Parse SDF message
  msgs::StringMsg sdfMsg;
  sdfMsg.ParseFromString(sdfIter->Data());

  // Load recorded SDF file
  sdf::Root root;
  if (root.LoadSdfString(sdfMsg.data()).size() != 0 || root.WorldCount() <= 0)
  {
    ignerr << "Error loading SDF string logged in file [" << dbPath << "]"
      << std::endl;
    return false;
  }
  const sdf::World *sdfWorld = root.WorldByIndex(0);

  std::vector <sdf::ElementPtr> pluginsRm;

  // Look for LogRecord plugin in the SDF and remove it, so that playback
  //   is not re-recorded.
  // Remove Physics plugin, so that it does not clash with recorded poses.
  // TODO(anyone) Cherry-picking plugins to remove doesn't scale well,
  // handle this better once we're logging the initial world state in the DB
  // file.
  if (sdfWorld->Element()->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");

    // If never found, nothing to remove
    while (pluginElt != nullptr)
    {
      if (pluginElt->HasAttribute("name"))
      {
        if ((pluginElt->GetAttribute("name")->GetAsString().find("LogRecord")
          != std::string::npos) ||
          (pluginElt->GetAttribute("name")->GetAsString().find("Physics")
          != std::string::npos))
        {
          // Flag for removal.
          // Do not actually remove plugin from parent while looping through
          //   children of this parent. Else cannot access next element.
          pluginsRm.push_back(pluginElt);
        }
      }

      // Go to next plugin
      pluginElt = pluginElt->GetNextElement("plugin");
    }
  }

  // Remove the marked plugins
  for (auto &it : pluginsRm)
  {
    it->RemoveFromParent();
    igndbg << "Removed " << it->GetAttribute("name")->GetAsString()
      << " plugin from loaded SDF\n";
  }

  // Create all Entities in SDF <world> tag
  ignition::gazebo::SdfEntityCreator creator =
    ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);

  // Models
  for (uint64_t modelIndex = 0; modelIndex < sdfWorld->ModelCount();
      ++modelIndex)
  {
    auto model = sdfWorld->ModelByIndex(modelIndex);
    auto modelEntity = creator.CreateEntities(model);

    creator.SetParent(modelEntity, _worldEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < sdfWorld->LightCount();
      ++lightIndex)
  {
    auto light = sdfWorld->LightByIndex(lightIndex);
    auto lightEntity = creator.CreateEntities(light);

    creator.SetParent(lightEntity, _worldEntity);
  }

  // Access all messages in .tlog file
  this->batch = log->QueryMessages();
  this->iter = this->batch.begin();

  if (this->iter == this->batch.end())
  {
    ignerr << "No messages found in log file [" << dbPath << "]" << std::endl;
  }

  this->instStarted = true;
  LogPlaybackPrivate::started = true;
  return true;
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->dataPtr->instStarted)
    return;

  // TODO(anyone) Support rewind
  // Sanity check. If playing reached the end, done.
  if (this->dataPtr->iter == this->dataPtr->batch.end())
  {
    // Print only once
    if (!this->dataPtr->printedEnd)
    {
      ignmsg << "Finished playing all recorded data\n";
      this->dataPtr->printedEnd = true;
    }
    return;
  }

  auto msgType = this->dataPtr->iter->Type();

  // Only playback if current sim time has exceeded next logged timestamp
  // TODO(anyone) Support multiple msgs per update, in case playback has a lower
  // frequency than record
  if (msgType == "ignition.msgs.Pose_V")
  {
    msgs::Pose_V msg;
    msg.ParseFromString(this->dataPtr->iter->Data());

    auto stamp = convert<std::chrono::steady_clock::duration>(
        msg.header().stamp());

    if (_info.simTime >= stamp)
    {
      this->dataPtr->Parse(_ecm, msg);
      ++(this->dataPtr->iter);
    }
  }
  else if (msgType == "ignition.msgs.SerializedState")
  {
    msgs::SerializedState msg;
    msg.ParseFromString(this->dataPtr->iter->Data());

    auto stamp = convert<std::chrono::steady_clock::duration>(
        msg.header().stamp());

    if (_info.simTime >= stamp)
    {
      this->dataPtr->Parse(_ecm, msg);
      ++(this->dataPtr->iter);
    }
  }
  else if (msgType == "ignition.msgs.StringMsg")
  {
    // Do nothing, we assume this is the SDF string
    ++(this->dataPtr->iter);
  }
  else
  {
    ignwarn << "Trying to playback unsupported message type ["
            << msgType << "]" << std::endl;
    ++(this->dataPtr->iter);
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogPlayback,
                          "ignition::gazebo::systems::LogPlayback")
