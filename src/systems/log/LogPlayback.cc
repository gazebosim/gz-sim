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

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Time.hh>
#include <ignition/fuel_tools/Zip.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/RegisterMore.hh>
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
  /// \brief Extract model resource files and state file from compression.
  /// \return True if extraction was successful.
  public: bool ExtractStateAndResources();

  /// \brief Start log playback.
  /// \param[in] _logPath Path of recorded state to playback.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  /// \return True if any playback has been started successfully.
  public: bool Start(EntityComponentManager &_ecm);

  /// \brief Updates the ECM according to the given message.
  /// \param[in] _ecm Mutable ECM.
  /// \param[in] _msg Message containing pose updates.
  public: void Parse(EntityComponentManager &_ecm, const msgs::Pose_V &_msg);

  /// \brief Updates the ECM according to the given message.
  /// \param[in] _ecm Mutable ECM.
  /// \param[in] _msg Message containing state updates.
  public: void Parse(EntityComponentManager &_ecm,
      const msgs::SerializedState &_msg);

  /// \brief Updates the ECM according to the given message.
  /// \param[in] _ecm Mutable ECM.
  /// \param[in] _msg Message containing state updates.
  public: void Parse(EntityComponentManager &_ecm,
      const msgs::SerializedStateMap &_msg);

  /// \brief A batch of data from log file, of all pose messages
  public: transport::log::Batch batch;

  /// \brief Pointer to ign-transport Log
  public: std::unique_ptr<transport::log::Log> log;

  /// \brief Indicator of whether any playback instance has ever been started
  public: static bool started;

  /// \brief Directory in which to place log file
  public: std::string logPath{""};

  /// \brief Directory to which compressed file is extracted to
  public: std::string extDest{""};

  /// \brief Indicator of whether this instance has been started
  public: bool instStarted{false};

  /// \brief Flag to print finish message once
  public: bool printedEnd{false};

  /// \brief Pointer to the event manager
  public: EventManager *eventManager{nullptr};
};

bool LogPlaybackPrivate::started{false};

//////////////////////////////////////////////////
LogPlayback::LogPlayback()
  : System(), dataPtr(std::make_unique<LogPlaybackPrivate>())
{
}

//////////////////////////////////////////////////
LogPlayback::~LogPlayback()
{
  if (!this->dataPtr->extDest.empty())
  {
    common::removeAll(this->dataPtr->extDest);
  }
}

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

    // The ComponentState::OneTimeChange argument forces the
    // SceneBroadcaster system to publish a message. This parameter used to
    // be ComponentState::PeriodicChange. The problem with PeriodicChange is
    // that state updates could be missed by the SceneBroadscaster, which
    // publishes at 60Hz using the wall clock. If a 60Hz tick
    // doesn't fall on the same update cycle as this state change then the
    // GUI will not receive the state information. The result is jumpy
    // playback.
    //
    // \todo(anyone) I don't think using OneTimeChange is necessarily bad, but
    // it would be nice if other systems could know that log playback is
    // active/enabled. Then a system could make decisions on how to process
    // information.
    _ecm.SetChanged(_entity, components::Pose::typeId,
        ComponentState::OneTimeChange);

    return true;
  });
}

//////////////////////////////////////////////////
void LogPlaybackPrivate::Parse(EntityComponentManager &_ecm,
    const msgs::SerializedStateMap &_msg)
{
  _ecm.SetState(_msg);
}

//////////////////////////////////////////////////
void LogPlaybackPrivate::Parse(EntityComponentManager &_ecm,
    const msgs::SerializedState &_msg)
{
  _ecm.SetState(_msg);
}

//////////////////////////////////////////////////
void LogPlayback::Configure(const Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  // Get directory paths from SDF
  this->dataPtr->logPath = _sdf->Get<std::string>("path");

  this->dataPtr->eventManager = &_eventMgr;

  // Prepend working directory if path is relative
  this->dataPtr->logPath = common::absPath(this->dataPtr->logPath);

  // If path is a file, assume it is a compressed file
  // (Otherwise assume it is a directory containing recorded files.)
  if (common::isFile(this->dataPtr->logPath))
  {
    if (!this->dataPtr->ExtractStateAndResources())
    {
      ignerr << "Cannot play back files.\n";
      return;
    }
  }

  // Enforce only one playback instance
  if (!LogPlaybackPrivate::started)
  {
    this->dataPtr->Start(_ecm);
  }
  else
  {
    ignwarn << "A LogPlayback instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogPlaybackPrivate::Start(EntityComponentManager &_ecm)
{
  if (LogPlaybackPrivate::started)
  {
    ignwarn << "A LogPlayback instance has already been started. "
      << "Will not start another.\n";
    return true;
  }

  if (this->logPath.empty())
  {
    ignerr << "Unspecified log path to playback. Nothing to play.\n";
    return false;
  }

  // Append file name
  std::string dbPath = common::joinPaths(this->logPath, "state.tlog");
  ignmsg << "Loading log file [" + dbPath + "]\n";
  if (!common::exists(dbPath))
  {
    ignerr << "Log path invalid. File [" << dbPath << "] "
      << "does not exist. Nothing to play.\n";
    return false;
  }

  // Call Log.hh directly to load a .tlog file
  this->log = std::make_unique<transport::log::Log>();
  if (!this->log->Open(dbPath))
  {
    ignerr << "Failed to open log file [" << dbPath << "]" << std::endl;
  }

  // Access all messages in .tlog file
  this->batch = this->log->QueryMessages();
  auto iter = this->batch.begin();

  if (iter == this->batch.end())
  {
    ignerr << "No messages found in log file [" << dbPath << "]" << std::endl;
  }

  // Look for the first SerializedState message and use it to set the initial
  // state of the world. Messages received before this are ignored.
  for (; iter != this->batch.end(); ++iter)
  {
    auto msgType = iter->Type();
    if (msgType == "ignition.msgs.SerializedState")
    {
      msgs::SerializedState msg;
      msg.ParseFromString(iter->Data());
      this->Parse(_ecm, msg);
      break;
    }
    else if (msgType == "ignition.msgs.SerializedStateMap")
    {
      msgs::SerializedStateMap msg;
      msg.ParseFromString(iter->Data());
      this->Parse(_ecm, msg);
      break;
    }
  }

  this->instStarted = true;
  LogPlaybackPrivate::started = true;
  return true;
}

//////////////////////////////////////////////////
bool LogPlaybackPrivate::ExtractStateAndResources()
{
  // Create a temporary directory to extract compressed file content into
  this->extDest = std::string(this->logPath);
  size_t extIdx = this->logPath.find_last_of('.');
  if (extIdx != std::string::npos)
    this->extDest = this->logPath.substr(0, extIdx);
  this->extDest += "_extracted";
  this->extDest = common::uniqueDirectoryPath(this->extDest);

  if (fuel_tools::Zip::Extract(this->logPath, this->extDest))
  {
    ignmsg << "Extracted recording to [" << this->extDest << "]" << std::endl;

    // Replace value in variable with the directory of extracted files
    // Assume directory has same name as compressed file, without extension
    // Remove extension
    this->logPath = common::joinPaths(this->extDest,
      common::basename(this->logPath.substr(0, extIdx)));
    return true;
  }
  else
  {
    ignerr << "Failed to extract recording to [" << this->extDest << "]"
      << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogPlayback::Update");
  if (_info.dt == std::chrono::steady_clock::duration::zero())
    return;

  if (!this->dataPtr->instStarted)
    return;

  // Get all messages from this timestep
  // TODO(anyone) Jumping forward can be expensive for long jumps. For now,
  // just playing every single step so we don't miss insertions and deletions.
  auto startTime = _info.simTime - _info.dt;
  auto endTime = _info.simTime;

  bool seekRewind = false;
  std::set<Entity> entitiesToRemove;
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    // Detected jumping back in time. This can be expensive.
    // To rewind / seek backward in time, we also need to play every single
    // step from the beginning so we don't miss insertions and deletions
    // This is because each serialized state is a changed state and not an
    // absolute state.
    // todo(anyone) Record absolute states during recording, i.e. key frames,
    // so that playback can jump to these states without the need to
    // incrementally build the states from the beginning

    // Create a list of entities to be removed. The list will be updated later
    // as the log steps forward below
    seekRewind = true;
    const auto &entities = _ecm.Entities().Vertices();
    for (const auto &entity : entities)
      entitiesToRemove.insert(Entity(entity.first));

    startTime = std::chrono::steady_clock::duration::zero();
  }

  this->dataPtr->batch = this->dataPtr->log->QueryMessages(
      transport::log::AllTopics({startTime, endTime}));

  msgs::Pose_V queuedPose;

  auto iter = this->dataPtr->batch.begin();
  while (iter != this->dataPtr->batch.end())
  {
    auto msgType = iter->Type();

    // Only set the last pose of a sequence of poses.
    if (msgType != "ignition.msgs.Pose_V" && queuedPose.pose_size() > 0)
    {
      this->dataPtr->Parse(_ecm, queuedPose);
      queuedPose.Clear();
    }

    if (msgType == "ignition.msgs.Pose_V")
    {
      // Queue poses to be set later
      queuedPose.ParseFromString(iter->Data());
    }
    else if (msgType == "ignition.msgs.SerializedState")
    {
      msgs::SerializedState msg;
      msg.ParseFromString(iter->Data());

      // For seeking back in time only:
      // While stepping, update the list of entities to be removed
      // so we do not remove any entities that are to be created
      if (seekRewind)
      {
        for (const auto &entIt : msg.entities())
        {
          Entity entity{entIt.id()};
          if (entIt.remove())
          {
            entitiesToRemove.insert(entity);
          }
          else
          {
            entitiesToRemove.erase(entity);
          }
        }
      }

      this->dataPtr->Parse(_ecm, msg);
    }
    else if (msgType == "ignition.msgs.SerializedStateMap")
    {
      msgs::SerializedStateMap msg;
      msg.ParseFromString(iter->Data());

      // For seeking back in time only:
      // While stepping, update the list of entities to be removed
      // so we do not remove any entities that are to be created
      if (seekRewind)
      {
        for (const auto &entIt : msg.entities())
        {
          const auto &entityMsg = entIt.second;
          Entity entity{entityMsg.id()};
          if (entityMsg.remove())
          {
            entitiesToRemove.insert(entity);
          }
          else
          {
            entitiesToRemove.erase(entity);
          }
        }
      }

      this->dataPtr->Parse(_ecm, msg);
    }
    else if (msgType == "ignition.msgs.StringMsg")
    {
      // Do nothing, we assume this is the SDF string
    }
    else
    {
      ignwarn << "Trying to playback unsupported message type ["
              << msgType << "]" << std::endl;
    }
    ++iter;
  }

  if (queuedPose.pose_size() > 0)
  {
    this->dataPtr->Parse(_ecm, queuedPose);
  }

  // for seek back in time only
  // remove entities that should not be present in the current time step
  for (auto entity : entitiesToRemove)
  {
    _ecm.RequestRemoveEntity(entity);
  }

  // pause playback if end of log is reached
  if (_info.simTime >= this->dataPtr->log->EndTime())
  {
    ignmsg << "End of log file reached. Time: " <<
      std::chrono::duration_cast<std::chrono::seconds>(
      this->dataPtr->log->EndTime()).count() << " seconds" << std::endl;

    this->dataPtr->eventManager->Emit<events::Pause>(true);
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogPlayback,
                          "ignition::gazebo::systems::LogPlayback")
