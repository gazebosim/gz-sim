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
#include <ignition/common/Profiler.hh>
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
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  /// \return True if any playback has been started successfully.
  public: bool Start(const std::string &_logPath, EntityComponentManager &_ecm);

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

  /// \brief
  public: std::unique_ptr<transport::log::Log> log;

  /// \brief Indicator of whether any playback instance has ever been started
  public: static bool started;

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
  auto logPath = _sdf->Get<std::string>("path");

  this->dataPtr->eventManager = &_eventMgr;

  // Enforce only one playback instance
  if (!LogPlaybackPrivate::started)
  {
    this->dataPtr->Start(logPath, _ecm);
  }
  else
  {
    ignwarn << "A LogPlayback instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogPlaybackPrivate::Start(const std::string &_logPath,
                               EntityComponentManager &_ecm)
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
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    // If jumping backwards, check 1 second before
    startTime = endTime - std::chrono::seconds(1);
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
      this->dataPtr->Parse(_ecm, msg);
    }
    else if (msgType == "ignition.msgs.SerializedStateMap")
    {
      msgs::SerializedStateMap msg;
      msg.ParseFromString(iter->Data());
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
