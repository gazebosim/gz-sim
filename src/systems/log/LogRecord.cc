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

#include "LogRecord.hh"

#include <sys/stat.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <string>
#include <fstream>
#include <ctime>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Recorder.hh>

#include <sdf/World.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"

using namespace ignition;
using namespace ignition::gazebo::systems;

// Private data class.
class ignition::gazebo::systems::LogRecordPrivate
{
  /// \brief Start recording
  /// \param[in] _logPath Path to record to.
  /// \return True if any recorder has been started successfully.
  public: bool Start(const std::string &_logPath = std::string(""));

  /// \brief Indicator of whether any recorder instance has ever been started.
  /// Currently, only one instance is allowed. This enforcement may be removed
  /// in the future.
  public: static bool started;

  /// \brief Indicator of whether this instance has been started
  public: bool instStarted{false};

  /// \brief Ignition transport recorder
  public: transport::log::Recorder recorder;

  /// \brief Directory in which to place log file
  public: std::string logPath{""};

  /// \brief Clock used to timestamp recorded messages with sim time.
  /// This is not the timestamp on the header, rather a logging-specific stamp.
  /// This stamp is used by LogPlayback to step through logs.
  /// In case there's disagreement between these stamps, the one in the
  /// header should be the most accurate.
  public: std::unique_ptr<transport::NetworkClock> clock;

  /// \brief Name of this world
  public: std::string worldName{""};

  /// \brief SDF of this plugin
  public: std::shared_ptr<const sdf::Element> sdf{nullptr};

  /// \brief Transport node for publishing SDF string to be recorded
  public: transport::Node node;

  /// \brief Publisher for SDF string
  public: transport::Node::Publisher sdfPub;

  /// \brief Publisher for state changes
  public: transport::Node::Publisher statePub;

  /// \brief Message holding SDF string of world
  public: msgs::StringMsg sdfMsg;

  /// \brief Whether the SDF has already been published
  public: bool sdfPublished{false};
};

bool LogRecordPrivate::started{false};

//////////////////////////////////////////////////
LogRecord::LogRecord()
  : System(), dataPtr(std::make_unique<LogRecordPrivate>())
{
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
  if (this->dataPtr->instStarted)
  {
    // Use ign-transport directly
    this->dataPtr->recorder.Stop();

    ignmsg << "Stopping recording" << std::endl;
  }
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf;

  this->dataPtr->worldName = _ecm.Component<components::Name>(_entity)->Data();

  // If plugin is specified in both the SDF tag and on command line, only
  //   activate one recorder.
  if (!LogRecordPrivate::started)
  {
    auto logPath = _sdf->Get<std::string>("path");
    // Path is initialized by server if record is set from command line options.
    //   Otherwise, path is loaded from SDF. If a path is not specified in
    //   SDF, initialize to default here.
    if (logPath.empty())
    {
      logPath = ignLogDirectory();
    }

    this->dataPtr->Start(logPath);
  }
  else
  {
    ignwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogRecordPrivate::Start(const std::string &_logPath)
{
  // Only start one recorder instance
  if (LogRecordPrivate::started)
  {
    ignwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
    return true;
  }

  this->logPath = _logPath;

  // The ServerConfig takes care of specifying a default log record path.
  // This if-statement is reached if the record plugin is only specified in
  //   SDF, and a <path> is not specified.
  if (this->logPath.empty() ||
      (common::exists(this->logPath) && !common::isDirectory(this->logPath)))
  {
    ignerr << "Unspecified or invalid log path[" << this->logPath << "]. "
      << "Recording will not take place." << std::endl;
    return false;
  }

  LogRecordPrivate::started = true;

  // Create log directory
  if (!common::exists(this->logPath))
  {
    common::createDirectories(this->logPath);
  }

  // Go up to root of SDF, to record entire SDF file
  sdf::ElementPtr sdfRoot = this->sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }

  // Construct message with SDF string
  this->sdfMsg.set_data(sdfRoot->ToString(""));

  // Use directory basename as topic name, to be able to retrieve at playback
  std::string sdfTopic = "/" + common::basename(this->logPath) + "/sdf";
  this->sdfPub = this->node.Advertise(sdfTopic, this->sdfMsg.GetTypeName());

  // TODO(louise) Combine with SceneBroadcaster's state topic
  std::string stateTopic = "/world/" + this->worldName + "/changed_state";
  this->statePub = this->node.Advertise<msgs::SerializedStateMap>(stateTopic);

  // Append file name
  std::string dbPath = common::joinPaths(this->logPath, "state.tlog");
  if (common::exists(dbPath))
  {
    ignmsg << "Overwriting existing file [" << dbPath << "]\n";
    common::removeFile(dbPath);
  }
  ignmsg << "Recording to log file [" << dbPath << "]" << std::endl;

  // Use ign-transport directly
  sdf::ElementPtr sdfWorld = sdfRoot->GetElement("world");
  this->recorder.AddTopic("/world/" + this->worldName + "/dynamic_pose/info");
  this->recorder.AddTopic(sdfTopic);
  this->recorder.AddTopic(stateTopic);
  // this->recorder.AddTopic(std::regex(".*"));

  // Timestamp messages with sim time and republish that time on
  // a ~/log/clock topic (which we don't really need).
  // Note that the message headers should also have a timestamp
  auto clockTopic = "/world/" + this->worldName + "/log/clock";
  this->clock = std::make_unique<transport::NetworkClock>(clockTopic,
      transport::NetworkClock::TimeBase::SIM);
  this->recorder.Sync(this->clock.get());

  // This calls Log::Open() and loads sql schema
  if (this->recorder.Start(dbPath) ==
      ignition::transport::log::RecorderError::SUCCESS)
  {
    this->instStarted = true;
    return true;
  }
  else
    return false;
}

//////////////////////////////////////////////////
void LogRecord::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &)
{
  IGN_PROFILE("LogRecord::PreUpdate");
  // Safe guard to prevent seg faults if recorder could not be started
  if (!this->dataPtr->instStarted)
    return;
  this->dataPtr->clock->SetTime(_info.simTime);
}

//////////////////////////////////////////////////
void LogRecord::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogRecord::PostUpdate");

  // Safe guard to prevent seg faults if recorder could not be started
  if (!this->dataPtr->instStarted)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Publish only once
  if (!this->dataPtr->sdfPublished)
  {
    this->dataPtr->sdfPub.Publish(this->dataPtr->sdfMsg);
    this->dataPtr->sdfPublished = true;
  }

  // TODO(louise) Use the SceneBroadcaster's topic once that publishes
  // the changed state
  // \todo(anyone) A potential enhancement here is have a keyframe mechanism
  // to store complete state periodically, and then store incremental from
  // that. It would reduce some of the compute on replaying
  // (especially in tools like plotting or seeking through logs).
  msgs::SerializedStateMap stateMsg;
  _ecm.ChangedState(stateMsg);
  if (!stateMsg.entities().empty())
    this->dataPtr->statePub.Publish(stateMsg);
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemPreUpdate,
                    LogRecord::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogRecord,
                          "ignition::gazebo::systems::LogRecord")
