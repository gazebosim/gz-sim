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

#include <ignition/common/Filesystem.hh>
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

  /// \brief Default directory to record to
  public: static std::string DefaultRecordPath();

  // TODO(mabelmzhang) port to ign-common Filesystem
  /// \brief Generates a path for a file which doesn't collide with existing
  /// files, by appending numbers to it (i.e. (0), (1), ...)
  /// \param[in] _pathAndName Full absolute path and file name up to the
  /// file extension.
  /// \param[in] _extension File extension, such as "ddf".
  /// \return Full path with name and extension, which doesn't collide with
  /// existing files
  public: std::string UniqueFilePath(const std::string &_pathAndName,
    const std::string &_extension);

  /// \brief Unique directory path to not overwrite existing directory
  /// \param[in] _pathAndName Full absolute path
  public: std::string UniqueDirectoryPath(const std::string &_dir);

  /// \brief Indicator of whether any recorder instance has ever been started.
  /// Currently, only one instance is allowed. This enforcement may be removed
  /// in the future.
  public: static bool started;

  /// \brief Indicator of whether this instance has been started
  public: bool instStarted{false};

  /// \brief Ignition transport recorder
  public: transport::log::Recorder recorder;

  /// \brief SDF of this plugin
  public: std::shared_ptr<const sdf::Element> sdf{nullptr};

  /// \brief Transport node for publishing SDF string to be recorded
  public: transport::Node node;

  /// \brief Publisher for SDF string
  public: transport::Node::Publisher pub;

  /// \brief Message holding SDF string of world
  public: msgs::StringMsg sdfMsg;

  /// \brief Whether the SDF has already been published
  public: bool sdfPublished{false};
};

bool LogRecordPrivate::started{false};

//////////////////////////////////////////////////
std::string LogRecordPrivate::DefaultRecordPath()
{
  std::string home;
  common::env(IGN_HOMEDIR, home);

  std::time_t timestamp = std::time(nullptr);

  std::string path = common::joinPaths(home,
    ".ignition", "gazebo", "log", std::to_string(timestamp));

  return path;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::UniqueFilePath(const std::string &_pathAndName,
  const std::string &_extension)
{
  std::string result = _pathAndName + "." + _extension;
  int count = 1;

  // Check if file exists and change name accordingly
  while (common::exists(result.c_str()))
  {
    result = _pathAndName + "(" + std::to_string(count++) + ").";
    result += _extension;
  }

  return result;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::UniqueDirectoryPath(const std::string &_dir)
{
  std::string result = _dir;
  int count = 1;

  // Check if file exists and change name accordingly
  while (common::exists(result.c_str()))
  {
    result = _dir + "(" + std::to_string(count++) + ")";
  }

  return result;
}

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
void LogRecord::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf;

  // Get directory paths from SDF params
  auto logPath = _sdf->Get<std::string>("path");

  // If plugin is specified in both the SDF tag and on command line, only
  //   activate one recorder.
  if (!LogRecordPrivate::started)
  {
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
  LogRecordPrivate::started = true;

  std::string logPath = _logPath;

  // If unspecified, or specified is not a directory, use default directory
  if (logPath.empty() ||
      (common::exists(logPath) && !common::isDirectory(logPath)))
  {
    logPath = this->DefaultRecordPath();
    ignmsg << "Unspecified or invalid log path to record to. "
      << "Recording to default location [" << logPath << "]" << std::endl;
  }

  // If directoriy already exists, do not overwrite
  if (common::exists(logPath))
  {
    logPath = this->UniqueDirectoryPath(logPath);
    ignwarn << "Log path already exists on disk! "
      << "Recording instead to [" << logPath << "]" << std::endl;
  }

  // Create log directory
  if (!common::exists(logPath))
  {
    common::createDirectories(logPath);
  }

  // Go up to root of SDF, to record entire SDF file
  sdf::ElementPtr sdfRoot = this->sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }

  // Construct message with SDF string
  this->sdfMsg.set_data(sdfRoot->ToString(""));

  // Use directory basename as topic name, to be able to retrieve the SDF
  //   at playback
  std::string sdfTopic = "/" + common::basename(logPath) + "/sdf";
  this->pub = this->node.Advertise(sdfTopic,
    this->sdfMsg.GetTypeName());

  // Append file name
  std::string dbPath = common::joinPaths(logPath, "state.tlog");
  ignmsg << "Recording to log file [" << dbPath << "]" << std::endl;

  // Use ign-transport directly
  sdf::ElementPtr sdfWorld = sdfRoot->GetElement("world");
  this->recorder.AddTopic("/world/" +
    sdfWorld->GetAttribute("name")->GetAsString() + "/pose/info");
  this->recorder.AddTopic(sdfTopic);
  // this->recorder.AddTopic(std::regex(".*"));

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
void LogRecord::Update(const UpdateInfo &_info,
  EntityComponentManager &/*_ecm*/)
{
  if (_info.paused)
    return;

  // Publish only once
  if (!this->dataPtr->sdfPublished)
  {
    this->dataPtr->pub.Publish(this->dataPtr->sdfMsg);
    this->dataPtr->sdfPublished = true;
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogRecord,
                          "ignition::gazebo::systems::LogRecord")
