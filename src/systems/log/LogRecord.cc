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

#include <ignition/msgs/pose_v.pb.h>
#include <sys/stat.h>

#include <string>
#include <fstream>
#include <ctime>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/log/Log.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Joint.hh"


using namespace ignition::gazebo::systems;

// Private data class.
class ignition::gazebo::systems::LogRecordPrivate
{
  // Use ign-transport directly
  /// \brief ign-transport message recorder
  public: ignition::transport::log::Recorder recorder;

  /// \brief Default directory to record to
  public: std::string DefaultRecordPath();

  // TODO(mabelmzhang) port to ign-common Filesystem
  /// \brief Unique file path to not overwrite existing file
  public: std::string UniqueFilePath(const std::string &_pathAndName,
    const std::string &_extension);
  /// \brief Unique directory path to not overwrite existing file
  public: std::string UniqueDirectoryPath(const std::string &_dir);
};

//////////////////////////////////////////////////
std::string LogRecordPrivate::DefaultRecordPath()
{
  std::string fsLogPath = ignition::common::joinPaths(IGN_HOMEDIR,
    ".ignition/gazebo/log");

  std::time_t timestamp = std::time(nullptr);
  fsLogPath = ignition::common::joinPaths(fsLogPath,
    std::to_string(timestamp));

  return fsLogPath;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::UniqueFilePath(const std::string &_pathAndName,
  const std::string &_extension)
{
  std::string result = _pathAndName + "." + _extension;
  int count = 1;
  struct stat buf;

  // Check if file exists and change name accordingly
  while (stat(result.c_str(), &buf) != -1)
  {
    result = _pathAndName + "(" + std::to_string(count++) + ")." + _extension;
  }

  return result;
}

std::string LogRecordPrivate::UniqueDirectoryPath(const std::string &_dir)
{
  std::string result = _dir;
  int count = 1;
  struct stat buf;

  // Check if file exists and change name accordingly
  while (stat(result.c_str(), &buf) != -1)
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
  // Use ign-transport directly
  this->dataPtr->recorder.Stop();

  ignmsg << "Stopping recording" << std::endl;
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
  // Get directory paths from SDF params
  // Name of log file to record
  // If use ign-transport Log, must end in .tlog
  std::string logPath;
  logPath = _sdf->Get<std::string>("path", logPath).first;

  // If unspecified, use default directory
  if (logPath.empty())
  {
    logPath = this->dataPtr->DefaultRecordPath();
    ignwarn << "Unspecified log path to record to. "
      << "Recording to default location " << logPath << std::endl;
  }

  // If directory already exists, do not overwrite
  if (ignition::common::exists(logPath))
  {
    logPath = this->dataPtr->UniqueDirectoryPath(logPath);
    ignwarn << "Log path already exist on disk! "
      << "Recording instead to " << logPath << std::endl;
  }

  // Create log directory
  if (!ignition::common::exists(logPath))
  {
    ignition::common::createDirectories(logPath);
  }

  // Append file names
  std::string dbPath = ignition::common::joinPaths(logPath, "state.tlog");
  // Temporary for recording sdf string
  std::string sdfPath = ignition::common::joinPaths(logPath, "state.sdf");

  ignmsg << "Recording to log file " << dbPath << std::endl;


  // Use ign-transport directly

  this->dataPtr->recorder.AddTopic("/world/default/pose/info");
  // this->dataPtr->recorder.AddTopic(std::regex(".*"));

  // This calls Log::Open() and loads sql schema
  this->dataPtr->recorder.Start(dbPath);


  // Record SDF as a string.

  // TODO(mabelmzhang): For now, just dumping a big string to a text file,
  //   until we have a message for the SDF.
  std::ofstream ofs(sdfPath);
  // Go up to root of SDF, to output entire SDF file
  sdf::ElementPtr sdfRoot = _sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }
  ofs << sdfRoot->ToString("");
  ignmsg << "Outputted SDF to " << sdfPath << std::endl;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogRecord,
                          "ignition::gazebo::systems::LogRecord")
