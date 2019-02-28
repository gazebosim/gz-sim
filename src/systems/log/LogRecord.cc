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

#include <string>
#include <fstream>
#include <filesystem>
#include <ctime>

#include <ignition/common/Filesystem.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Recorder.hh>

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
  /// \brief Ignition transport recorder
  public: ignition::transport::log::Recorder recorder;

  /// \brief Generates a path for a file which doesn't collide with existing
  /// files, by appending numbers to it (i.e. (0), (1), ...)
  /// \param[in] _pathAndName Full absolute path and file name up to the
  /// file extension.
  /// \param[in] _extension File extension, such as "ddf".
  /// \return Full path with name and extension, which doesn't collide with
  /// existing files
  public: std::string UniqueFilePath(const std::string &_pathAndName,
    const std::string &_extension);
};

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
  // Get params from SDF
  auto logPath = _sdf->Get<std::string>("log_path");
  auto sdfPath = _sdf->Get<std::string>("sdf_path");

  if (logPath.empty() || sdfPath.empty())
  {
    std::filesystem::path fsLogPath = std::getenv("HOME");
    fsLogPath /= ".ignition/gazebo/log";

    // Create log directory
    if (!std::filesystem::exists(fsLogPath))
    {
      std::filesystem::create_directories(fsLogPath);
    }

    std::time_t timestamp = std::time(nullptr);
    ignerr << std::to_string(timestamp) << std::endl;
    fsLogPath /= std::to_string(timestamp);

    logPath = fsLogPath.string() + ".tlog";
    sdfPath = fsLogPath.string() + ".sdf";

    ignwarn << "Unspecified log path to record to. "
      << "Recording to default location " << logPath << " and "
      << sdfPath << std::endl;
  }

  // Check if files already exist, don't overwrite
  if (std::filesystem::exists(logPath))
  {
    std::filesystem::path fsLogPath = logPath;
    logPath = this->dataPtr->UniqueFilePath(
      fsLogPath.stem().string(), fsLogPath.extension().string().substr(1));

    ignwarn << "log_path already exist on disk! "
      << "Recording instead to " << logPath << std::endl;
  }

  if (std::filesystem::exists(sdfPath))
  {
    std::filesystem::path fsSdfPath = sdfPath;
    sdfPath = this->dataPtr->UniqueFilePath(
      fsSdfPath.stem().string(), fsSdfPath.extension().string().substr(1));

    ignwarn << "sdf_path already exist on disk! "
      << "Recording instead to " << sdfPath << std::endl;
  }

  ignmsg << "Recording to log file " << logPath << std::endl;

  // Use ign-transport directly
  this->dataPtr->recorder.AddTopic("/world/default/pose/info");
  // this->dataPtr->recorder.AddTopic(std::regex(".*"));

  // This calls Log::Open() and loads sql schema
  this->dataPtr->recorder.Start(logPath);

  // Record SDF as a string.

  // TODO(mabelmzhang): For now, just dumping a big string to a text file,
  // until we have a message for the SDF.
  std::ofstream ofs(sdfPath);

  // Go up to root of SDF, to output entire SDF file
  sdf::ElementPtr sdfRoot = _sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }
  ofs << sdfRoot->ToString("");

  ignmsg << "Save initial SDF file to [" << sdfPath << "]" << std::endl;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure)
