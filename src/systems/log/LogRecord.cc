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
#include <filesystem>
#include <ctime>

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
  // If use ign-transport Log, must end in .tlog
  /// \brief Name of log file to record
  public: std::string logPath;
  // Temporary for recording sdf string, before have custom SQL field for
  //   a big SDF string.
  public: std::string sdfPath;

  // Use ign-transport directly
  /// \brief Log file or nullptr if not recording
  public: ignition::transport::log::Recorder recorder;

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
  this->dataPtr->logPath = _sdf->Get<std::string>("log_path",
    this->dataPtr->logPath).first;
  this->dataPtr->sdfPath = _sdf->Get<std::string>("sdf_path",
    this->dataPtr->sdfPath).first;

  if (this->dataPtr->logPath.empty() || this->dataPtr->sdfPath.empty())
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

    this->dataPtr->logPath = fsLogPath.string() + ".tlog";
    this->dataPtr->sdfPath = fsLogPath.string() + ".sdf";

    ignwarn << "Unspecified log path to record to. "
      << "Recording to default location " << this->dataPtr->logPath << " and "
      << this->dataPtr->sdfPath << std::endl;
  }

  // Check if files already exist, don't overwrite
  if (std::filesystem::exists(this->dataPtr->logPath))
  {
    std::filesystem::path fsLogPath = this->dataPtr->logPath;
    this->dataPtr->logPath = this->dataPtr->UniqueFilePath(
      fsLogPath.stem().string(), fsLogPath.extension().string().substr(1));

    ignwarn << "log_path already exist on disk! "
      << "Recording instead to " << this->dataPtr->logPath << std::endl;
  }

  if (std::filesystem::exists(this->dataPtr->sdfPath))
  {
    std::filesystem::path fsSdfPath = this->dataPtr->sdfPath;
    this->dataPtr->sdfPath = this->dataPtr->UniqueFilePath(
      fsSdfPath.stem().string(), fsSdfPath.extension().string().substr(1));

    ignwarn << "sdf_path already exist on disk! "
      << "Recording instead to " << this->dataPtr->sdfPath << std::endl;
  }

  ignmsg << "Recording to log file " << this->dataPtr->logPath << std::endl;


  // Use ign-transport directly

  this->dataPtr->recorder.AddTopic("/world/default/pose/info");
  // this->dataPtr->recorder.AddTopic(std::regex(".*"));

  // This calls Log::Open() and loads sql schema
  this->dataPtr->recorder.Start(this->dataPtr->logPath);


  // Record SDF as a string.

  // TODO(mabelmzhang): For now, just dumping a big string to a text file,
  //   until we have a message for the SDF.
  std::ofstream ofs(this->dataPtr->sdfPath);
  // Go up to root of SDF, to output entire SDF file
  sdf::ElementPtr sdfRoot = _sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }
  ofs << sdfRoot->ToString("");
  ignmsg << "Outputted SDF to " << this->dataPtr->sdfPath << std::endl;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure)
