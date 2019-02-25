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

#include <string>
#include <fstream>
#include <filesystem>

#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Joint.hh"

#include <ignition/transport/log/Log.hh>


using namespace ignition::gazebo::systems;

//////////////////////////////////////////////////
LogRecord::LogRecord()
  : System()
{
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
  // Use ign-transport directly
  this->recorder.Stop();

  ignmsg << "Stopping recording" << std::endl;
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
  // Get params from SDF
  this->logPath = _sdf->Get<std::string>("log_path",
      this->logPath).first;
  this->sdfPath = _sdf->Get<std::string>("sdf_path",
      this->sdfPath).first;

  // Check if files already exist, don't overwrite
  if (std::filesystem::exists(this->logPath) ||
      std::filesystem::exists(this->sdfPath))
  {
    ignerr << "log_path and/or sdf_path already exist on disk! "
      << "Not overwriting. Will not record." << std::endl;
    return;
  }

  ignmsg << "Recording to log file " << this->logPath << std::endl;


  // Use ign-transport directly

  this->recorder.AddTopic("/world/default/pose/info");
  // this->recorder.AddTopic(std::regex(".*"));

  // This calls Log::Open() and loads sql schema
  this->recorder.Start(this->logPath);


  // Record SDF as a string.

  // TODO(mabelmzhang): For now, just dumping a big string to a text file,
  //   until we have a message for the SDF.
  std::ofstream ofs(this->sdfPath);
  // Go up to root of SDF, to output entire SDF file
  sdf::ElementPtr sdf_root = _sdf->GetParent();
  while (sdf_root->GetParent() != NULL)
  {
    sdf_root = sdf_root->GetParent();
  }
  ofs << sdf_root->ToString("");
  ignmsg << "Outputted SDF to " << this->sdfPath << std::endl;
}

//////////////////////////////////////////////////
void LogRecord::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{

}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemUpdate)
