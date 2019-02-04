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

#include <fstream>

#include <ignition/plugin/RegisterMore.hh>

#include "LogPlayback.hh"

using namespace ignition::gazebo::systems;

//////////////////////////////////////////////////
LogPlayback::LogPlayback()
  : System()
{
}

//////////////////////////////////////////////////
LogPlayback::~LogPlayback()
{
}

//////////////////////////////////////////////////
void LogPlayback::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
  // Get params from SDF
  this->logPath = _sdf->Get<std::string>("log_path",
      this->logPath).first;
  ignmsg << "Playing back log file " << this->logPath << std::endl;

}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_manager)
{
  std::ifstream ifs(this->logPath);
  ifs >> _manager;

  // TODO: Look into how to actually move the joints etc
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
