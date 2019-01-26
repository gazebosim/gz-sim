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

#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "LogRecord.hh"

using namespace ignition::gazebo::systems;

//////////////////////////////////////////////////
LogRecord::LogRecord()
  : System()
{
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void LogRecord::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_manager)
{
  {
    std::ofstream ofs("myfile.log");
    ofs << _manager;
  }
  {
    // Read the object back in.
    std::ifstream ifs("myfile.log");
    ifs >> _manager;
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemUpdate)
