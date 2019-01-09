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

#include <ignition/plugin/RegisterMore.hh>

#include "ignition/gazebo/systems/LogRecord.hh"

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
void LogRecord::Configure(const EntityId &/*_id*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void LogRecord::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &/*_ecm*/)
{
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemPostUpdate)
