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
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  // Get params from SDF
  this->logPath = _sdf->Get<std::string>("log_path",
      this->logPath).first;
  ignerr << "Playing back log file " << this->logPath << std::endl;


  // Use ign-transport directly
  // TODO: This only plays the messages on ign topic, but doesn't create or
  //   change any objects in the world! Still need to pull out all the
  //   objects from the .tlog file through SQL, and talk to ECM to create those
  //   objects in the world!
  //   So maybe don't need to use playback at all. Just call Log.hh to load
  //   the .tlog file, and then we do stuff with objects in it.
  this->player.reset (new ignition::transport::log::Playback (this->logPath));

  const int64_t addTopicResult = this->player->AddTopic (std::regex (".*"));
  if (addTopicResult == 0)
  {
    ignerr << "No topics to play back\n";
  }
  else if (addTopicResult < 0)
  {
    ignerr << "Failed to advertise topics: " << addTopicResult << std::endl;
    this->player.reset ();
  }
  else
  {
    const auto handle = player->Start ();
    if (!handle)
    {
      ignerr << "Failed to start playback\n";
      this->player.reset ();
    }
    else
    {
      ignerr << "Starting playback\n";
    }
  }

  /* Call Log.hh directly to load a .tlog file

  this->log.Open (this->logPath);



  */


  // Use ECM

  // Load log file, find all models in it
  /*
  for ()
  {
    Entity eid = _ecm.CreateEntity ();
    if (eid == kNullEntity)
    {
      ignerr << "Error in creating entity" << std::endl;
    }
    else
    {
      ignerr << "Created an entity" << std::endl;
    }

    ComponentType data = ?
    ComponentKey ck = _ecm.CreateComponent (eid, data);
  }
  ignerr << _ecm.EntityCount () << " entities" << std::endl;
  */

}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_manager)
{
  //std::ifstream ifs(this->logPath);
  //ifs >> _manager;


  // Use ECM

  // TODO: Look into how to actually move the joints etc

  // Subscribe to a topic, then call ECM to override states - no idea what that means.


}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
