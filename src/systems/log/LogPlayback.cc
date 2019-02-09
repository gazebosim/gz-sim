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

// To read contents in a .tlog database file
//#include <ignition/transport/log/Descriptor.hh>
#include <ignition/transport/log/QueryOptions.hh>
#include <ignition/transport/log/Message.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <sdf/Root.hh>
#include <ignition/gazebo/Factory.hh>


using namespace ignition::gazebo::systems;
using namespace ignition::transport::log;

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
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  // Get params from SDF
  this->logPath = _sdf->Get<std::string>("log_path",
    this->logPath).first;
  this->sdfPath = _sdf->Get<std::string>("sdf_path",
    this->sdfPath).first;
  ignerr << "Playing back log file " << this->logPath << std::endl;


  // Use ign-transport playback directly
  // TODO: This only plays the messages on ign topic, but doesn't create or
  //   change any objects in the world! Still need to pull out all the
  //   objects from the .tlog file through SQL, and talk to ECM to create those
  //   objects in the world!
  //   So maybe don't need to use playback at all. Just call Log.hh to load
  //   the .tlog file, and then we do stuff with objects in it.
  /*
  this->player.reset (new Playback (this->logPath));

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
  */


  // Call Log.hh directly to load a .tlog file

  this->log.reset (new Log ());
  this->log->Open (this->logPath);

  // Don't need
  //const Descriptor * desc = this->log->Descriptor ();
  /*
  Descriptor::NameToMap name_to_map = desc->TopicsToMsgTypesToId ();
  Descriptor::NameToId name_to_id = name_to_map.at ("/world/default/pose/info");
  int64_t row_i = name_to_id.at ("ignition.msgs.Pose_V");
  */
  // Above 3 lines can be replaced by this convenience function:
  //int64_t row_i = desc->TopicId ("/world/default/pose/info",
  //  "ignition.msgs.Pose_V");
  //ignerr << "row " << row_i << std::endl;

  // Access messages in .tlog file
  TopicList opts = TopicList ("/world/default/pose/info");
  this->poseBatch = this->log->QueryMessages (opts);
  this->iter = this->poseBatch.begin ();

  ignerr << this->iter->TimeReceived ().count () << std::endl;
  ignerr << this->iter->Type () << std::endl;
  // Once have the message type e.g. pose, can talk to ECM to move things.
  //   But a pose message doesn't tell me what an object looks like! Need the
  //   original SDF string - which needs to be recorded, to know object
  //   geometry. TODO add that to LogRecorder.cc.
  ignerr << this->iter->Data() << std::endl;

  // TODO: Parse iter->Type () to get substring after last ".", to know what
  //   message type to create!

  // Protobuf message
  // For now, just hardcode Pose_V
  // Convert binary bytes in string into a ign-msgs msg
  ignition::msgs::Pose_V posev_msg;
  posev_msg.ParseFromString (this->iter->Data());
  ignerr << "Pose_V size: " << posev_msg.pose_size () << std::endl;
  for (int i = 0; i < posev_msg.pose_size (); i ++)
  {
    ignition::msgs::Pose pose = posev_msg.pose (i);
    ignerr << pose.name () << std::endl;
    //ignerr << pose.id () << std::endl;
    ignerr << pose.position ().x () << ", " << pose.position ().y () << ", " << pose.position ().z () << std::endl;
    ignerr << pose.orientation ().x () << ", " << pose.orientation ().y () << ", " << pose.orientation ().z () << ", " << pose.orientation ().w () << std::endl;
  }


  // Use ECM

  // Load recorded SDF file
  sdf::Root root;
  if (root.Load (this->sdfPath).size () != 0)
  {
    ignerr << "Error loading SDF file " << this->sdfPath << std::endl;
    return;
  }
  ignerr << "World count: " << root.WorldCount () << std::endl;
  ignerr << "Model count: " << root.ModelCount () << std::endl;
  const sdf::World * sdf_world = root.WorldByIndex (0);

  ignition::gazebo::Factory factory = ignition::gazebo::Factory (_ecm,
    _eventMgr);
  factory.CreateEntities (sdf_world);


  // TODO Load all models in SDF into the world. How? Talk to ECM? Does ECM have
  //   a function to just take an SDF and spawn everything??
  //   Factory.cc? SceneManager.cc? SimulationRunnher.hh? Server.hh includes sdf/Root
  // Loop through models in SDF. Add models and light
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

    //ignition::gazebo::components::Light
    // From Factory.cc
    //Entity Factory::CreateEntities(const sdf::Light *_light)
    //ignition::gazebo::components::Model

    ComponentType data = ?
    ComponentKey ck = _ecm.CreateComponent (eid, data);
  }
  */
  ignerr << _ecm.EntityCount () << " entities" << std::endl;


  ++(this->iter);
  this->printedEnd = false;
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

  // Process one timestamp per Update() step
  if (this->iter == this->poseBatch.end ())
  {
    // Print only once
    if (! this->printedEnd)
    {
      ignerr << "Finished playing all recorded data\n";
      this->printedEnd = true;
    }
    return;
  }

  // Print timestamp
  //ignerr << this->iter->TimeReceived ().count () << std::endl;
  ++(this->iter);

}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
