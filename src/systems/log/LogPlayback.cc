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
#include <sdf/Root.hh>
#include "ignition/gazebo/Factory.hh"
//#include <ignition/common/Time.hh>
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>


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

void LogPlayback::parsePose (EntityComponentManager &_ecm)
{
  // TODO: Parse iter->Type () to get substring after last ".", to know what
  //   message type to create!
  //   For now just assuming Pose_V

  // Protobuf message
  ignition::msgs::Pose_V posev_msg;
  // Convert binary bytes in string into a ign-msgs msg
  posev_msg.ParseFromString (this->iter->Data());

  igndbg << "Pose_V size: " << posev_msg.pose_size () << std::endl;

  for (int i = 0; i < posev_msg.pose_size (); i ++)
  {
    ignition::msgs::Pose pose = posev_msg.pose (i);

    igndbg << pose.name () << std::endl;
    //igndbg << pose.id () << std::endl;

    // TODO: Links do not have parent information, so if two links of different
    //   models are of same name, there is no way to distinguish between them.
    //   Therefore link names in SDF must be different.

    // Update link pose in map
    this->name_to_pose.insert_or_assign (pose.name (), pose);

    /*
    igndbg << pose.position ().x () << ", " << pose.position ().y () << ", "
      << pose.position ().z () << std::endl;
    igndbg << pose.orientation ().x () << ", " << pose.orientation ().y ()
      << ", " << pose.orientation ().z () << ", " << pose.orientation ().w ()
      << std::endl;
    */
  }


  // Loop through actual models in world
  _ecm.Each<components::Model, components::Name, components::ParentEntity,
               components::Pose>(
      [&](const Entity &_entity, components::Model *,
          components::Name *_nameComp,
          components::ParentEntity *_parentComp,
          components::Pose *_poseComp) -> bool
  {
    igndbg << "Model " << _nameComp->Data() << std::endl;
    // This prints a 6-tuple. components::Pose is a
    //   SimpleWrapper around ignition::math::Pose3d, whose Rot() returns
    //   math::Quaternion, whose operator<<() prints rpy.
    //igndbg << "Actual pose: " << _poseComp->Data() << std::endl;
    // Explicitly print 7-tuple
    igndbg << "Actual pose: \n";
    igndbg << _poseComp->Data().Pos() << std::endl;
    igndbg << _poseComp->Data().Rot().X() << " "
           << _poseComp->Data().Rot().Y() << " "
           << _poseComp->Data().Rot().Z() << " "
           << _poseComp->Data().Rot().W() << std::endl;


    // Look for model pose in log entry loaded
    ignition::msgs::Pose pose = this->name_to_pose.at (_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose (ignition::math::Pose3d (
      ignition::math::Vector3(pose.position().x(), pose.position().y(),
                              pose.position().z()),
      ignition::math::Quaternion(pose.orientation().w(), pose.orientation().x(),
                 pose.orientation().y(), pose.orientation().z())));

    return true;
  });


  /*
  // Loop through actual links in world
  // TODO: Use parentComp to distinguish between Links with same name for
  //   different Models!
  _ecm.Each<components::Link, components::Name, components::ParentEntity,
               components::Pose>(
      [&](const Entity &_entity, components::Link *,
          components::Name *_nameComp,
          components::ParentEntity *_parentComp,
          components::Pose *_poseComp) -> bool
  {
    igndbg << "Link " << _nameComp->Data() << std::endl;
    // This prints a 6-tuple. Not sure why not 7. components::Pose is a
    //   SimpleWrapper around ignition::math::Pose3d, whose Rot() returns
    //   quaternion.
    //igndbg << "Actual pose: " << _poseComp->Data() << std::endl;
    // Print 7-tuple
    igndbg << "Actual pose: \n";
    igndbg << _poseComp->Data().Pos() << std::endl;
    igndbg << _poseComp->Data().Rot() << std::endl;


    // Look for the link poses in log entry loaded
    ignition::msgs::Pose pose = this->name_to_pose.at (_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose (ignition::math::Pose3d (
      ignition::math::Vector3(pose.position().x(), pose.position().y(),
                              pose.position().z()),
      ignition::math::Quaternion(pose.orientation().w(), pose.orientation().x(),
                 pose.orientation().y(), pose.orientation().z())));

    return true;
  });
  */
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
  ignmsg << "Playing back log file " << this->logPath << std::endl;


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
      ignmsg << "Starting playback\n";
    }
  }
  */


  // Use ECM

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
  //igndbg << "row " << row_i << std::endl;

  // Access messages in .tlog file
  TopicList opts = TopicList ("/world/default/pose/info");
  this->poseBatch = this->log->QueryMessages (opts);
  this->iter = this->poseBatch.begin ();

  // Record first timestamp
  this->logStartTime = this->iter->TimeReceived ();
  igndbg << this->logStartTime.count () << std::endl;
  igndbg << this->iter->Type () << std::endl;

  parsePose (_ecm);


  // Load recorded SDF file

  // TODO: Not sure if this fixes the problem that sometimes the SDF objects
  //   get loaded into the world, sometimes not.
  //   If move this block to start of function, then mostly it doesn't load.
  //   Look into why.
  // Wait a little, else SDF objects don't load
  ignition::common::Time::Sleep(ignition::common::Time(1));

  sdf::Root root;
  if (root.Load (this->sdfPath).size () != 0)
  {
    ignerr << "Error loading SDF file " << this->sdfPath << std::endl;
    return;
  }
  igndbg << "World count: " << root.WorldCount () << std::endl;
  igndbg << "Model count: " << root.ModelCount () << std::endl;
  const sdf::World * sdf_world = root.WorldByIndex (0);

  // TODO: Look for LogRecord plugin in the SDF, and remove that <plugin>,
  //   so that recorder isn't also loaded! It necessarily is in the SDF,
  //   because it was loaded in the original SDF to record the log file.
  // TODO Hardcoding name for now. Ideally can do regex *LogRecord
  /*
  sdf::ElementPtr recordPlugin = sdf_world->Element()->GetElement (
    "ignition::gazebo::systems::v0::LogRecord");
  if (recordPlugin != NULL)
  {
    recordPlugin->RemoveFromParent ();
    ignerr << "Removing LogRecord plugin from loaded SDF\n";
  }
  */
  

  // TODO: Use latest version, Factory is renamed
  // Create all Entities in SDF <world> tag
  ignition::gazebo::Factory factory = ignition::gazebo::Factory (_ecm,
    _eventMgr);
  factory.CreateEntities (sdf_world);


  // TODO: Check for whether world is running, start when it starts running!
  /* Too long.
  std::chrono::time_point<std::chrono::system_clock> now =
    std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  this->worldStartTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
    duration);
  */
  this->worldStartTime = std::chrono::high_resolution_clock::now();

  // Advance one entry in batch for Update()
  ++(this->iter);
  this->printedEnd = false;
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  //std::ifstream ifs(this->logPath);
  //ifs >> _ecm;


  // Use ign-transport playback - probably won't.
  // Subscribe to a topic, then call ECM to override states - no idea what that means.


  // Use ECM

  // Sanity check. If reached the end, done.
  if (this->iter == this->poseBatch.end ())
  {
    // Print only once
    if (! this->printedEnd)
    {
      ignmsg << "Finished playing all recorded data\n";
      this->printedEnd = true;
    }
    return;
  }


  // If timestamp since start of program has exceeded next logged timestamp,
  //   play the joint positions at next logged timestamp.

  auto now = std::chrono::high_resolution_clock::now ();
  auto diff_time = std::chrono::duration_cast <std::chrono::nanoseconds> (
    now - this->worldStartTime);

  if (diff_time.count () >=
      (this->iter->TimeReceived ().count () - this->logStartTime.count ()))
  {
    // Print timestamp of this log entry
    igndbg << this->iter->TimeReceived ().count () << std::endl;

    // Parse pose and move link
    parsePose (_ecm);
 
    // Advance one entry in batch for next Update() iteration
    // Process one log entry per Update() step.
    ++(this->iter);
  }
  // Else nothing to play
  else
    return;



  /*
  // Models
  _ecm.Each<components::Model, components::Name,
               components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
  {
    igndbg << "Entity " << _entity << ": " << _nameComp->Data() << std::endl;
    igndbg << "Pose: " << _poseComp->Data() << std::endl;

    //_ecm.Component (_entity)

    return true;
  });

  // Joints
  _ecm.Each<components::Joint, components::Name, components::ParentEntity,
               components::Pose>(
      [&](const Entity &_entity, const components::Joint *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
  {
    igndbg << "Joint " << _nameComp->Data() << std::endl;
    igndbg << "Pose: " << _poseComp->Data() << std::endl;

     return true;
  });
  */


}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
