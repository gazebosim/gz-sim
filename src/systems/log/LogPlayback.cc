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

#include "LogPlayback.hh"

#include <fstream>
#include <string>

#include <ignition/plugin/RegisterMore.hh>

// To read contents in a .tlog database file
// #include <ignition/transport/log/Descriptor.hh>
#include <ignition/transport/log/QueryOptions.hh>
#include <ignition/transport/log/Message.hh>
#include <sdf/Root.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
// #include <ignition/common/Time.hh>
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"


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

void LogPlayback::parsePose(EntityComponentManager &_ecm)
{
  // TODO(mabelmzhang): Parse iter->Type() to get substring after last ".",
  //   to know what message type to create. For now just assuming Pose_V.

  // Protobuf message
  ignition::msgs::Pose_V posev_msg;
  // Convert binary bytes in string into a ign-msgs msg
  posev_msg.ParseFromString(this->iter->Data());

  igndbg << "Pose_V size: " << posev_msg.pose_size() << std::endl;

  for (int i = 0; i < posev_msg.pose_size(); i ++)
  {
    ignition::msgs::Pose pose = posev_msg.pose(i);

    // igndbg << pose.name() << std::endl;

    // TODO(mabelmzhang): Pose ign-msgs do not have parent information, so if
    //   two links of different models are of same name, there is no way to
    //   distinguish between them. Therefore link names in SDF must be
    //   different, until ECM is serialized.

    // Update link pose in map
    this->name_to_pose.insert_or_assign(pose.name(), pose);
  }


  // Loop through actual models in world
  _ecm.Each<components::Model, components::Name, components::ParentEntity,
               components::Pose>(
      [&](const Entity &/*_entity*/, components::Model *,
          components::Name *_nameComp,
          components::ParentEntity * /*_parentComp*/,
          components::Pose *_poseComp) -> bool
  {
    igndbg << "Model " << _nameComp->Data() << std::endl;
    igndbg << "Actual pose: \n";
    igndbg << _poseComp->Data().Pos() << std::endl;
    igndbg << _poseComp->Data().Rot().X() << " "
           << _poseComp->Data().Rot().Y() << " "
           << _poseComp->Data().Rot().Z() << " "
           << _poseComp->Data().Rot().W() << std::endl;


    // Look for model pose in log entry loaded
    ignition::msgs::Pose pose = this->name_to_pose.at(_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(ignition::math::Pose3d(
      ignition::math::Vector3(pose.position().x(), pose.position().y(),
                              pose.position().z()),
      ignition::math::Quaternion(pose.orientation().w(), pose.orientation().x(),
                 pose.orientation().y(), pose.orientation().z())));

    return true;
  });


  /*
  // Loop through actual links in world
  // TODO(mabelmzhang): Use parentComp to distinguish between Links with same
  //   name for different Models.
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
    ignition::msgs::Pose pose = this->name_to_pose.at(_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(ignition::math::Pose3d(
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


  // Call Log.hh directly to load a .tlog file

  this->log.reset(new Log());
  this->log->Open(this->logPath);

  // Access messages in .tlog file
  TopicList opts = TopicList("/world/default/pose/info");
  this->poseBatch = this->log->QueryMessages(opts);
  this->iter = this->poseBatch.begin();

  // Record first timestamp
  this->logStartTime = this->iter->TimeReceived();
  igndbg << this->logStartTime.count() << std::endl;
  igndbg << this->iter->Type() << std::endl;

  parsePose(_ecm);


  // Load recorded SDF file

  sdf::Root root;
  if (root.Load(this->sdfPath).size() != 0)
  {
    ignerr << "Error loading SDF file " << this->sdfPath << std::endl;
    return;
  }
  igndbg << "World count: " << root.WorldCount() << std::endl;
  igndbg << "Model count: " << root.ModelCount() << std::endl;
  const sdf::World * sdf_world = root.WorldByIndex(0);

  // Look for LogRecord plugin in the SDF and remove it, so that playback
  //   is not re-recorded. The SDF necessarily contains the recorder, which
  //   produced the log file this plugin is playing back.
  if (sdf_world->Element()->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElt = sdf_world->Element()->GetElement("plugin");
    // If never found, nothing to remove
    while (pluginElt != sdf::ElementPtr(nullptr))
    {
      if (pluginElt->HasAttribute("name"))
      {
        if (pluginElt->GetAttribute("name")->GetAsString().find ("LogRecord")
          == std::string::npos)
        {
          // Go to next plugin
          pluginElt = pluginElt->GetNextElement("plugin");
        }
        // If found it, remove it
        else
        {
          ignerr << "Found LogRecord plugin\n";
          pluginElt->RemoveFromParent();
          ignerr << "Removed LogRecord plugin from loaded SDF\n";
          break;
        }
      }
    }
  }


  // size_t nEntities = _ecm.EntityCount();
  ignerr << _ecm.EntityCount() << " entities" << std::endl;
  // Create all Entities in SDF <world> tag
  ignition::gazebo::SdfEntityCreator creator =
    ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);
  creator.CreateEntities(sdf_world);


  this->worldStartTime = std::chrono::high_resolution_clock::now();

  // Advance one entry in batch for Update()
  ++(this->iter);
  this->printedEnd = false;
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  // Sanity check. If reached the end, done.
  if (this->iter == this->poseBatch.end())
  {
    // Print only once
    if (!this->printedEnd)
    {
      ignmsg << "Finished playing all recorded data\n";
      this->printedEnd = true;
    }
    return;
  }


  // If timestamp since start of program has exceeded next logged timestamp,
  //   play the joint positions at next logged timestamp.

  auto now = std::chrono::high_resolution_clock::now();
  auto diff_time = std::chrono::duration_cast <std::chrono::nanoseconds>(
    now - this->worldStartTime);

  if (diff_time.count() >=
    (this->iter->TimeReceived().count() - this->logStartTime.count()))
  {
    // Print timestamp of this log entry
    igndbg << this->iter->TimeReceived().count() << std::endl;

    // Parse pose and move link
    parsePose(_ecm);
 
    // Advance one entry in batch for next Update() iteration
    // Process one log entry per Update() step.
    ++(this->iter);
  }
  // Else nothing to play
  else
    return;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
