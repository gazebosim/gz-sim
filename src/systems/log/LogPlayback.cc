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

#include <string>
#include <fstream>
#include <filesystem>

#include <ignition/msgs/pose_v.pb.h>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/RegisterMore.hh>

#include <ignition/transport/log/QueryOptions.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Message.hh>

#include <sdf/Root.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private LogPlayback data class.
class ignition::gazebo::systems::LogPlaybackPrivate
{
  /// \brief Reads the next pose message in log file, set poses in world
  public: void ParsePose(EntityComponentManager &_ecm);

  /// \brief Name of recorded log file to play back
  public: std::string logPath;

  /// \brief Name of recorded SDF file
  public: std::string sdfPath;

  /// \brief Log object to read ign-transport log file
  public: std::unique_ptr <transport::log::Log> log;

  /// \brief A batch of data from log file, of all pose messages
  public: transport::log::Batch poseBatch;

  /// \brief Iterator to go through messages in Batch
  public: transport::log::MsgIter iter;

  /// \brief First timestamp in log file
  public: std::chrono::nanoseconds logStartTime;

  /// \brief Timestamp when plugin started
  public: std::chrono::time_point<std::chrono::system_clock> worldStartTime;

  /// \brief Flag to print finish message once
  public: bool printedEnd;

  // Key: link name. Value: link pose
  /// \brief Maps link name to link pose recorded
  public: std::map <std::string, msgs::Pose> nameToPose;
};


//////////////////////////////////////////////////
LogPlayback::LogPlayback()
  : System(), dataPtr(std::make_unique<LogPlaybackPrivate>())
{
}

//////////////////////////////////////////////////
LogPlayback::~LogPlayback() = default;

//////////////////////////////////////////////////
void LogPlaybackPrivate::ParsePose(EntityComponentManager &_ecm)
{
  // TODO(mabelmzhang): Parse iter->Type() to get substring after last ".",
  //   to know what message type to create. For now just assuming Pose_V.

  // Protobuf message
  msgs::Pose_V posevMsg;
  // Convert binary bytes in string into a ign-msgs msg
  posevMsg.ParseFromString(this->iter->Data());

  igndbg << "Pose_V size: " << posevMsg.pose_size() << std::endl;

  for (int i = 0; i < posevMsg.pose_size(); ++i)
  {
    msgs::Pose pose = posevMsg.pose(i);

    // igndbg << pose.name() << std::endl;

    // TODO(mabelmzhang): Pose ign-msgs do not have parent information, so if
    //   two links of different models are of same name, there is no way to
    //   distinguish between them. Therefore link names in SDF must be
    //   different, until ECM is serialized.

    // Update link pose in map
    this->nameToPose.insert_or_assign(pose.name(), pose);
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
    msgs::Pose pose = this->nameToPose.at(_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(math::Pose3d(
      math::Vector3(pose.position().x(), pose.position().y(),
                              pose.position().z()),
      math::Quaternion(pose.orientation().w(), pose.orientation().x(),
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
    //   SimpleWrapper around math::Pose3d, whose Rot() returns
    //   quaternion.
    //igndbg << "Actual pose: " << _poseComp->Data() << std::endl;
    // Print 7-tuple
    igndbg << "Actual pose: \n";
    igndbg << _poseComp->Data().Pos() << std::endl;
    igndbg << _poseComp->Data().Rot() << std::endl;


    // Look for the link poses in log entry loaded
    msgs::Pose pose = this->nameToPose.at(_nameComp->Data());

    igndbg << "Recorded pose: " << std::endl;
    igndbg << pose.position().x() << ", " << pose.position().y() << ", "
      << pose.position().z() << std::endl;
    igndbg << pose.orientation().x() << ", " << pose.orientation().y()
      << ", " << pose.orientation().z() << ", " << pose.orientation().w()
      << std::endl;


    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(math::Pose3d(
      math::Vector3(pose.position().x(), pose.position().y(),
                              pose.position().z()),
      math::Quaternion(pose.orientation().w(), pose.orientation().x(),
                 pose.orientation().y(), pose.orientation().z())));

    return true;
  });
  */
}

//////////////////////////////////////////////////
void LogPlayback::Configure(const Entity &_worldEntity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  // Get params from SDF
  this->dataPtr->logPath = _sdf->Get<std::string>("log_path",
    this->dataPtr->logPath).first;
  this->dataPtr->sdfPath = _sdf->Get<std::string>("sdf_path",
    this->dataPtr->sdfPath).first;

  if (this->dataPtr->logPath.empty() || this->dataPtr->sdfPath.empty())
  {
    ignerr << "Unspecified log path to playback. Nothing to play.\n";
    return;
  }

  if (!std::filesystem::exists(this->dataPtr->logPath) ||
      !std::filesystem::exists(this->dataPtr->sdfPath))
  {
    ignerr << "log_path and/or sdf_path invalid. File(s) do not exist. "
      << "Nothing to play.\n";
    return;
  }


  ignmsg << "Playing back log file " << this->dataPtr->logPath << std::endl;

  // Call Log.hh directly to load a .tlog file

  this->dataPtr->log = std::make_unique<transport::log::Log>();
  this->dataPtr->log->Open(this->dataPtr->logPath);

  // Access messages in .tlog file
  transport::log::TopicList opts("/world/default/pose/info");
  this->dataPtr->poseBatch = this->dataPtr->log->QueryMessages(opts);
  this->dataPtr->iter = this->dataPtr->poseBatch.begin();

  // Record first timestamp
  this->dataPtr->logStartTime = this->dataPtr->iter->TimeReceived();
  igndbg << this->dataPtr->logStartTime.count() << std::endl;
  igndbg << this->dataPtr->iter->Type() << std::endl;

  this->dataPtr->ParsePose(_ecm);

  // Load recorded SDF file

  sdf::Root root;
  if (root.Load(this->dataPtr->sdfPath).size() != 0)
  {
    ignerr << "Error loading SDF file " << this->dataPtr->sdfPath << std::endl;
    return;
  }
  igndbg << "World count: " << root.WorldCount() << std::endl;
  igndbg << "Model count: " << root.ModelCount() << std::endl;
  const sdf::World * sdfWorld = root.WorldByIndex(0);

  // Look for LogRecord plugin in the SDF and remove it, so that playback
  //   is not re-recorded.
  if (sdfWorld->Element()->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");
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
          igndbg << "Found LogRecord plugin\n";
          pluginElt->RemoveFromParent();
          igndbg << "Removed LogRecord plugin from loaded SDF\n";
          break;
        }
      }
    }
  }


  igndbg << _ecm.EntityCount() << " entities" << std::endl;
  // Create all Entities in SDF <world> tag
  gazebo::SdfEntityCreator creator =
    gazebo::SdfEntityCreator(_ecm, _eventMgr);

  // Models
  for (uint64_t modelIndex = 0; modelIndex < sdfWorld->ModelCount();
      ++modelIndex)
  {
    auto model = sdfWorld->ModelByIndex(modelIndex);
    auto modelEntity = creator.CreateEntities(model);

    creator.SetParent(modelEntity, _worldEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < sdfWorld->LightCount();
      ++lightIndex)
  {
    auto light = sdfWorld->LightByIndex(lightIndex);
    auto lightEntity = creator.CreateEntities(light);

    creator.SetParent(lightEntity, _worldEntity);
  }

  _eventMgr.Emit<events::LoadPlugins>(_worldEntity, sdfWorld->Element());

  this->dataPtr->worldStartTime = std::chrono::high_resolution_clock::now();

  // Advance one entry in batch for Update()
  ++(this->dataPtr->iter);
  this->dataPtr->printedEnd = false;
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  // Sanity check. If reached the end, done.
  if (this->dataPtr->iter == this->dataPtr->poseBatch.end())
  {
    // Print only once
    if (!this->dataPtr->printedEnd)
    {
      ignmsg << "Finished playing all recorded data\n";
      this->dataPtr->printedEnd = true;
    }
    return;
  }


  // If timestamp since start of program has exceeded next logged timestamp,
  //   play the joint positions at next logged timestamp.

  auto now = std::chrono::high_resolution_clock::now();
  auto diffTime = std::chrono::duration_cast <std::chrono::nanoseconds>(
    now - this->dataPtr->worldStartTime);

  if (diffTime.count() >= (this->dataPtr->iter->TimeReceived().count() -
    this->dataPtr->logStartTime.count()))
  {
    // Print timestamp of this log entry
    igndbg << this->dataPtr->iter->TimeReceived().count() << std::endl;

    // Parse pose and move link
    this->dataPtr->ParsePose(_ecm);

    // Advance one entry in batch for next Update() iteration
    // Process one log entry per Update() step.
    ++(this->dataPtr->iter);
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogPlayback,
                    ignition::gazebo::System,
                    LogPlayback::ISystemConfigure,
                    LogPlayback::ISystemUpdate)
