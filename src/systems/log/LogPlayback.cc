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

#include <ignition/msgs/pose_v.pb.h>

#include <chrono>
#include <string>

#include <ignition/msgs/Utility.hh>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/RegisterMore.hh>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Time.hh>
#include <ignition/transport/log/QueryOptions.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Message.hh>

#include <sdf/Root.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private LogPlayback data class.
class ignition::gazebo::systems::LogPlaybackPrivate
{
  /// \brief Reads the next pose message in log file, set poses in world
  public: void ParsePose(EntityComponentManager &_ecm);

  /// \brief Log object to read ign-transport log file
  public: std::unique_ptr <transport::log::Log> log;

  /// \brief A batch of data from log file, of all pose messages
  public: transport::log::Batch poseBatch;

  /// \brief Iterator to go through messages in Batch
  public: transport::log::MsgIter iter;

  /// \brief First timestamp in log file
  public: std::chrono::nanoseconds logStartTime;

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

  for (int i = 0; i < posevMsg.pose_size(); ++i)
  {
    msgs::Pose pose = posevMsg.pose(i);

    // igndbg << pose.name() << std::endl;

    // TODO(mabelmzhang): Pose ign-msgs do not have parent information, so if
    //   two links of different models are of same name, there is no way to
    //   distinguish between them. Therefore link names in SDF must be
    //   different, until ECM is serialized.

    // Update entity pose in map
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
    // Look for model pose in log entry loaded
    msgs::Pose pose = this->nameToPose.at(_nameComp->Data());

    // Set current pose to recorded pose
    // Use copy assignment operator
    *_poseComp = components::Pose(msgs::Convert(pose));

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
    *_poseComp = components::Pose(msgs::Convert(pose));

    return true;
  });
  */
}

//////////////////////////////////////////////////
void LogPlayback::Configure(const Entity &_worldEntity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  // Get directory paths from SDF
  auto logPath = _sdf->Get<std::string>("path");

  if (logPath.empty())
  {
    ignerr << "Unspecified log path to playback. Nothing to play.\n";
    return;
  }

  if (!common::isDirectory(logPath))
  {
    ignerr << "Specified log path must be a directory.\n";
    return;
  }

  // Append file name
  std::string dbPath = common::joinPaths(logPath, "state.tlog");

  // Temporary. Name of recorded SDF file
  std::string sdfPath = common::joinPaths(logPath, "state.sdf");

  if (!common::exists(dbPath) ||
      !common::exists(sdfPath))
  {
    ignerr << "Log path invalid. File(s) do not exist. Nothing to play.\n";
    return;
  }

  // Load recorded SDF file
  sdf::Root root;
  if (root.Load(sdfPath).size() != 0 || root.WorldCount() <= 0)
  {
    ignerr << "Error loading SDF file [" << sdfPath << "]" << std::endl;
    return;
  }
  const sdf::World * sdfWorld = root.WorldByIndex(0);

  std::vector <sdf::ElementPtr> plugins_rm;

  // Look for LogRecord plugin in the SDF and remove it, so that playback
  //   is not re-recorded.
  // Remove Physics plugin, so that it does not clash with recorded poses.
  if (sdfWorld->Element()->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");

    // If never found, nothing to remove
    while (pluginElt != nullptr)
    {
      ignerr << pluginElt->ToString("");
      ignerr << pluginElt->GetName() << std::endl;

      if (pluginElt->HasAttribute("name"))
      {
        ignerr << pluginElt->GetAttribute("name")->GetAsString() << std::endl;

        if ((pluginElt->GetAttribute("name")->GetAsString().find("LogRecord")
          != std::string::npos) ||
          (pluginElt->GetAttribute("name")->GetAsString().find("Physics")
          != std::string::npos))
        {
          // Flag for removal.
          // Do not actually remove plugin from parent while looping through
          //   children of this parent. Else cannot access next element.
          plugins_rm.push_back(pluginElt);
        }
      }

      // Go to next plugin
      pluginElt = pluginElt->GetNextElement("plugin");
    }
  }

  // Remove the marked plugins
  for (std::vector<sdf::ElementPtr>::iterator it = plugins_rm.begin();
    it != plugins_rm.end(); ++it)
  {
    (*it)->RemoveFromParent();
    ignerr << "Removed " << (*it)->GetAttribute("name")->GetAsString()
      << " plugin from loaded SDF\n";
  }

  // Create all Entities in SDF <world> tag
  ignition::gazebo::SdfEntityCreator creator =
    ignition::gazebo::SdfEntityCreator(_ecm, _eventMgr);

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

  ignmsg << "Playing back log file [" << dbPath << "]" << std::endl;

  // Call Log.hh directly to load a .tlog file
  this->dataPtr->log = std::make_unique<transport::log::Log>();
  this->dataPtr->log->Open(dbPath);

  // Access messages in .tlog file
  transport::log::TopicList opts("/world/default/pose/info");
  this->dataPtr->poseBatch = this->dataPtr->log->QueryMessages(opts);
  this->dataPtr->iter = this->dataPtr->poseBatch.begin();

  // Record first timestamp
  this->dataPtr->logStartTime = this->dataPtr->iter->TimeReceived();

  this->dataPtr->ParsePose(_ecm);

  // Advance one entry in batch for Update()
  ++(this->dataPtr->iter);
  this->dataPtr->printedEnd = false;
}

//////////////////////////////////////////////////
void LogPlayback::Update(const UpdateInfo &_info,
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

  auto now = _info.simTime;
  if (now.count() >= (this->dataPtr->iter->TimeReceived().count() -
    this->dataPtr->logStartTime.count()))
  {
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

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogPlayback,
                          "ignition::gazebo::systems::LogPlayback")
