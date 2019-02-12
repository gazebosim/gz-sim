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
#include <filesystem>

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
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/SimpleWrapper.hh"
#include "ignition/gazebo/components/Joint.hh"


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
void LogRecord::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  // Get params from SDF
  this->logPath = _sdf->Get<std::string>("log_path",
      this->logPath).first;
  this->sdfPath = _sdf->Get<std::string>("sdf_path",
      this->sdfPath).first;

  // Check if files already exist, don't overwrite
  if (std::filesystem::exists (this->logPath) ||
      std::filesystem::exists (this->sdfPath))
  {
    ignerr << "log_path and/or sdf_path already exist on disk! Not overwriting. Will not record." << std::endl;
    return;
  }

  ignmsg << "Recording to log file " << this->logPath << std::endl;


  // Use ign-transport directly

  this->recorder.AddTopic("/world/default/pose/info");
  //this->recorder.AddTopic(std::regex(".*"));

  // This calls Log::Open() and loads 0.1.0.sql
  this->recorder.Start(this->logPath);


  // Use ECM

  // Entity is just an int
  igndbg << _ecm.EntityCount () << " entities" << std::endl;

  // Record SDF as a string.
  // TODO: For now, just dumping a big string to a text file, until we have a
  //   custom SQL field for the SDF.
  std::ofstream ofs(this->sdfPath);
  // Go up to root of SDF, to output entire SDF file
  sdf::ElementPtr sdf_root = _sdf->GetParent ();
  while (sdf_root->GetParent () != NULL)
  {
    sdf_root = sdf_root->GetParent ();
  }
  ofs << sdf_root->ToString ("");
  ignmsg << "Outputted SDF to " << this->sdfPath << std::endl;


}

//////////////////////////////////////////////////
void LogRecord::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  // Use ECM

  //igndbg << "Update()" << std::endl;

  //for (auto ent : _ecm.Entities ())
  /*
  // Models
  _ecm.EachNew<components::Model, components::Name,
               components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
  {
    igndbg << "Entity " << _entity << ": " << _nameComp->Data() << std::endl;
    igndbg << "Pose: " << _poseComp->Data() << std::endl;

    //_ecm.Component (ent)

    return true;
  });

  // Joints
  _ecm.EachNew<components::Joint, components::Name, components::ParentEntity,
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



  /*
  {
    std::ofstream ofs(this->logPath);
    ofs << _ecm;
  }
  {
    // Read the object back in.
    std::ifstream ifs(this->logPath);
    ifs >> _ecm;
  }
  */
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemUpdate)
