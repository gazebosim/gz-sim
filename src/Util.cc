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

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "ignition/gazebo/Util.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
//////////////////////////////////////////////////
math::Pose3d worldPose(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  // work out pose in world frame
  math::Pose3d pose = _ecm.Component<components::Pose>(_entity)->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p)
  {
    // get pose of parent entity
    auto parentPose = _ecm.Component<components::Pose>(p->Data());
    if (!parentPose)
      break;
    // transform pose
    pose = pose + parentPose->Data();
    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }
  return pose;
}

//////////////////////////////////////////////////
std::string scopedName(const Entity &_entity,
    const EntityComponentManager &_ecm, const std::string &_delim,
    bool _includePrefix)
{
  std::string result;

  auto entity = _entity;

  while (true)
  {
    // Get entity name
    auto nameComp = _ecm.Component<components::Name>(entity);
    if (nullptr == nameComp)
      break;
    auto name = nameComp->Data();

    // Get entity type
    std::string prefix;
    if (_ecm.Component<components::World>(entity))
    {
      prefix = "world";
    }
    else if (_ecm.Component<components::Model>(entity))
    {
      prefix = "model";
    }
    else if (_ecm.Component<components::Light>(entity))
    {
      prefix = "light";
    }
    else if (_ecm.Component<components::Link>(entity))
    {
      prefix = "link";
    }
    else if (_ecm.Component<components::Collision>(entity))
    {
      prefix = "collision";
    }
    else if (_ecm.Component<components::Visual>(entity))
    {
      prefix = "visual";
    }
    else if (_ecm.Component<components::Joint>(entity))
    {
      prefix = "joint";
    }
    else if (_ecm.Component<components::Sensor>(entity))
    {
      prefix = "sensor";
    }
    else
    {
      ignwarn << "Skipping entity [" << name
              << "] when generating scoped name, entity type not known."
              << std::endl;
    }


    auto parentComp = _ecm.Component<components::ParentEntity>(entity);
    if (!prefix.empty())
    {
      result.insert(0, name);
      if (_includePrefix)
      {
        result.insert(0, _delim);
        result.insert(0, prefix);
      }
    }

    if (nullptr == parentComp)
      break;

    if (!prefix.empty())
      result.insert(0, _delim);

    entity = parentComp->Data();
  }

  return result;
}
}
}
}

