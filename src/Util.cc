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


#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
math::Pose3d Util::WorldPose(const Entity &_entity,
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


