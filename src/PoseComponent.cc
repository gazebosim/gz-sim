/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <string>

#include <ignition/math/Pose3.hh>
#include "ignition/gazebo/PoseComponent.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::PoseComponentPrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit PoseComponentPrivate(const ignition::math::Pose3d &_pose)
          : pose(_pose)
  {
  }

  /// \brief Name of the component.
  public: std::string name{"PoseComponent"};

  /// \brief The pose data.
  public: ignition::math::Pose3d pose;
};

//////////////////////////////////////////////////
PoseComponent::PoseComponent(const ignition::math::Pose3d &_pose)
  : dataPtr(new PoseComponentPrivate(_pose))
{
}

//////////////////////////////////////////////////
PoseComponent::PoseComponent(const PoseComponent &_pose)
  : dataPtr(new PoseComponentPrivate(_pose.Pose()))
{
}

//////////////////////////////////////////////////
PoseComponent::PoseComponent(PoseComponent &&_pose)
  : dataPtr(std::move(_pose.dataPtr))
{
}

//////////////////////////////////////////////////
PoseComponent::~PoseComponent()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Pose3d>(this->Name());
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &PoseComponent::Pose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
const std::string &PoseComponent::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
PoseComponent &PoseComponent::operator=(PoseComponent &&_pose)
{
  this->dataPtr = std::move(_pose.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
PoseComponent &PoseComponent::operator=(const PoseComponent &_pose)
{
  this->dataPtr->pose = _pose.Pose();
  return *this;
}
