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
#include "ignition/gazebo/PoseComponentType.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::PoseComponentTypePrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit PoseComponentTypePrivate(const ignition::math::Pose3d &_pose)
          : pose(_pose)
  {
  }

  /// \brief Name of the component.
  public: std::string name{"PoseComponent"};

  /// \brief The pose data.
  public: ignition::math::Pose3d pose;
};

//////////////////////////////////////////////////
PoseComponentType::PoseComponentType(const ignition::math::Pose3d &_pose)
  : dataPtr(new PoseComponentTypePrivate(_pose))
{
}

//////////////////////////////////////////////////
PoseComponentType::PoseComponentType(const PoseComponentType &_pose)
  : dataPtr(new PoseComponentTypePrivate(_pose.Pose()))
{
}

//////////////////////////////////////////////////
PoseComponentType::PoseComponentType(PoseComponentType &&_pose)
  : dataPtr(std::move(_pose.dataPtr))
{
}

//////////////////////////////////////////////////
PoseComponentType::~PoseComponentType()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Pose3d>(this->Name());
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &PoseComponentType::Pose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
const std::string &PoseComponentType::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
PoseComponentType &PoseComponentType::operator=(PoseComponentType &&_pose)
{
  this->dataPtr = std::move(_pose.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
PoseComponentType &PoseComponentType::operator=(const PoseComponentType &_pose)
{
  this->dataPtr->pose = _pose.Pose();
  return *this;
}
