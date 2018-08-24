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

#include <ignition/math/Pose3.hh>
#include "ignition/gazebo/components/Pose.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::PosePrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit PosePrivate(const ignition::math::Pose3d &_pose)
          : pose(_pose)
  {
  }

  /// \brief The pose data.
  public: ignition::math::Pose3d pose;
};

//////////////////////////////////////////////////
Pose::Pose(const ignition::math::Pose3d &_pose)
  : dataPtr(std::make_unique<PosePrivate>(_pose))
{
}

//////////////////////////////////////////////////
Pose::Pose(const Pose &_pose)
  : dataPtr(std::make_unique<PosePrivate>(_pose.Data()))
{
}

//////////////////////////////////////////////////
Pose::Pose(Pose &&_pose) noexcept
  : dataPtr(std::move(_pose.dataPtr))
{
}

//////////////////////////////////////////////////
Pose::~Pose()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Pose3d>(this->Name());
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Pose::Data() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
Pose &Pose::operator=(Pose &&_pose)
{
  this->dataPtr = std::move(_pose.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Pose &Pose::operator=(const Pose &_pose)
{
  this->dataPtr->pose = _pose.Data();
  return *this;
}
