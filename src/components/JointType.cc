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

#include "ignition/gazebo/components/JointType.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::JointTypePrivate
{
  /// \brief Constructor.
  /// \param[in] _jointType JointType data.
  public: explicit JointTypePrivate(const sdf::JointType &_jointType)
          : jointType(_jointType)
  {
  }

  /// \brief The jointType data.
  public: sdf::JointType jointType;
};

//////////////////////////////////////////////////
JointType::JointType(const sdf::JointType &_jointType)
  : dataPtr(std::make_unique<JointTypePrivate>(_jointType))
{
}

//////////////////////////////////////////////////
JointType::JointType(const JointType &_jointType)
  : dataPtr(std::make_unique<JointTypePrivate>(_jointType.Data()))
{
}

//////////////////////////////////////////////////
JointType::JointType(JointType &&_jointType) noexcept
  : dataPtr(std::move(_jointType.dataPtr))
{
}

//////////////////////////////////////////////////
JointType::~JointType()
{
}

//////////////////////////////////////////////////
const sdf::JointType &JointType::Data() const
{
  return this->dataPtr->jointType;
}

//////////////////////////////////////////////////
JointType &JointType::operator=(JointType &&_jointType)
{
  this->dataPtr = std::move(_jointType.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
JointType &JointType::operator=(const JointType &_jointType)
{
  this->dataPtr->jointType = _jointType.Data();
  return *this;
}
