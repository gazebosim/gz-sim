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

#include "ignition/gazebo/components/Name.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::NamePrivate
{
  /// \brief Constructor.
  /// \param[in] _name Name data.
  public: explicit NamePrivate(const std::string &_name)
          : name(_name)
  {
  }

  /// \brief The name data.
  public: std::string name;
};

//////////////////////////////////////////////////
Name::Name(const std::string &_name)
  : dataPtr(std::make_unique<NamePrivate>(_name))
{
}

//////////////////////////////////////////////////
Name::~Name()
{
}

//////////////////////////////////////////////////
Name::Name(const Name &_name)
  : dataPtr(std::make_unique<NamePrivate>(_name.Data()))
{
}

//////////////////////////////////////////////////
Name::Name(Name &&_name) noexcept
  : dataPtr(std::move(_name.dataPtr))
{
}

//////////////////////////////////////////////////
Name &Name::operator=(Name &&_name)
{
  this->dataPtr = std::move(_name.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Name &Name::operator=(const Name &_name)
{
  this->dataPtr->name = _name.Data();
  return *this;
}

//////////////////////////////////////////////////
const std::string &Name::Data() const
{
  return this->dataPtr->name;
}

