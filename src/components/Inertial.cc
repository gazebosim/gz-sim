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

#include <ignition/math/Inertial.hh>
#include "ignition/gazebo/components/Inertial.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::InertialPrivate
{
  /// \brief Constructor.
  /// \param[in] _inertial Inertial data.
  public: explicit InertialPrivate(const ignition::math::Inertiald &_inertial)
          : inertial(_inertial)
  {
  }

  /// \brief The inertial data.
  public: ignition::math::Inertiald inertial;
};

//////////////////////////////////////////////////
Inertial::Inertial(const ignition::math::Inertiald &_inertial)
  : dataPtr(std::make_unique<InertialPrivate>(_inertial))
{
}

//////////////////////////////////////////////////
Inertial::Inertial(const Inertial &_inertial)
  : dataPtr(std::make_unique<InertialPrivate>(_inertial.Data()))
{
}

//////////////////////////////////////////////////
Inertial::Inertial(Inertial &&_inertial) noexcept
  : dataPtr(std::move(_inertial.dataPtr))
{
}

//////////////////////////////////////////////////
Inertial::~Inertial()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Inertiald>(this->Name());
}

//////////////////////////////////////////////////
const ignition::math::Inertiald &Inertial::Data() const
{
  return this->dataPtr->inertial;
}

//////////////////////////////////////////////////
Inertial &Inertial::operator=(Inertial &&_inertial)
{
  this->dataPtr = std::move(_inertial.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Inertial &Inertial::operator=(const Inertial &_inertial)
{
  this->dataPtr->inertial = _inertial.Data();
  return *this;
}
