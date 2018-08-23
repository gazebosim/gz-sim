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

#include "ignition/gazebo/components/Geometry.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::GeometryPrivate
{
  /// \brief Constructor.
  /// \param[in] _geometry Geometry data.
  public: explicit GeometryPrivate(const sdf::Geometry &_geometry)
          : geometry(_geometry)
  {
  }

  /// \brief The geometry data.
  public: sdf::Geometry geometry;
};

//////////////////////////////////////////////////
Geometry::Geometry(const sdf::Geometry &_geometry)
  : dataPtr(std::make_unique<GeometryPrivate>(_geometry))
{
}

//////////////////////////////////////////////////
Geometry::~Geometry()
{
}

//////////////////////////////////////////////////
Geometry::Geometry(const Geometry &_geometry)
  : dataPtr(new GeometryPrivate(_geometry.Data()))
{
}

//////////////////////////////////////////////////
Geometry::Geometry(Geometry &&_geometry) noexcept
  : dataPtr(std::move(_geometry.dataPtr))
{
}

//////////////////////////////////////////////////
Geometry &Geometry::operator=(Geometry &&_geometry)
{
  this->dataPtr = std::move(_geometry.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Geometry &Geometry::operator=(const Geometry &_geometry)
{
  this->dataPtr->geometry = _geometry.Data();
  return *this;
}

//////////////////////////////////////////////////
const sdf::Geometry &Geometry::Data() const
{
  return this->dataPtr->geometry;
}

