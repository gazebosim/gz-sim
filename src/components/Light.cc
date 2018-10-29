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

#include "ignition/gazebo/components/Light.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::LightPrivate
{
  /// \brief Constructor.
  /// \param[in] _light Light data.
  public: explicit LightPrivate(const sdf::Light &_light)
          : light(_light)
  {
  }

  /// \brief The light data.
  public: sdf::Light light;
};

//////////////////////////////////////////////////
Light::Light(const sdf::Light &_light)
  : dataPtr(std::make_unique<LightPrivate>(_light))
{
}

//////////////////////////////////////////////////
Light::~Light()
{
}

//////////////////////////////////////////////////
Light::Light(const Light &_light)
  : dataPtr(std::make_unique<LightPrivate>(_light.Data()))
{
}

//////////////////////////////////////////////////
Light::Light(Light &&_light) noexcept
  : dataPtr(std::move(_light.dataPtr))
{
}

//////////////////////////////////////////////////
Light &Light::operator=(Light &&_light)
{
  this->dataPtr = std::move(_light.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Light &Light::operator=(const Light &_light)
{
  this->dataPtr->light = _light.Data();
  return *this;
}

//////////////////////////////////////////////////
const sdf::Light &Light::Data() const
{
  return this->dataPtr->light;
}

