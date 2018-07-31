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
#include "ignition/gazebo/ComponentType.hh"

using namespace ignition;
using namespace gazebo;

// Private data class
class ignition::gazebo::ComponentTypePrivate
{
  /// \brief Constructor
  /// \param[in] _typeId Component type id of this component.
  public: explicit ComponentTypePrivate(const ComponentTypeId &_typeId)
          : typeId(_typeId)
  {
  }

  /// \brief Type id of the component
  public: ComponentTypeId typeId;
};

//////////////////////////////////////////////////
ComponentType::ComponentType(const ComponentTypeId &_typeId)
  : dataPtr(new ComponentTypePrivate(_typeId))
{
}

//////////////////////////////////////////////////
ComponentType::~ComponentType()
{
}

//////////////////////////////////////////////////
bool ComponentType::Valid() const
{
  return this->dataPtr->typeId != kComponentTypeIdInvalid;
}

//////////////////////////////////////////////////
const ComponentTypeId &ComponentType::TypeId() const
{
  return this->dataPtr->typeId;
}
