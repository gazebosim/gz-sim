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
#ifndef IGNITION_GAZEBO_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_COMPONENT_TYPE_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // Forward declarations.
    class ComponentTypePrivate;

    /// \brief Base class for all component types. A component should
    /// consist of only data. Multiple components can be assigned to an
    /// Entity. Systems, such as a physics system, will read and potentially
    /// modify components.
    class IGNITION_GAZEBO_VISIBLE ComponentType
    {
      /// \brief Constructor.
      /// \param[in] _typeId Component Type Id, which can be retrieved from
      /// EntityComponentManager::Register().
      public: explicit ComponentType(const ComponentTypeId &_typeId);

      /// \brief Destructor.
      public: virtual ~ComponentType();

      /// \brief Get the name of the component type.
      /// \return Name of the component type
      public: virtual const std::string &Name() const = 0;

      /// \brief Get the component type id.
      /// \return Component type id.
      public: const ComponentTypeId &TypeId() const;

      /// \brief Returns whether or not this component is valid. This is
      /// equalivent to TypeId() != kComponentTypeIdInvalid.
      /// \return True if the component is valid.
      public: bool Valid() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<ComponentTypePrivate> dataPtr;
    };
    }
  }
}
#endif
