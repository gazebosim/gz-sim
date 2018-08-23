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
#ifndef IGNITION_GAZEBO_COMPONENTS_PARENTENTITY_HH_
#define IGNITION_GAZEBO_COMPONENTS_PARENTENTITY_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
namespace components
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class ParentEntityPrivate;

  /// \brief This component holds the ID of its parent entity.
  class IGNITION_GAZEBO_VISIBLE ParentEntity
  {
    /// \brief Constructor
    /// \param[in] _id The parent entity's id
    public: explicit ParentEntity(const EntityId &_id);

    /// \brief Destructor
    public: virtual ~ParentEntity();

    /// \brief Copy Constructor
    /// \param[in] _parentEntity ParentEntity component to copy.
    public: ParentEntity(const ParentEntity &_parentEntity);

    /// \brief Move Constructor
    /// \param[in] _parentEntity ParentEntity component to move.
    public: ParentEntity(ParentEntity &&_parentEntity) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _parentEntity ParentEntity component to move.
    /// \return Reference to this.
    public: ParentEntity &operator=(ParentEntity &&_parentEntity);

    /// \brief Copy assignment operator.
    /// \param[in] _parentEntity ParentEntity component to copy.
    /// \return Reference to this.
    public: ParentEntity &operator=(const ParentEntity &_parentEntity);

    /// \brief Get the parent entity's ID.
    /// \return The parent entity's ID.
    public: const EntityId &Id() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ParentEntityPrivate> dataPtr;
  };
  }
}
}
}
#endif
