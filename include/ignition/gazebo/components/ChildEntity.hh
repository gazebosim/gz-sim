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
#ifndef IGNITION_GAZEBO_COMPONENTS_CHILDENTITY_HH_
#define IGNITION_GAZEBO_COMPONENTS_CHILDENTITY_HH_

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
  class ChildEntityPrivate;

  /// \brief This component holds the ID of its child entity.
  class IGNITION_GAZEBO_VISIBLE ChildEntity
  {
    /// \brief Constructor
    /// \param[in] _id The child entity's id
    public: explicit ChildEntity(const EntityId &_id);

    /// \brief Destructor
    public: virtual ~ChildEntity();

    /// \brief Copy Constructor
    /// \param[in] _childEntity ChildEntity component to copy.
    public: ChildEntity(const ChildEntity &_childEntity);

    /// \brief Move Constructor
    /// \param[in] _childEntity ChildEntity component to move.
    public: ChildEntity(ChildEntity &&_childEntity) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _childEntity ChildEntity component to move.
    /// \return Reference to this.
    public: ChildEntity &operator=(ChildEntity &&_childEntity);

    /// \brief Copy assignment operator.
    /// \param[in] _childEntity ChildEntity component to copy.
    /// \return Reference to this.
    public: ChildEntity &operator=(const ChildEntity &_childEntity);

    /// \brief Get the child entity's ID.
    /// \return The child entity's ID.
    public: const EntityId &Id() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ChildEntityPrivate> dataPtr;
  };
  }
}
}
}
#endif
