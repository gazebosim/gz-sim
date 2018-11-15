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
#ifndef IGNITION_GAZEBO_ENTITY_HH_
#define IGNITION_GAZEBO_ENTITY_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

/// \brief This library is part of the [Ignition
/// Robotics](https://ignitionrobotics.org) project.
namespace ignition
{
  /// \brief Gazebo is a leading open source robotics simulator, that
  /// provides high fidelity physics, rendering, and sensor simulation.
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Adding component namespace information here because there is
    // currently no one component class that seems like a good place to hold
    // this documentation.
    /// \brief Components represent data, such as position information. An
    /// Entity usually has one or more associated components.
    ///
    /// The set of Components assigned to an Entity also act as a key.
    /// Systems process Entities based on their key. For example, a physics
    /// system may process only entities that have pose and inertia
    /// components.
    namespace components {}

    /// \brief An Entity is an id.
    using EntityId = int;

    /// \brief Indicates a non-existant or invalid Entity.
    const EntityId kNullEntity = -1;

    // Forward Declaration
    class EntityPrivate;

    /// \class Entity Entity.hh ignition/gazebo/Entity.hh
    /// \brief An Entity identifies a single object in simulation such as
    /// a model, link, or light. At its core, an Entity is just an identifier.
    ///
    /// An Entity usually has one or more associated Components. Components
    /// represent data, such as position information.
    ///
    /// The set of Components assigned to an Entity also act as a key.
    /// Systems process Entities based on their key. For example, a physics
    /// system may process only entities that have pose and inertia
    /// components.
    ///
    /// An Entity that needs to be identified and used by Systems should be
    /// created through the Server.
    class IGNITION_GAZEBO_VISIBLE Entity
    {
      /// \brief Default constructor
      public: Entity() = default;

      /// \brief Construct an Entity with an id value.
      /// \param[in] _id Id of an entity to create.
      public: explicit Entity(const EntityId _id);

      /// \brief Move constructor.
      /// \param[in] _entity Entity ID to copy.
      public: explicit Entity(Entity &&_entity) noexcept;

      /// \brief Equality operator. Checks if this Entity is equivalent to
      /// the provided Entity.
      /// \param[in] _entity Entity to compare.
      /// \returns true if the Entity Id's are the same.
      public: bool operator==(const Entity &_entity) const;

      /// \brief Move assignment operator.
      /// \param[in] _entity Entity to move.
      /// \return Reference to this object.
      public: Entity &operator=(Entity &&_entity);

      /// \brief Return id of entity.
      /// \return Id of this Entity.
      public: EntityId Id() const;

      // Note: We are not using the private data pattern on purpose. An Entity
      // should only have an Id.

      /// \brief Id of the entity
      private: EntityId id = kNullEntity;
    };
    }
  }
}
#endif
