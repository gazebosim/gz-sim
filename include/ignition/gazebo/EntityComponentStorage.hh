/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_ENTITYCOMPONENTSTORAGE_HH_
#define IGNITION_GAZEBO_ENTITYCOMPONENTSTORAGE_HH_

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/config.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief Enum class that lets external users of EntityComponentStorage
    /// know what type of action took place when a component was added to an
    /// entity.
    enum class ComponentAdditionResult
    {
      /// \brief The type of the component added to an entity is a type that
      /// wasn't previously associated with the entity.
      NEW_ADDITION = 0,
      /// \brief The type of the component added to an entity was a previously
      /// existing component type for that entity.
      RE_ADDITION = 1,
      /// \brief The component was not added to the entity because the entity
      /// already has a valid instance of this component type. Instead, the data
      /// for the valid instance should be modified with the new component data.
      MODIFICATION = 2,
      /// \brief The attempt to add the component to the entity failed, probably
      /// because the entity does not exist.
      FAILED_ADDITION = 3
    };

    /// \brief A class that stores all components that have been created. This
    /// class also keeps track of the entity a component belongs to.
    class IGNITION_GAZEBO_VISIBLE EntityComponentStorage
    {
      /// \brief Add an entity to the component storage, so that newly created
      /// components for this entity can be mapped to the entity. This should be
      /// called whenever an entity is created.
      /// \param[in] _entity The entity
      /// \return True if _entity was succesfully added to the component
      /// storage. False if _entity already belongs to the component storage
      public: bool AddEntity(const Entity _entity);

      /// \brief Remove an entity and all of its component data. This should be
      /// called whenever an entity is deleted.
      /// \param[in] _entity The entity
      /// \return True if _entity was in the component storage and removed.
      /// False if _entity was not in the component storage
      public: bool RemoveEntity(const Entity _entity);

      /// \brief Add a component to an entity.
      /// \param[in] _entity The entity
      /// \param[in] _component The component
      /// \return The result of the attempted component addition
      public: ComponentAdditionResult AddComponent(const Entity _entity,
                  std::unique_ptr<components::BaseComponent> _component);

      /// \brief Remove a component of a particular type from an entity
      /// \param[in] _entity The entity
      /// \return True if _entity exists and a component of type _typeId was
      /// removed from it. False otherwise
      public: bool RemoveComponent(const Entity _entity,
                  const ComponentTypeId _typeId);

      /// \brief Get a pointer to a component of a particular type that belongs
      /// to an entity. This is useful for determining whether an entity has a
      /// reference to a particular component type or not, but says nothing
      /// about the state/validity of the component. The component's removed
      /// flag should be checked to learn more about the state/validity of the
      /// component.
      /// \param[in] _entity The entity
      /// \param[in] _typeId The type of component to be retrieved
      /// \return The pointer to a component of type _typeId that belongs to
      /// _entity, if it exists. Otherwise, nullptr. nullptr is also returned if
      /// _entity does not exist.
      /// \sa ValidComponent
      private: const components::BaseComponent *Component(const Entity _entity,
                  const ComponentTypeId _typeId) const;

      /// \brief non-const version of the Component method
      /// \sa Component
      private: components::BaseComponent *Component(const Entity _entity,
                  const ComponentTypeId _typeId);

      /// \brief Get a pointer to a component of a particular type that belongs
      /// to an entity, if the component pointer is valid. A valid component
      /// pointer is one that is not nullptr and is also one that does not have
      /// the removed flag set to true.
      /// \param[in] _entity The entity
      /// \param[in] _typeId The type of the component to retrieve
      /// \return The pointer to a component, if 1) the pointer isn't nullptr,
      /// and 2) the removed flag of the component pointer is false. Otherwise,
      /// nullptr is returned
      public: const components::BaseComponent *ValidComponent(
                  const Entity _entity, const ComponentTypeId _typeId) const;

      /// \brief non-const version of the ValidComponent method
      /// \sa ValidComponent
      public: components::BaseComponent *ValidComponent(const Entity _entity,
                  const ComponentTypeId _typeId);

      /// \brief A map of an entity to its components
      private: std::unordered_map<Entity,
               std::vector<std::unique_ptr<components::BaseComponent>>>
                 entityComponents;

      /// \brief A map that keeps track of where each type of component is
      /// located in the this->entityComponents vector. Since the
      /// this->entityComponents vector is of type BaseComponent, we need to
      /// keep track of which component type corresponds to a given index in
      /// the vector so that we can cast the BaseComponent to this type if
      /// needed.
      ///
      /// The key of this map is the Entity, and the value is a map of the
      /// component type to the corresponding index in the
      /// this->entityComponents vector (a component of a particular type is
      /// only a key for the value map if a component of this type exists in
      /// the this->entityComponents vector)
      private: std::unordered_map<Entity,
               std::unordered_map<ComponentTypeId, std::size_t>>
                                    componentTypeIndex;
    };
    }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
  }    // namespace gazebo
}      // namespace ignition
#endif
