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
#ifndef IGNITION_GAZEBO_ENTITYCOMPONENTMANAGER_HH_
#define IGNITION_GAZEBO_ENTITYCOMPONENTMANAGER_HH_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>
#include <ignition/common/Console.hh>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN EntityComponentManagerPrivate;

    /// \brief A unique identifier for an entity query.
    using EntityQueryId = int;

    /// \cond
    /// \brief All component instances of the same type are stored
    /// squentially in memory. This is a base class for storing components
    /// of a particular type.
    class IGNITION_GAZEBO_HIDDEN ComponentStorageBase
    {
      /// \brief Constructor
      public: ComponentStorageBase() = default;

      /// \brief Create a new component using the provided data.
      /// \param[in] _data Data used to construct the component.
      /// \return Id of the new component. kComponentIdInvalid is returned
      /// if the component could not be created.
      public: virtual ComponentId Create(const void *_data) = 0;

      /// \brief Remove a component based on an id.
      /// \param[in] _id Id of the component to remove.
      /// \return True if the component was removed.
      public: virtual bool Remove(const ComponentId _id) = 0;

      /// \brief Remove all components
      public: virtual void RemoveAll() = 0;

      /// \brief Get a component based on an id.
      /// \param[in] _id Id of the component to get.
      /// \return A pointer to the component, or nullptr if the component
      /// could not be found.
      public: virtual const void *Component(const ComponentId _id) const = 0;

      /// \brief Get a mutable component based on an id.
      /// \param[in] _id Id of the component to get.
      /// \return A pointer to the component, or nullptr if the component
      /// could not be found.
      public: virtual void *ComponentMutable(const ComponentId _id) const = 0;

      /// \brief Get the first component.
      /// \return First component or nullptr if there are no components.
      public: virtual void *First() = 0;

      /// \brief Mutex used to prevent data corruption.
      protected: mutable std::mutex mutex;
    };

    /// \brief Templated implementation of component storage.
    template<typename ComponentTypeT>
    class IGNITION_GAZEBO_HIDDEN ComponentStorage : public ComponentStorageBase
    {
      /// \brief Constructor
      public: explicit ComponentStorage()
              : ComponentStorageBase()
      {
      }

      // Documentation inherited.
      public: bool Remove(const ComponentId _id) override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        // Get an iterator to the component that should be removed.
        std::map<ComponentId, int>::iterator iter = this->idMap.find(_id);

        // Make sure the component exists.
        if (iter != this->idMap.end())
        {
          // Handle the case where there are more components than the
          // component to be removed
          if (this->components.size() > 1)
          {
            // Swap the component to be removed with the component at the
            // back of the vector.
            std::swap(this->components[iter->second],
                      this->components.back());

            // After the swap, we have to fix all the id mappings.
            for (std::map<ComponentId, int>::iterator idIter =
                 this->idMap.begin(); idIter != this->idMap.end(); ++idIter)
            {
              if (static_cast<unsigned int>(idIter->second) ==
                  this->components.size()-1)
              {
                idIter->second = iter->second;
              }
            }
          }

          // Remove the component.
          this->components.pop_back();

          // Remove the id mapping.
          this->idMap.erase(iter);
          return true;
        }
        return false;
      }

      // Documentation inherited.
      public: void RemoveAll() override final
      {
        this->idCounter = 0;
        this->idMap.clear();
        this->components.clear();
      }

      // Documentation inherited.
      public: ComponentId Create(const void *_data) override final
      {
        ComponentId result;  // = kComponentIdInvalid;

        std::lock_guard<std::mutex> lock(this->mutex);
        result = this->idCounter++;
        this->idMap[result] = this->components.size();
        // Copy the component
        this->components.push_back(std::move(
              ComponentTypeT(*static_cast<const ComponentTypeT*>(_data))));

        return result;
      }

      // Documentation inherited.
      public: void *ComponentMutable(const ComponentId _id) const override final
      {
        return const_cast<void *>(this->Component(_id));
      }

      // Documentation inherited.
      public: const void *Component(const ComponentId _id) const override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        std::map<ComponentId, int>::const_iterator iter = this->idMap.find(_id);

        if (iter != this->idMap.end())
        {
          return static_cast<const void *>(&this->components.at(iter->second));
        }
        return nullptr;
      }

      // Documentation inherited.
      public: void *First() override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (!this->components.empty())
          return static_cast<void *>(&this->components[0]);
        return nullptr;
      }

      /// \brief The id counter is used to get unique ids within this
      /// storage class.
      private: ComponentId idCounter = 0;

      /// \brief Map of ComponentId to Components (see the components vector).
      private: std::map<ComponentId, int> idMap;

      /// \brief Sequential storage of components.
      public: std::vector<ComponentTypeT> components;
    };
    /// \endcond

    /** \class EntityComponentManager EntityComponentManager.hh \
     * ignition/gazebo/EntityComponentManager.hh
    **/
    /// \brief The EntityComponentManager constructs, deletes, and returns
    /// components and entities.
    class IGNITION_GAZEBO_VISIBLE EntityComponentManager
    {
      /// \brief Constructor
      public: EntityComponentManager();

      /// \brief Destructor
      public: ~EntityComponentManager();

      /// \brief Creates a new Entity.
      /// \return An id for the Entity, or kNullEntity on failure.
      public: EntityId CreateEntity();

      /// \brief Get the number of entities on the server.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Delete an existing Entity.
      /// \returns True if the Entity existed and was deleted.
      /// \todo(nkoenig) Implement this function
      // public: bool EraseEntity(const EntityId _id);

      /// \brief Delete all entities.
      public: void EraseEntities();

      /// \brief Get whether an Entity exists.
      /// \param[in] _id Entity id to confirm.
      /// \return True if the Entity exists.
      public: bool HasEntity(const EntityId _id) const;

      /// \brief Get whether a component type has ever been created.
      /// \param[in] _typeId ID of the component type to check.
      /// \return True if the provided _typeId has been created.
      public: bool HasComponentType(const ComponentTypeId _typeId);

      /// \brief Check whether an entity has a specific component.
      /// \param[in] _id Id of the entity to check.
      /// \param[in] _key The component to check.
      /// \return True if the component key belongs to the entity.
      public: bool EntityHasComponent(const EntityId _id,
                  const ComponentKey &_key) const;

      /// \brief Check whether an entity has a specific component type.
      /// \param[in] _id Id of the entity to check.
      /// \param[in] _typeId Component type id to check.
      /// \return True if the entity exists and has at least one component
      /// with the provided type.
      public: bool EntityHasComponentType(const EntityId _id,
                  const ComponentTypeId &_typeId) const;

      /// \brief Get whether an entity has all the given component types.
      /// \param[in] _id Id of the Entity to check.
      /// \param[in] _types Component types to check that the Entity has.
      /// \return True if the given entity has all the given types.
      public: bool EntityMatches(EntityId _id,
        const std::set<ComponentTypeId> &_types) const;

      /// \brief Add an entity query. An internal copy of the query is made,
      /// and an Id of the internal query is returned.
      /// \param[in] _query The query to add.
      /// \return Id of the newly added query.
      public: EntityQueryId AddQuery(const EntityQuery &_query);

      /// \brief Get an entity query.
      /// \param[in] _id Id of the EntityQuery to retrieve.
      /// \return Optional reference to the query. The return value will be
      /// std::nullopt if the entity query does not exist.
      public: const std::optional<std::reference_wrapper<EntityQuery>> Query(
                  const EntityQueryId _id) const;

      /// \brief Remove a component from an entity based on a key.
      /// \param[in] _id Id of the entity.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return True if the entity and component existed and the component was
      ///  removed.
      public: bool RemoveComponent(
                  const EntityId _id, const ComponentKey &_key);

      /// \brief Get the type id of a component type. This is a convenience
      /// function that is equivalent to typeid(ComponentTypeT).hash_code().
      /// \return The ComponentTypeId associated with the provided
      /// ComponentTypeT.
      public: template<typename ComponentTypeT>
              static ComponentTypeId ComponentType()
      {
        // Get a unique identifier to the component type
        return typeid(ComponentTypeT).hash_code();
      }

      /// \brief Create a component of a particular type. This will copy the
      /// _data parameter.
      /// \param[in] _entityId Id of the Entity that will be associated with
      /// the component.
      /// \param[in] _data Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      public: template<typename ComponentTypeT>
              ComponentKey CreateComponent(const EntityId _entityId,
                  const ComponentTypeT &_data)
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentTypeT).hash_code();

        // Create the component storage if one does not exist for
        // the component type.
        if (!this->HasComponentType(typeId))
        {
          this->RegisterComponentType(typeId,
                new ComponentStorage<ComponentTypeT>());
        }

        return this->CreateComponentImplementation(_entityId, typeId, &_data);
      }

      /// \brief Get a component assigned to an entity based on a
      /// component type.
      /// \param[in] _id Id of the entity.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      public: template<typename ComponentType>
              const ComponentType *Component(const EntityId _id) const
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentType).hash_code();

        return static_cast<const ComponentType*>(
            this->ComponentImplementation(_id, typeId));
      }

      /// \brief Get a mutable component assigned to an entity based on a
      /// component type.
      /// \param[in] _id Id of the entity.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      public: template<typename ComponentTypeT>
              ComponentTypeT *ComponentMutable(const EntityId _id) const
      {
        return const_cast<ComponentTypeT*>(
            this->Component<ComponentTypeT>(_id));
      }

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *Component(const ComponentKey &_key) const
      {
        return static_cast<const ComponentTypeT *>(
            this->ComponentImplementation(_key));
      }

      /// \brief Get a mutable component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentTypeT>
              ComponentTypeT *ComponentMutable(const ComponentKey &_key) const
      {
        return const_cast<ComponentTypeT*>(
            this->Component<ComponentTypeT>(_key));
      }

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *First() const
      {
        return static_cast<const ComponentTypeT *>(
            this->First(this->ComponentType<ComponentTypeT>()));
      }

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      public: template<typename ComponentTypeT>
              ComponentTypeT *First()
      {
        return static_cast<ComponentTypeT *>(
            this->First(this->ComponentType<ComponentTypeT>()));
      }

      /// why is this required?
      private: template <typename T> struct identity { typedef T type; };

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      public: template<typename ...ComponentTypeTs>
              void Each(typename identity<std::function<
                  void(const EntityId &_entity,
                       const ComponentTypeTs *...)>>::type _f) const
      {
        // \todo(louise) We should create Views, which will replace Queries.
        // A View can be completely internal to EntityComponentManager.
        // A View will store the entities that match the provided
        // ComponenTypeTs.
        //
        // Instead of iterating over all the entities, this function can
        // call Each(blah) on the appropriate View. If the View doesn't
        // exist, then create the View.
        //
        // System's will then no longer create and register queries. The
        // "InitSystem" code can go away. Instead, a System can call "Each"
        // to get the desired entities.
        //
        // The reason to do this approach is:
        // 1. It's cleaner - the concept of queries and all of that
        // infrastructure can be hidden from the user.
        // 2. Systems can alter the entities they need more easily at run
        // time.
        // 3. A System can process multiple entities with different
        // component signatures in a single callback.
        // 4. This opens the door to supporting more complex like:
        // Each<ComponentTypeA, ComponentTypeB>().Or<ComponentTypeC>().
        //
        // Reference:
        // https://github.com/alecthomas/entityx/blob/master/entityx/Entity.h
        for (const Entity &entity : this->Entities())
        {
          auto types = std::set<ComponentTypeId>{
              this->ComponentType<ComponentTypeTs>()...};

          if (this->EntityMatches(entity.Id(), types))
          {
            _f(entity.Id(), this->Component<ComponentTypeTs>(entity.Id())...);
          }
        }
      }

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      private: void *First(const ComponentTypeId _componentTypeId);

      /// \brief Implmentation of CreateComponent.
      /// \param[in] _entityId Id of the Entity that will be associated with
      /// the component.
      /// \param[in] _componentTypeId Id of the component type.
      /// \param[in] _data Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      private: ComponentKey CreateComponentImplementation(
                   const EntityId _entityId,
                   const ComponentTypeId _componentTypeId,
                   const void *_data);

      /// \brief Get a component based on a component type.
      /// \param[in] _id Id of the entity.
      /// \param[in] _type Id of the component type.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      private: const void *ComponentImplementation(const EntityId _id,
                   const ComponentTypeId _type) const;

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      private: const void *ComponentImplementation(
                   const ComponentKey &_key) const;

      /// \brief Register a new component type.
      /// \param[in] _typeId Type if of the new component.
      /// \param[in] _type Pointer to the component storage. Ownership is
      /// transfered.
      private: void RegisterComponentType(
                   const ComponentTypeId _typeId,
                   ComponentStorageBase *_type);

      private: std::vector<Entity> &Entities() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<EntityComponentManagerPrivate> dataPtr;
    };
    }
  }
}
#endif
