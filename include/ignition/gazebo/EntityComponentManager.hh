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
#include <type_traits>
#include <utility>
#include <vector>
#include <ignition/common/Console.hh>
#include "ignition/gazebo/Entity.hh"
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

    /// \cond
    /// \brief A key into the map of views
    using ComponentTypeKey = std::set<ComponentTypeId>;

    /// \brief A view is a cache to entities, and their components, that
    /// match a set of component types. A cache is used because systems will
    /// frequently, potentially every iteration, query the
    /// EntityComponentManager for sets of entities that match a set of
    /// component types. Rather than look up the entities every time, we can
    /// use a cache to improve performance. The assumption is that entities
    /// and the types of components assigned to entities change infrequently
    /// compared to the frequency of queries performed by systems.
    class IGNITION_GAZEBO_VISIBLE View
    {
      /// Get a pointer to a component for an entity based on a component type.
      /// \param[in] _id Id of the entity.
      /// \param[in] _ecm Pointer to the entity component manager.
      /// \return Pointer to the component.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *Component(const EntityId _id,
                  const EntityComponentManager *_ecm) const
      {
        ComponentTypeId typeId = typeid(ComponentTypeT).hash_code();
        return static_cast<const ComponentTypeT *>(
            this->ComponentImplementation(_id, typeId, _ecm));
      }

      /// Get a pointer to a component for an entity based on a component type.
      /// \param[in] _id Id of the entity.
      /// \param[in] _ecm Pointer to the entity component manager.
      /// \return Pointer to the component.
      public: template<typename ComponentTypeT>
              ComponentTypeT *Component(const EntityId _id,
                  const EntityComponentManager *_ecm)
      {
        ComponentTypeId typeId = typeid(ComponentTypeT).hash_code();
        return static_cast<ComponentTypeT *>(
            const_cast<void *>(
              this->ComponentImplementation(_id, typeId, _ecm)));
      }

      /// \brief Add an entity to the view.
      /// \param[in] _id Id of the entity to add.
      /// \param[in] _new Whether to add the entity to the list of new entities.
      /// The new here is to indicate whether the entity is new to the entity
      /// component manager. An existing entity can be added when creating a new
      /// view or when rebuilding the view.
      public: void AddEntity(const EntityId _id, const bool _new = false);

      /// \brief Remove an entity from the view.
      /// \param[in] _id Id of the entity to remove.
      /// \param[in] _key Components that should also be removed.
      /// \return True if the entity was erased, false if the entity did not
      /// exist in the view.
      public: bool EraseEntity(const EntityId _id,
                               const ComponentTypeKey &_key);

      /// \brief Add the entity to the list of entities to be removed
      /// \param[in] _id Id of the entity to add.
      /// \return True if the entity was added to the list, false if the entity
      /// did not exist in the view.
      public: bool AddEntityToErased(const EntityId _id);

      /// \brief Add a component to an entity.
      /// \param[in] _id Id of the entity.
      /// \param[in] _compTypeId Component type id.
      /// \param[in] _compId Component id.
      public: void AddComponent(const EntityId _id,
                                const ComponentTypeId _compTypeId,
                                const ComponentId _compId);

      /// \brief Implementation of the Component accessor.
      /// \param[in] _id Id of the Entity.
      /// \param[in] _typeId Type id of the component.
      /// \param[in] _ecm Pointer to the EntityComponentManager.
      /// \return Pointer to the component, or nullptr if not found.
      private: const void *ComponentImplementation(const EntityId _id,
                   ComponentTypeId _typeId,
                   const EntityComponentManager *_ecm) const;

      /// \brief Clear the list of new entities
      public: void ClearNewEntities();

      /// \brief All the entities that belong to this view.
      public: std::set<EntityId> entities;

      /// \brief List of newly created entities
      public: std::set<EntityId> newEntities;

      /// \brief List of entities about to be erased
      public: std::set<EntityId> toEraseEntities;

      /// \brief All of the components for each entity.
      public: std::map<std::pair<EntityId, ComponentTypeId>,
              ComponentId> components;
    };
    /// \endcond

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
      /// \return Id of the new component, and whether the components array
      /// was expanded. kComponentIdInvalid is returned
      /// if the component could not be created.
      public: virtual std::pair<ComponentId, bool> Create(
                  const void *_data) = 0;

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
      public: virtual void *Component(const ComponentId _id) = 0;

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
        // Reserve a chunk of memory for the components. The size here will
        // effect how often Views are rebuilt when
        // EntityComponentManager::CreateComponent() is called.
        //
        // Views would be rebuilt if the components vector capacity is
        // exceeded after an EntityComponentManager::Each call has already
        // been executed.
        //
        // See also this class's Create() function, which expands the value
        // of components vector whenever the capacity is reached.
        this->components.reserve(100);
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
      public: std::pair<ComponentId, bool> Create(
                  const void *_data) override final
      {
        ComponentId result;  // = kComponentIdInvalid;
        bool expanded = false;
        if (this->components.size() == this->components.capacity())
        {
          this->components.reserve(this->components.capacity() + 100);
          expanded = true;
        }

        std::lock_guard<std::mutex> lock(this->mutex);
        result = this->idCounter++;
        this->idMap[result] = this->components.size();
        // Copy the component
        this->components.push_back(std::move(
              ComponentTypeT(*static_cast<const ComponentTypeT *>(_data))));

        return {result, expanded};
      }

      // Documentation inherited.
      public: const void *Component(const ComponentId _id) const override final
      {
        return static_cast<const void*>(
            const_cast<ComponentStorage<ComponentTypeT>*>(
              this)->Component(_id));
      }

      public: void *Component(const ComponentId _id) override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        std::map<ComponentId, int>::const_iterator iter = this->idMap.find(_id);

        if (iter != this->idMap.end())
        {
          return static_cast<void *>(&this->components.at(iter->second));
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

      /// \brief Request an entity deletion. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      public: void RequestEraseEntity(const EntityId _id);

      /// \brief Request to erase all entities. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      public: void RequestEraseEntities();

      /// \brief Get whether an Entity exists.
      /// \param[in] _id Entity id to confirm.
      /// \return True if the Entity exists.
      public: bool HasEntity(const EntityId _id) const;

      /// \brief Get whether a component type has ever been created.
      /// \param[in] _typeId ID of the component type to check.
      /// \return True if the provided _typeId has been created.
      public: bool HasComponentType(const ComponentTypeId _typeId) const;

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

      /// \brief Remove a component from an entity based on a key.
      /// \param[in] _id Id of the entity.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return True if the entity and component existed and the component was
      ///  removed.
      public: bool RemoveComponent(
                  const EntityId _id, const ComponentKey &_key);

      /// \brief Rebuild all the views. This could be an expensive
      /// operation.
      public: void RebuildViews();

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
        const ComponentTypeId typeId = ComponentType<ComponentTypeT>();

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
      public: template<typename ComponentTypeT>
              const ComponentTypeT *Component(const EntityId _id) const
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = ComponentType<ComponentTypeT>();

        return static_cast<const ComponentTypeT *>(
            this->ComponentImplementation(_id, typeId));
      }

      /// \brief Get a mutable component assigned to an entity based on a
      /// component type.
      /// \param[in] _id Id of the entity.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      public: template<typename ComponentTypeT>
              ComponentTypeT *Component(const EntityId _id)
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = ComponentType<ComponentTypeT>();

        return static_cast<ComponentTypeT *>(
            this->ComponentImplementation(_id, typeId));
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
              ComponentTypeT *Component(const ComponentKey &_key)
      {
        return static_cast<ComponentTypeT *>(
            this->ComponentImplementation(_key));
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

      /// \brief Get an entity which matches the value of all the given
      /// components. For example, the following will return the entity which
      /// has an int component equal to 123, and a string component equal to
      /// "name":
      ///
      ///  auto entity = EntityByComponents(123, std::string("name"));
      ///
      /// \detail Component type must have inequality operator.
      ///
      /// \param[in] _desiredComponents All the components which must match.
      /// \return Entity Id or kNullEntity if no entity has the exact
      /// components.
      public: template<typename ...ComponentTypeTs>
              EntityId EntityByComponents(
                   const ComponentTypeTs &..._desiredComponents)
      {
        // Get all entities which have components of the desired types
        const auto &view = this->FindView<ComponentTypeTs...>();

        // Iterate over entities
        EntityId result{kNullEntity};
        for (const EntityId entity : view.entities)
        {
          bool different{false};

          // Iterate over desired components, comparing each of them to the
          // equivalent component in the entity.
          ForEach([&](const auto &_desiredComponent)
          {
            auto entityComponent = this->Component<
                std::remove_cv_t<std::remove_reference_t<
                    decltype(_desiredComponent)>>>(entity);

            if (*entityComponent != _desiredComponent)
            {
              different = true;
            }
          }, _desiredComponents...);

          if (!different)
          {
            result = entity;
            break;
          }
        }

        return result;
      }

      /// why is this required?
      private: template <typename T>
               struct identity
               {
                 typedef T type;
               };

      /// \brief A version of Each() that doesn't use a cache. The cached
      /// version, Each(), is preferred.
      /// Get all entities which contain given component types, as well
      /// as the components.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, Update, or PostUpdate callbacks.
      public: template<typename ...ComponentTypeTs>
              void EachNoCache(typename identity<std::function<
                  bool(const EntityId &_entity,
                       const ComponentTypeTs *...)>>::type _f) const
      {
        for (const Entity &entity : this->Entities())
        {
          auto types = std::set<ComponentTypeId>{
              this->ComponentType<ComponentTypeTs>()...};

          if (this->EntityMatches(entity.Id(), types))
          {
            if (!_f(entity.Id(),
                    this->Component<ComponentTypeTs>(entity.Id())...))
            {
              break;
            }
          }
        }
      }

      /// \brief A version of Each() that doesn't use a cache. The cached
      /// version, Each(), is preferred.
      /// Get all entities which contain given component types, as well
      /// as the mutable components.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired mutable component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, Update, or PostUpdate callbacks.
      public: template<typename ...ComponentTypeTs>
              void EachNoCache(typename identity<std::function<
                  bool(const EntityId &_entity,
                       ComponentTypeTs *...)>>::type _f)
      {
        for (const Entity &entity : this->Entities())
        {
          auto types = std::set<ComponentTypeId>{
              this->ComponentType<ComponentTypeTs>()...};

          if (this->EntityMatches(entity.Id(), types))
          {
            if (!_f(entity.Id(),
                    this->Component<ComponentTypeTs>(entity.Id())...))
            {
              break;
            }
          }
        }
      }

      /// \brief Get all entities which contain given component types, as well
      /// as the components. Note that an entity marked for erasure (but not
      /// processed yet) will be included in the list of entities iterated by
      /// this call.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, Update, or PostUpdate callbacks.
      public: template<typename ...ComponentTypeTs>
              void Each(typename identity<std::function<
                  bool(const EntityId &_entity,
                       const ComponentTypeTs *...)>>::type _f) const
      {
        // Get the view. This will create a new view if one does not already
        // exist.
        View &view = this->FindView<ComponentTypeTs...>();

        // Iterate over the entities in the view, and invoke the callback
        // function.
        for (const EntityId entity : view.entities)
        {
          if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
          {
            break;
          }
        }
      }

      /// \brief Get all entities which contain given component types, as well
      /// as the mutable components. Note that an entity marked for erasure (but
      /// not processed yet) will be included in the list of entities iterated
      /// by this call.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired mutable component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, Update, or PostUpdate callbacks.
      public: template<typename ...ComponentTypeTs>
              void Each(typename identity<std::function<
                  bool(const EntityId &_entity,
                       ComponentTypeTs *...)>>::type _f)
      {
        // Get the view. This will create a new view if one does not already
        // exist.
        View &view = this->FindView<ComponentTypeTs...>();

        // Iterate over the entities in the view, and invoke the callback
        // function.
        for (const EntityId entity : view.entities)
        {
          if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
          {
            break;
          }
        }
      }

      /// \brief Call a function for each parameter in a pack.
      /// \param[in] _f Function to be called.
      /// \param[in] _components Parameters which should be passed to the
      /// function.
      public: template <class Function, class... ComponentTypeTs>
      static void ForEach(Function _f, const ComponentTypeTs &... _components)
      {
        (_f(_components), ...);
      }

      /// \brief Get all newly created entities which contain given component
      /// types, as well as the components. This "newness" is cleared at the end
      /// of a simulation step.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, callback. The result of call after PreUpdate is invalid
      public: template <typename... ComponentTypeTs>
              void EachNew(typename identity<std::function<
                           bool(const EntityId &_entity,
                                ComponentTypeTs *...)>>::type _f)
      {
        // Get the view. This will create a new view if one does not already
        // exist.
        View &view = this->FindView<ComponentTypeTs...>();

        // Iterate over the entities in the view and in the newly created
        // entities list, and invoke the callback
        // function.
        for (const EntityId entity : view.newEntities)
        {
          if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
          {
            break;
          }
        }
      }

      /// \brief Get all newly created entities which contain given component
      /// types, as well as the components. This "newness" is cleared at the end
      /// of a simulation step. This is the const version.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, callback. The result of call after PreUpdate is invalid
      public: template <typename... ComponentTypeTs>
              void EachNew(typename identity<std::function<
                           bool(const EntityId &_entity,
                                const ComponentTypeTs *...)>>::type _f) const
      {
        // Get the view. This will create a new view if one does not already
        // exist.
        View &view = this->FindView<ComponentTypeTs...>();

        // Iterate over the entities in the view and in the newly created
        // entities list, and invoke the callback
        // function.
        for (const EntityId entity : view.newEntities)
        {
          if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
          {
            break;
          }
        }
      }

      /// \brief Get all entities which contain given component types and are
      /// about to be erased, as well as the components.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, callback. The result of call after PreUpdate is invalid
      public: template<typename ...ComponentTypeTs>
              void EachErased(typename identity<std::function<
                  bool(const EntityId &_entity,
                       const ComponentTypeTs *...)>>::type _f) const
      {
        // Get the view. This will create a new view if one does not already
        // exist.
        View &view = this->FindView<ComponentTypeTs...>();

        // Iterate over the entities in the view and in the newly created
        // entities list, and invoke the callback
        // function.
        for (const EntityId entity : view.toEraseEntities)
        {
          if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
          {
            break;
          }
        }
      }

      /// \brief Clear the list of newly added entities so that a call to
      /// EachAdded after this will have no entities to iterate. This function
      /// is protected to facilitate testing.
      protected: void ClearNewlyCreatedEntities();

      /// \brief Process all entity erase requests. This will remove
      /// entities and their components. This function is protected to
      /// facilitate testing.
      protected: void ProcessEraseEntityRequests();

      /// \brief Get whether an Entity exists and is new.
      ///
      /// Entities are considered new in the time between their creation and a
      /// call to ClearNewlyCreatedEntities
      /// \param[in] _id Entity id to check.
      /// \return True if the Entity is new.
      private: bool IsNewEntity(const EntityId _id) const;

      /// \brief Get whether an Entity has been marked to be erased.
      /// \param[in] _id Entity id to check.
      /// \return True if the Entity has been marked to be erased.
      private: bool IsMarkedForErasure(const EntityId _id) const;

      /// \brief Delete an existing Entity.
      /// \param[in] _id Id of the Entity to erase.
      /// \returns True if the Entity existed and was deleted.
      private: bool EraseEntity(const EntityId _id);

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

      /// \brief Get a mutable component based on a component type.
      /// \param[in] _id Id of the entity.
      /// \param[in] _type Id of the component type.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      private: void *ComponentImplementation(const EntityId _id,
                   const ComponentTypeId _type);

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      private: const void *ComponentImplementation(
                   const ComponentKey &_key) const;

      /// \brief Get a mutable component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      private: void *ComponentImplementation(const ComponentKey &_key);

      /// \brief Register a new component type.
      /// \param[in] _typeId Type if of the new component.
      /// \param[in] _type Pointer to the component storage. Ownership is
      /// transfered.
      private: void RegisterComponentType(
                   const ComponentTypeId _typeId,
                   ComponentStorageBase *_type);

      /// \brief Get all the entities.
      /// \return All the entities.
      private: std::vector<Entity> &Entities() const;

      /// \brief End of the AddComponentToView recursion. This function is
      /// called when Rest is empty.
      /// \param[in, out] _view The FirstComponent will be added to the
      /// _view.
      /// \param[in] _id Id of the entity.
      private: template<typename FirstComponent,
                        typename ...RemainingComponents,
                        typename std::enable_if<
                          sizeof...(RemainingComponents) == 0, int>::type = 0>
               void AddComponentsToView(View &_view, const EntityId _id) const
      {
        const ComponentTypeId typeId = ComponentType<FirstComponent>();
        const ComponentId compId = this->EntityComponentIdFromType(_id, typeId);
        if (compId >= 0)
        {
          // Add the component to the view.
          _view.AddComponent(_id, typeId, compId);
        }
        else
        {
          ignerr << "Entity[" << _id << "] has no component of type["
            << typeId << "]. This should never happen.\n";
        }
      }

      /// \brief Recursively add components to a view. This function is
      /// called when Rest is NOT empty.
      /// \param[in, out] _view The FirstComponent will be added to the
      /// _view.
      /// \param[in] _id Id of the entity.
      private: template<typename FirstComponent,
                        typename ...RemainingComponents,
                        typename std::enable_if<
                          sizeof...(RemainingComponents) != 0, int>::type = 0>
              void AddComponentsToView(View &_view, const EntityId _id) const
      {
        const ComponentTypeId typeId = ComponentType<FirstComponent>();
        const ComponentId compId = this->EntityComponentIdFromType(_id, typeId);
        if (compId >= 0)
        {
          // Add the component to the view.
          _view.AddComponent(_id, typeId, compId);
        }
        else
        {
          ignerr << "Entity[" << _id << "] has no component of type["
            << typeId << "]. This should never happen.\n";
        }

        // Add the remaining components to the view.
        this->AddComponentsToView<RemainingComponents...>(_view, _id);
      }

      /// \brief Find a View that matches the set of ComponentTypeIds. If
      /// a match is not found, then a new view is created.
      /// \tparam ComponentTypeTs All the component types that define a view.
      /// \return A reference to the view.
      private: template<typename ...ComponentTypeTs> View &FindView() const
      {
        auto types = std::set<ComponentTypeId>{
            this->ComponentType<ComponentTypeTs>()...};

        std::map<ComponentTypeKey, View>::iterator viewIter;

        // Find the view. If the view doesn't exist, then create a new view.
        if (!this->FindView(types, viewIter))
        {
          View view;
          // Add all the entities that match the component types to the
          // view.
          for (const Entity &entity : this->Entities())
          {
            if (this->EntityMatches(entity.Id(), types))
            {
              view.AddEntity(entity.Id(), this->IsNewEntity(entity.Id()));
              // If there is a request to delete this entity, update the view as
              // well
              if (this->IsMarkedForErasure(entity.Id()))
              {
                view.AddEntityToErased(entity.Id());
              }

              // Store pointers to all the components. This recursively adds
              // all the ComponentTypeTs that belong to the entity to the view.
              this->AddComponentsToView<ComponentTypeTs...>(view, entity.Id());
            }
          }

          // Store the view.
          return this->AddView(types, std::move(view))->second;
        }

        return viewIter->second;
      }

      /// \brief Find a view based on the provided component type ids.
      /// \param[in] _types The component type ids that serve as a key into
      /// a map of views.
      /// \param[out] _iter Iterator to the found element in the view map.
      /// Check the return value to see if this iterator is valid.
      /// \return True if the view was found, false otherwise.
      private: bool FindView(const std::set<ComponentTypeId> &_types,
                   std::map<ComponentTypeKey, View>::iterator &_iter) const;

      /// \brief Add a new view to the set of stored views.
      /// \param[in] _types The set of component type ids that is the key
      /// for the view.
      /// \param[in] _view The view to add.
      /// \return An iterator to the view.
      private: std::map<ComponentTypeKey, View>::iterator AddView(
                   const std::set<ComponentTypeId> &_types, View &&_view) const;

      /// \brief Update views that contain the provided entity.
      /// \param[in] _id Id of the entity.
      private: void UpdateViews(const EntityId _id);

      private: ComponentId EntityComponentIdFromType(
                   const EntityId _id, const ComponentTypeId _type) const;

      /// \brief Private data pointer.
      private: std::unique_ptr<EntityComponentManagerPrivate> dataPtr;

      /// Make simulation runner a friend so that it can trigger entity
      /// erasures. This should be safe since SimulationRunner is internal
      /// to Gazebo.
      friend class SimulationRunner;

      /// Make View a friend so that it can access components.
      // This should be safe since View is internal to Gazebo.
      friend class View;
    };
    }
  }
}
#endif
