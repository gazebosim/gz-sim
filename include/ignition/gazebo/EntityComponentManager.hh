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

#include <ignition/msgs/serialized.pb.h>
#include <ignition/msgs/serialized_map.pb.h>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/math/graph/Graph.hh>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/detail/View.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN EntityComponentManagerPrivate;

    /// \brief Type alias for the graph that holds entities.
    /// Each vertex is an entity, and the direction points from the parent to
    /// its children.
    /// All edges are positive booleans.
    using EntityGraph = math::graph::DirectedGraph<Entity, bool>;

    /** \class EntityComponentManager EntityComponentManager.hh \
     * ignition/gazebo/EntityComponentManager.hh
    **/
    /// \brief The EntityComponentManager constructs, deletes, and returns
    /// components and entities.
    /// A component can be of any class which inherits from
    /// `components::BaseComponent`.
    class IGNITION_GAZEBO_VISIBLE EntityComponentManager
    {
      /// \brief Constructor
      public: EntityComponentManager();

      /// \brief Destructor
      public: ~EntityComponentManager();

      /// \brief Creates a new Entity.
      /// \return An id for the Entity, or kNullEntity on failure.
      public: Entity CreateEntity();

      /// \brief Get the number of entities on the server.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Request an entity deletion. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      ///
      /// \detail It is recommended that systems don't call this function
      /// directly, and instead use the `gazebo::SdfEntityCreator` class to
      /// remove entities.
      ///
      /// \param[in] _entity Entity to be removed.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      public: void RequestRemoveEntity(const Entity _entity,
          bool _recursive = true);

      /// \brief Request to remove all entities. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      public: void RequestRemoveEntities();

      /// \brief Get whether an Entity exists.
      /// \param[in] _entity Entity to confirm.
      /// \return True if the Entity exists.
      public: bool HasEntity(const Entity _entity) const;

      /// \brief Get the first parent of the given entity.
      /// \detail Entities are not expected to have multiple parents.
      /// TODO(louise) Either prevent multiple parents or provide full support
      /// for multiple parents.
      /// \param[in] _entity Entity.
      /// \return The parent entity or kNullEntity if there's none.
      public: Entity ParentEntity(const Entity _entity) const;

      /// \brief Set the parent of an entity.
      ///
      /// \detail It is recommended that systems don't call this function
      /// directly, and instead use the `gazebo::SdfEntityCreator` class to
      /// create entities that have the correct parent-child relationship.
      ///
      /// \param[in] _entity Entity or kNullEntity to remove current parent.
      /// \return True if successful. Will fail if entities don't exist.
      public: bool SetParentEntity(const Entity _child, const Entity _parent);

      /// \brief Get whether a component type has ever been created.
      /// \param[in] _typeId ID of the component type to check.
      /// \return True if the provided _typeId has been created.
      public: bool HasComponentType(const ComponentTypeId _typeId) const;

      /// \brief Check whether an entity has a specific component.
      /// \param[in] _entity The entity to check.
      /// \param[in] _key The component to check.
      /// \return True if the component key belongs to the entity.
      public: bool EntityHasComponent(const Entity _entity,
                  const ComponentKey &_key) const;

      /// \brief Check whether an entity has a specific component type.
      /// \param[in] _entity The entity to check.
      /// \param[in] _typeId Component type id to check.
      /// \return True if the entity exists and has at least one component
      /// with the provided type.
      public: bool EntityHasComponentType(const Entity _entity,
                  const ComponentTypeId &_typeId) const;

      /// \brief Get whether an entity has all the given component types.
      /// \param[in] _entity The entity to check.
      /// \param[in] _types Component types to check that the Entity has.
      /// \return True if the given entity has all the given types.
      public: bool EntityMatches(Entity _entity,
        const std::set<ComponentTypeId> &_types) const;

      /// \brief Remove a component from an entity based on a key.
      /// \param[in] _entity The entity.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return True if the entity and component existed and the component was
      ///  removed.
      public: bool RemoveComponent(
                  const Entity _entity, const ComponentKey &_key);

      /// \brief Remove a component from an entity based on a type id.
      /// \param[in] _entity The entity.
      /// \param[in] _typeId Component's type Id.
      /// \return True if the entity and component existed and the component was
      ///  removed.
      public: bool RemoveComponent(
                  const Entity _entity, const ComponentTypeId &_typeId);

      /// \brief Remove a component from an entity based on a type.
      /// \param[in] _entity The entity.
      /// \tparam Component type.
      /// \return True if the entity and component existed and the component was
      ///  removed.
      public: template<typename ComponentTypeT>
              bool RemoveComponent(Entity _entity);

      /// \brief Rebuild all the views. This could be an expensive
      /// operation.
      public: void RebuildViews();

      /// \brief Create a component of a particular type. This will copy the
      /// _data parameter.
      /// \param[in] _entity The entity that will be associated with
      /// the component.
      /// \param[in] _data Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      public: template<typename ComponentTypeT>
              ComponentKey CreateComponent(const Entity _entity,
                  const ComponentTypeT &_data);

      /// \brief Get a component assigned to an entity based on a
      /// component type.
      /// \param[in] _entity The entity.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *Component(const Entity _entity) const;

      /// \brief Get a mutable component assigned to an entity based on a
      /// component type.
      /// \param[in] _entity The entity.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      public: template<typename ComponentTypeT>
              ComponentTypeT *Component(const Entity _entity);

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *Component(const ComponentKey &_key) const;

      /// \brief Get a mutable component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentTypeT>
              ComponentTypeT *Component(const ComponentKey &_key);

      /// \brief Get the type IDs of all components attached to an entity.
      /// \param[in] _entity Entity to check.
      /// \return All the component type IDs.
      public: std::unordered_set<ComponentTypeId> ComponentTypes(
          Entity _entity) const;

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      public: template<typename ComponentTypeT>
              const ComponentTypeT *First() const;

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      public: template<typename ComponentTypeT>
              ComponentTypeT *First();

      /// \brief Get an entity which matches the value of all the given
      /// components. For example, the following will return the entity which
      /// has an name component equal to "name" and has a model component:
      ///
      ///  auto entity = EntityByComponents(components::Name("name"),
      ///    components::Model());
      ///
      /// \detail Component type must have inequality operator.
      ///
      /// \param[in] _desiredComponents All the components which must match.
      /// \return Entity or kNullEntity if no entity has the exact components.
      public: template<typename ...ComponentTypeTs>
              Entity EntityByComponents(
                   const ComponentTypeTs &..._desiredComponents) const;

      /// \brief Get all entities which match the value of all the given
      /// components. For example, the following will return the entities which
      /// have a name component equal to "camera" and a sensor component:
      ///
      ///  auto entities = EntitiesByComponents(components::Name("camera"),
      ///    components::Sensor());
      ///
      /// \detail Component type must have inequality operator.
      ///
      /// \param[in] _desiredComponents All the components which must match.
      /// \return All matching entities, or an empty vector if no child entity
      /// has the exact components.
      public: template<typename ...ComponentTypeTs>
              std::vector<Entity> EntitiesByComponents(
                   const ComponentTypeTs &..._desiredComponents) const;

      /// \brief Get all entities which match the value of all the given
      /// components and are immediate children of a given parent entity.
      /// For example, the following will return a child of entity `parent`
      /// which has an int component equal to 123, and a string component
      /// equal to "name":
      ///
      ///  auto entity = ChildrenByComponents(parent, 123, std::string("name"));
      ///
      /// \detail Component type must have inequality operator.
      ///
      /// \param[in] _parent Entity which should be an immediate parent of the
      /// returned entity.
      /// \param[in] _desiredComponents All the components which must match.
      /// \return All matching entities, or an empty vector if no child entity
      /// has the exact components.
      public: template<typename ...ComponentTypeTs>
              std::vector<Entity> ChildrenByComponents(Entity _parent,
                   const ComponentTypeTs &..._desiredComponents) const;

      /// why is this required?
      private: template <typename T>
               struct identity;  // NOLINT

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
                  bool(const Entity &_entity,
                       const ComponentTypeTs *...)>>::type _f) const;

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
                  bool(const Entity &_entity,
                       ComponentTypeTs *...)>>::type _f);

      /// \brief Get all entities which contain given component types, as well
      /// as the components. Note that an entity marked for removal (but not
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
                  bool(const Entity &_entity,
                       const ComponentTypeTs *...)>>::type _f) const;

      /// \brief Get all entities which contain given component types, as well
      /// as the mutable components. Note that an entity marked for removal (but
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
                  bool(const Entity &_entity,
                       ComponentTypeTs *...)>>::type _f);

      /// \brief Call a function for each parameter in a pack.
      /// \param[in] _f Function to be called.
      /// \param[in] _components Parameters which should be passed to the
      /// function.
      public: template <class Function, class... ComponentTypeTs>
      static void ForEach(Function _f, const ComponentTypeTs &... _components);

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
                           bool(const Entity &_entity,
                                ComponentTypeTs *...)>>::type _f);

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
                           bool(const Entity &_entity,
                                const ComponentTypeTs *...)>>::type _f) const;

      /// \brief Get all entities which contain given component types and are
      /// about to be removed, as well as the components.
      /// \param[in] _f Callback function to be called for each matching entity.
      /// The function parameter are all the desired component types, in the
      /// order they're listed on the template. The callback function can
      /// return false to stop subsequent calls to the callback, otherwise
      /// a true value should be returned.
      /// \tparam ComponentTypeTs All the desired component types.
      /// \warning This function should not be called outside of System's
      /// PreUpdate, callback. The result of call after PreUpdate is invalid
      public: template<typename ...ComponentTypeTs>
              void EachRemoved(typename identity<std::function<
                  bool(const Entity &_entity,
                       const ComponentTypeTs *...)>>::type _f) const;

      /// \brief Get a graph with all the entities. Entities are vertices and
      /// edges point from parent to children.
      /// \return Entity graph.
      public: const EntityGraph &Entities() const;

      /// \brief Get all entities which are descendants of a given entity,
      /// including the entity itself.
      /// \param[in] _entity Entity whose descendants we want.
      /// \return All child entities recursively, including _entity. It will be
      /// empty if the entity doesn't exist.
      public: std::unordered_set<Entity> Descendants(Entity _entity) const;

      /// \brief Get a message with the serialized state of the given entities
      /// and components.
      /// \detail The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      /// \param[in] _entities Entities to be serialized. Leave empty to get
      /// all entities.
      /// \param[in] _types Type ID of components to be serialized. Leave empty
      /// to get all components.
      public: msgs::SerializedState State(
          const std::unordered_set<Entity> &_entities = {},
          const std::unordered_set<ComponentTypeId> &_types = {}) const;

      /// \brief Get a message with the serialized state of all entities and
      /// components that are changing in the current iteration
      ///
      /// Currently supported:
      /// * New entities and all of their components
      /// * Removed entities and all of their components
      ///
      /// Future work:
      /// * Entities which had a component added
      /// * Entities which had a component removed
      /// * Entities which had a component modified
      ///
      /// \detail The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      public: msgs::SerializedState ChangedState() const;

      /// \brief Get whether there are new entities.
      /// \return True if there are new entities.
      public: bool HasNewEntities() const;

      /// \brief Get whether there are any entities marked to be removed.
      /// \return True if there are entities marked to be removed.
      public: bool HasEntitiesMarkedForRemoval() const;

      /// \brief Get whether there are one-time component changes. These changes
      /// do not happen frequently and should be processed immediately.
      /// \return True if there are any components with one-time changes.
      public: bool HasOneTimeComponentChanges() const;

      /// \brief Set the absolute state of the ECM from a serialized message.
      /// Entities / components that are in the new state but not in the old
      /// one will be created.
      /// Entities / components that are marked as removed will be removed, but
      /// they won't be removed if they're not present in the state.
      /// \detail The header of the message will not be handled, it is the
      /// responsibility of the caller to use the timestamp.
      /// \param[in] _stateMsg Message containing state to be set.
      public: void SetState(const msgs::SerializedState &_stateMsg);

      /// \brief Get a message with the serialized state of the given entities
      /// and components.
      /// \detail The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      /// \param[in] _entities Entities to be serialized. Leave empty to get
      /// all entities.
      /// \param[in] _types Type ID of components to be serialized. Leave empty
      /// to get all components.
      /// \param[in] _full True to get all the entities and components.
      /// False will get only components and entities that have changed.
      public: void State(
                  msgs::SerializedStateMap &_state,
                  const std::unordered_set<Entity> &_entities = {},
                  const std::unordered_set<ComponentTypeId> &_types = {},
                  bool _full = false) const;

      /// \brief Get a message with the serialized state of all entities and
      /// components that are changing in the current iteration
      ///
      /// Currently supported:
      /// * New entities and all of their components
      /// * Removed entities and all of their components
      ///
      /// Future work:
      /// * Entities which had a component added
      /// * Entities which had a component removed
      /// * Entities which had a component modified
      ///
      /// \param[in] _state New serialized state.
      /// \detail The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      public: void ChangedState(msgs::SerializedStateMap &_state) const;

      /// \brief Set the absolute state of the ECM from a serialized message.
      /// Entities / components that are in the new state but not in the old
      /// one will be created.
      /// Entities / components that are marked as removed will be removed, but
      /// they won't be removed if they're not present in the state.
      /// \detail The header of the message will not be handled, it is the
      /// responsibility of the caller to use the timestamp.
      /// \param[in] _stateMsg Message containing state to be set.
      public: void SetState(const msgs::SerializedStateMap &_stateMsg);

      /// \brief Set the changed state of a component.
      /// \param[in] _entity The entity.
      /// \param[in] _type Type of the component.
      /// \param[in] _c Changed state value, defaults to one-time-change.
      public: void SetChanged(
          const Entity _entity, const ComponentTypeId _type,
          gazebo::ComponentState _c = ComponentState::OneTimeChange);

      /// \brief Get a component's state.
      /// \param[in] _entity Entity that contains the component.
      /// \param[in] _typeId Component type ID.
      /// \return Component's current state
      public: gazebo::ComponentState ComponentState(const Entity _entity,
          const ComponentTypeId _typeId) const;

      /// \brief Clear the list of newly added entities so that a call to
      /// EachAdded after this will have no entities to iterate. This function
      /// is protected to facilitate testing.
      protected: void ClearNewlyCreatedEntities();

      /// \brief Process all entity remove requests. This will remove
      /// entities and their components. This function is protected to
      /// facilitate testing.
      protected: void ProcessRemoveEntityRequests();

      /// \brief Mark all components as not changed.
      protected: void SetAllComponentsUnchanged();

      /// \brief Get whether an Entity exists and is new.
      ///
      /// Entities are considered new in the time between their creation and a
      /// call to ClearNewlyCreatedEntities
      /// \param[in] _entity Entity id to check.
      /// \return True if the Entity is new.
      private: bool IsNewEntity(const Entity _entity) const;

      /// \brief Get whether an Entity has been marked to be removed.
      /// \param[in] _entity Entity id to check.
      /// \return True if the Entity has been marked to be removed.
      private: bool IsMarkedForRemoval(const Entity _entity) const;

      /// \brief Delete an existing Entity.
      /// \param[in] _entity The entity to remove.
      /// \returns True if the Entity existed and was deleted.
      private: bool RemoveEntity(const Entity _entity);

      /// \brief The first component instance of the specified type.
      /// \return First component instance of the specified type, or nullptr
      /// if the type does not exist.
      private: components::BaseComponent *First(
                   const ComponentTypeId _componentTypeId);

      /// \brief Implementation of CreateComponent.
      /// \param[in] _entity The entity that will be associated with
      /// the component.
      /// \param[in] _componentTypeId Id of the component type.
      /// \param[in] _data Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      private: ComponentKey CreateComponentImplementation(
                   const Entity _entity,
                   const ComponentTypeId _componentTypeId,
                   const components::BaseComponent *_data);

      /// \brief Get a component based on a component type.
      /// \param[in] _entity The entity.
      /// \param[in] _type Id of the component type.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      private: const components::BaseComponent *ComponentImplementation(
                   const Entity _entity,
                   const ComponentTypeId _type) const;

      /// \brief Get a mutable component based on a component type.
      /// \param[in] _entity The entity.
      /// \param[in] _type Id of the component type.
      /// \return The component of the specified type assigned to specified
      /// Entity, or nullptr if the component could not be found.
      private: components::BaseComponent *ComponentImplementation(
                   const Entity _entity,
                   const ComponentTypeId _type);

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      private: const components::BaseComponent *ComponentImplementation(
                   const ComponentKey &_key) const;

      /// \brief Get a mutable component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      private: components::BaseComponent *ComponentImplementation(
                   const ComponentKey &_key);

      /// \brief End of the AddComponentToView recursion. This function is
      /// called when Rest is empty.
      /// \param[in, out] _view The FirstComponent will be added to the
      /// _view.
      /// \param[in] _entity The entity.
      private: template<typename FirstComponent,
                        typename ...RemainingComponents,
                        typename std::enable_if<
                          sizeof...(RemainingComponents) == 0, int>::type = 0>
          void AddComponentsToView(detail::View &_view,
               const Entity _entity) const;

      /// \brief Recursively add components to a view. This function is
      /// called when Rest is NOT empty.
      /// \param[in, out] _view The FirstComponent will be added to the
      /// _view.
      /// \param[in] _entity The entity.
      private: template<typename FirstComponent,
                        typename ...RemainingComponents,
                        typename std::enable_if<
                          sizeof...(RemainingComponents) != 0, int>::type = 0>
          void AddComponentsToView(detail::View &_view,
              const Entity _entity) const;

      /// \brief Find a View that matches the set of ComponentTypeIds. If
      /// a match is not found, then a new view is created.
      /// \tparam ComponentTypeTs All the component types that define a view.
      /// \return A reference to the view.
      private: template<typename ...ComponentTypeTs>
          detail::View &FindView() const;

      /// \brief Find a view based on the provided component type ids.
      /// \param[in] _types The component type ids that serve as a key into
      /// a map of views.
      /// \param[out] _iter Iterator to the found element in the view map.
      /// Check the return value to see if this iterator is valid.
      /// \return True if the view was found, false otherwise.
      private: bool FindView(const std::set<ComponentTypeId> &_types,
          std::map<detail::ComponentTypeKey,
          detail::View>::iterator &_iter) const;

      /// \brief Add a new view to the set of stored views.
      /// \param[in] _types The set of component type ids that is the key
      /// for the view.
      /// \param[in] _view The view to add.
      /// \return An iterator to the view.
      private: std::map<detail::ComponentTypeKey, detail::View>::iterator
          AddView(const std::set<ComponentTypeId> &_types,
              detail::View &&_view) const;

      /// \brief Update views that contain the provided entity.
      /// \param[in] _entity The entity.
      private: void UpdateViews(const Entity _entity);

      /// \brief Get a component ID based on an entity and the component's type.
      /// \param[in] _entity The entity.
      /// \param[in] _type Component type ID.
      private: ComponentId EntityComponentIdFromType(
          const Entity _entity, const ComponentTypeId _type) const;

      /// \brief Add an entity and its components to a serialized state message.
      /// \param[out] _msg The state message.
      /// \param[in] _entity The entity to be added.
      /// \param[in] _types Component types to be added. Leave empty for all
      /// components.
      private: void AddEntityToMessage(msgs::SerializedState &_msg,
          Entity _entity,
          const std::unordered_set<ComponentTypeId> &_types = {}) const;

      /// \brief Private data pointer.
      private: std::unique_ptr<EntityComponentManagerPrivate> dataPtr;

      /// \brief Add an entity and its components to a serialized state message.
      /// \param[out] _msg The state message.
      /// \param[in] _entity The entity to be added.
      /// \param[in] _types Component types to be added. Leave empty for all
      /// components.
      /// \param[in] _full True to get all the entities and components.
      /// False will get only components and entities that have changed.
      /// \note This function will mark `Changed` components as not changed.
      /// See the todo in the implementation.
      private: void AddEntityToMessage(msgs::SerializedStateMap &_msg,
          Entity _entity,
          const std::unordered_set<ComponentTypeId> &_types = {},
          bool _full = false) const;

      // Make runners friends so that they can manage entity creation and
      // removal. This should be safe since runners are internal
      // to Gazebo.
      friend class GuiRunner;
      friend class SimulationRunner;

      // Make network managers friends so they have control over component
      // states. Like the runners, the managers are internal.
      friend class NetworkManagerPrimary;
      friend class NetworkManagerSecondary;

      // Make View a friend so that it can access components.
      // This should be safe since View is internal to Gazebo.
      friend class detail::View;
    };
    }
  }
}

#include "ignition/gazebo/detail/EntityComponentManager.hh"

#endif
