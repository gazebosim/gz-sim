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
#ifndef GZ_SIM_ENTITYCOMPONENTMANAGER_HH_
#define GZ_SIM_ENTITYCOMPONENTMANAGER_HH_

#include <gz/msgs/serialized.pb.h>
#include <gz/msgs/serialized_map.pb.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/graph/Graph.hh>
#include "gz/sim/Entity.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"

#include "gz/sim/components/Component.hh"
#include "gz/sim/detail/View.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class GZ_SIM_HIDDEN EntityComponentManagerPrivate;
    class EntityComponentManagerDiff;

    /// \brief Type alias for the graph that holds entities.
    /// Each vertex is an entity, and the direction points from the parent to
    /// its children.
    /// All edges are positive booleans.
    using EntityGraph = math::graph::DirectedGraph<Entity, bool>;

    /** \class EntityComponentManager EntityComponentManager.hh \
     * gz/sim/EntityComponentManager.hh
    **/
    /// \brief The EntityComponentManager constructs, deletes, and returns
    /// components and entities.
    /// A component can be of any class which inherits from
    /// `components::BaseComponent`.
    class GZ_SIM_VISIBLE EntityComponentManager
    {
      /// \brief Constructor
      public: EntityComponentManager();

      /// \brief Destructor
      public: ~EntityComponentManager();

      /// \brief Copies the contents of `_from` into this object.
      /// \note This is a member function instead of a copy constructor so that
      /// it can have additional parameters if the need arises in the future.
      /// Additionally, not every data member is copied making its behavior
      /// different from what would be expected from a copy constructor.
      /// \param[in] _from Object to copy from
      public: void CopyFrom(const EntityComponentManager &_fromEcm);

      /// \brief Creates a new Entity.
      /// \return An id for the Entity, or kNullEntity on failure.
      public: Entity CreateEntity();

      /// \brief Clone an entity and its components. If the entity has any child
      /// entities, they will also be cloned.
      /// When cloning entities, the following rules apply:
      ///   1. The name component of a cloned entity will consist of a unique
      ///      name, since all entities should have a unique name.
      ///   2. Cloned entities that have a canonical link will have their
      ///      canonical link set to the cloned canonical link, not the original
      ///      canonical link.
      ///   3. Child entities that are cloned will have their parent set to the
      ///      cloned parent entity.
      ///   4. Cloned joints with parent/child links will have their parent and
      ///      child links set to the cloned parent/child links.
      ///   5. Aside from the changes listed above, all other cloned components
      ///      remain unchanged.
      /// Currently, cloning detachable joints is not supported.
      /// \param[in] _entity The entity to clone.
      /// \param[in] _parent The parent of the cloned entity. Set this to
      /// kNullEntity if the cloned entity should not have a parent.
      /// \param[in] _name The name that should be given to the cloned entity.
      /// Set this to an empty string if the cloned entity name should be
      /// auto-generated to something unique.
      /// \param[in] _allowRename True if _name can be modified to be a unique
      /// name if it isn't already a unique name. False if _name cannot be
      /// modified to be a unique name. If _allowRename is set to False, and
      /// _name is not unique, _entity will not be cloned. If _name is an
      /// empty string, _allowRename is ignored since the cloned entity will
      /// have an auto-generated unique name.
      /// \return The cloned entity, which will have a unique name. kNullEntity
      /// is returned if cloning failed. Failure could occur if _entity does not
      /// exist, or if a unique name could not be generated for the entity to be
      /// cloned.
      /// \sa Clone
      public: Entity Clone(Entity _entity, Entity _parent,
                  const std::string &_name, bool _allowRename);

      /// \brief Get the number of entities on the server.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Request an entity deletion. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      ///
      /// \details It is recommended that systems don't call this function
      /// directly, and instead use the `sim::SdfEntityCreator` class to
      /// remove entities.
      ///
      /// \param[in] _entity Entity to be removed.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      public: void RequestRemoveEntity(const Entity _entity,
          bool _recursive = true);

      /// \brief Prevent an entity and optionally its children from
      /// being removed.
      ///
      /// This function can be useful when seek operations during log
      /// playback are used in conjunction with spawned entities. For
      /// example, you may want to record a video based on a log file
      /// using a headless simulation instance. This requires a
      /// camera sensor which would be spawned during log playback. If
      /// a seek backward in time is performed during log playback, then the
      /// spawned camera would be removed. Use this function to prevent the
      /// camera from automatic removal.
      ///
      /// \param[in] _entity Entity to be pinned.
      /// \param[in] _recursive Whether to recursively pin all child
      /// entities. True by default.
      public: void PinEntity(const Entity _entity, bool _recursive = true);

      /// \brief Allow an entity, and optionally its children, previously
      /// marked as pinned to be removed.
      /// \param[in] _entity Entity to be unpinned.
      /// \param[in] _recursive Whether to recursively unpin all child
      /// entities. True by default.
      /// \sa void PinEntity(const Entity, bool)
      public: void UnpinEntity(const Entity _entity, bool _recursive = true);

      /// \brief Allow all previously pinned entities to be removed.
      /// \sa void PinEntity(const Entity, bool)
      public: void UnpinAllEntities();

      /// \brief Request to remove all entities. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      public: void RequestRemoveEntities();

      /// \brief Get whether an Entity exists.
      /// \param[in] _entity Entity to confirm.
      /// \return True if the Entity exists.
      public: bool HasEntity(const Entity _entity) const;

      /// \brief Get the first parent of the given entity.
      /// \details Entities are not expected to have multiple parents.
      /// TODO(louise) Either prevent multiple parents or provide full support
      /// for multiple parents.
      /// \param[in] _entity Entity.
      /// \return The parent entity or kNullEntity if there's none.
      public: Entity ParentEntity(const Entity _entity) const;

      /// \brief Set the parent of an entity.
      ///
      /// \details It is recommended that systems don't call this function
      /// directly, and instead use the `sim::SdfEntityCreator` class to
      /// create entities that have the correct parent-child relationship.
      ///
      /// \param[in] _child Entity to set the parent
      /// \param[in] _parent Entity which should be an immediate parent _child
      /// entity.
      /// \return True if successful. Will fail if entities don't exist.
      public: bool SetParentEntity(const Entity _child, const Entity _parent);

      /// \brief Get whether a component type has ever been created.
      /// \param[in] _typeId ID of the component type to check.
      /// \return True if the provided _typeId has been created.
      public: bool HasComponentType(const ComponentTypeId _typeId) const;

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
      /// \return A pointer to the component that was created. nullptr is
      /// returned if the component was not able to be created. If _entity
      /// does not exist, nullptr will be returned.
      public: template<typename ComponentTypeT>
              ComponentTypeT *CreateComponent(
                  const Entity _entity,
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

      /// \brief Get a mutable component assigned to an entity based on a
      /// component type. If the component doesn't exist, create it and
      /// initialize with the given default value.
      /// \param[in] _entity The entity.
      /// \param[in] _default The value that should be used to construct
      /// the component in case the component doesn't exist.
      /// \return The component of the specified type assigned to the specified
      /// entity. If _entity does not exist, nullptr is returned.
      public: template<typename ComponentTypeT>
              ComponentTypeT *ComponentDefault(Entity _entity,
              const typename ComponentTypeT::Type &_default =
                  typename ComponentTypeT::Type());

      /// \brief Get the data from a component.
      /// * If the component type doesn't hold any data, this won't compile.
      /// * If the entity doesn't have that component, it will return nullopt.
      /// * If the entity has the component, return its data.
      /// \param[in] _entity The entity.
      /// \tparam ComponentTypeT Component type
      /// \return The data of the component of the specified type assigned to
      /// specified Entity, or nullptr if the component could not be found.
      public: template<typename ComponentTypeT>
              std::optional<typename ComponentTypeT::Type> ComponentData(
              const Entity _entity) const;

      /// \brief Set the data from a component.
      /// * If the component type doesn't hold any data, this won't compile.
      /// * If the entity doesn't have that component, the component will be
      ///   created.
      /// * If the entity has the component, its data will be updated.
      /// \param[in] _entity The entity.
      /// \param[in] _data New component data
      /// \tparam ComponentTypeT Component type
      /// \return True if data has changed. It will always be true if the data
      /// type doesn't have an equality operator.
      public: template<typename ComponentTypeT>
              bool SetComponentData(const Entity _entity,
              const typename ComponentTypeT::Type &_data);

      /// \brief Get the type IDs of all components attached to an entity.
      /// \param[in] _entity Entity to check.
      /// \return All the component type IDs.
      public: std::unordered_set<ComponentTypeId> ComponentTypes(
          Entity _entity) const;

      /// \brief Get an entity which matches the value of all the given
      /// components. For example, the following will return the entity which
      /// has a name component equal to "name" and has a model component:
      ///
      ///  auto entity = EntityByComponents(components::Name("name"),
      ///    components::Model());
      ///
      /// \details Component type must have inequality operator.
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
      /// \details Component type must have inequality operator.
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
      /// \details Component type must have inequality operator.
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

      /// \brief Helper function for cloning an entity and its children (this
      /// includes cloning components attached to these entities). This method
      /// should never be called directly - it is called internally from the
      /// public Clone method.
      /// \param[in] _entity The entity to clone.
      /// \param[in] _parent The parent of the cloned entity.
      /// \param[in] _name The name that should be given to the cloned entity.
      /// \param[in] _allowRename True if _name can be modified to be a unique
      /// name if it isn't already a unique name. False if _name cannot be
      /// modified to be a unique name.
      /// \return The cloned entity. kNullEntity is returned if cloning failed.
      /// \sa Clone
      private: Entity CloneImpl(Entity _entity, Entity _parent,
                  const std::string &_name, bool _allowRename);

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
      /// \warning Since entity creation occurs during PreUpdate, this function
      /// should not be called in a System's PreUpdate callback (it's okay to
      /// call this function in the Update callback). If you need to call this
      /// function in a system's PostUpdate callback, you should use the const
      /// version of this method.
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
      /// \warning Since entity creation occurs during PreUpdate, this function
      /// should not be called in a System's PreUpdate callback (it's okay to
      /// call this function in the Update or PostUpdate callback).
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
      /// PostUpdate callback.
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
      /// \details The header of the message will not be populated, it is the
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
      /// This includes:
      /// * New entities and all of their components
      /// * Removed entities and all of their components
      /// * Entities which had a component added
      /// * Entities which had a component removed
      /// * Entities which had a component modified
      ///
      /// \details The header of the message will not be populated, it is the
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

      /// \brief Get whether there are periodic component changes. These changes
      /// may happen frequently and are processed periodically.
      /// \return True if there are any components with periodic changes.
      public: bool HasPeriodicComponentChanges() const;

      /// \brief Get the components types that are marked as periodic changes.
      /// \return All the components that at least one entity marked as
      /// periodic changes.
      public: std::unordered_set<ComponentTypeId>
          ComponentTypesWithPeriodicChanges() const;

      /// \brief Get a cache of components with periodic changes.
      /// \param[inout] _changes A list of components with the latest periodic
      /// changes. If a component has a periodic change, it is added to the
      /// hash map. It the component or entity was removed, it is removed from
      /// the hashmap. This way the hashmap stores a list of components and
      /// entities which have had periodic changes in the past and still
      /// exist within the ECM.
      /// \sa EntityComponentManager::PeriodicStateFromCache
      public: void UpdatePeriodicChangeCache(std::unordered_map<ComponentTypeId,
        std::unordered_set<Entity>>&) const;

      /// \brief Set the absolute state of the ECM from a serialized message.
      /// Entities / components that are in the new state but not in the old
      /// one will be created.
      /// Entities / components that are marked as removed will be removed, but
      /// they won't be removed if they're not present in the state.
      /// \details The header of the message will not be handled, it is the
      /// responsibility of the caller to use the timestamp.
      /// \param[in] _stateMsg Message containing state to be set.
      public: void SetState(const msgs::SerializedState &_stateMsg);

      /// \brief Get a message with the serialized state of the given entities
      /// and components.
      /// \details The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      /// \param[out] _state The serialized state message to populate.
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

      /// \brief Populate a message with relevant changes to the state given
      /// a periodic change cache.
      /// \details The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use. Additionally,
      /// changes such as addition or removal will not be populated.
      /// \param[inout] _state The serialized state message to populate.
      /// \param[in] _cache A map of entities and components to serialize.
      /// \sa EntityComponenetManager::UpdatePeriodicChangeCache
      public: void PeriodicStateFromCache(
                  msgs::SerializedStateMap &_state,
                  const std::unordered_map<ComponentTypeId,
                        std::unordered_set<Entity>> &_cache) const;

      /// \brief Get a message with the serialized state of all entities and
      /// components that are changing in the current iteration
      ///
      /// This includes:
      /// * New entities and all of their components
      /// * Removed entities and all of their components
      /// * Entities which had a component added
      /// * Entities which had a component removed
      /// * Entities which had a component modified
      ///
      /// \param[in] _state New serialized state.
      /// \details The header of the message will not be populated, it is the
      /// responsibility of the caller to timestamp it before use.
      public: void ChangedState(msgs::SerializedStateMap &_state) const;

      /// \brief Set the absolute state of the ECM from a serialized message.
      /// Entities / components that are in the new state but not in the old
      /// one will be created.
      /// Entities / components that are marked as removed will be removed, but
      /// they won't be removed if they're not present in the state.
      /// \details The header of the message will not be handled, it is the
      /// responsibility of the caller to use the timestamp.
      /// \param[in] _stateMsg Message containing state to be set.
      public: void SetState(const msgs::SerializedStateMap &_stateMsg);

      /// \brief Set the changed state of a component.
      /// \param[in] _entity The entity.
      /// \param[in] _type Type of the component.
      /// \param[in] _c Changed state value, defaults to one-time-change.
      public: void SetChanged(
          const Entity _entity, const ComponentTypeId _type,
          sim::ComponentState _c = ComponentState::OneTimeChange);

      /// \brief Get a component's state.
      /// \param[in] _entity Entity that contains the component.
      /// \param[in] _typeId Component type ID.
      /// \return Component's current state
      public: sim::ComponentState ComponentState(const Entity _entity,
          const ComponentTypeId _typeId) const;

      /// \brief All future entities will have an id that starts at _offset.
      /// This can be used to avoid entity id collisions, such as during log
      /// playback.
      /// \param[in] _offset Offset value.
      public: void SetEntityCreateOffset(uint64_t _offset);

      /// \brief Given a diff, apply it to this ECM. Note that for removed
      /// entities, this would mark them for removal instead of actually
      /// removing the entities.
      /// \param[in] _other Original EntityComponentManager from which the diff
      /// was computed.
      public: void ResetTo(const EntityComponentManager &_other);

      /// \brief Return true if there are components marked for removal.
      /// \return True if there are components marked for removal.
      public: bool HasRemovedComponents() const;

      /// \brief Get an Entity based on a name component that is associated
      /// with the entity.
      /// \param[in] _name Name associated with the Entity
      /// \return The Entity, if an Entity with the given name exists,
      /// otherwise return std::nullopt.
      public: std::optional<Entity> EntityByName(
                  const std::string &_name) const;

      /// \brief Clear the list of newly added entities so that a call to
      /// EachAdded after this will have no entities to iterate. This function
      /// is protected to facilitate testing.
      protected: void ClearNewlyCreatedEntities();

      /// \brief Clear the list of removed components so that a call to
      /// RemoveComponent doesn't make the list grow indefinitely.
      protected: void ClearRemovedComponents();

      /// \brief Process all entity remove requests. This will remove
      /// entities and their components. This function is protected to
      /// facilitate testing.
      protected: void ProcessRemoveEntityRequests();

      /// \brief Mark all components as not changed.
      protected: void SetAllComponentsUnchanged();

      /// Compute the diff between this EntityComponentManager and _other at the
      /// entity level. This does not compute the diff between components of an
      /// entity.
      ///  * If an entity is in `_other`, but not in `this`, insert the entity
      ///  as an "added" entity.
      ///  * If an entity is in `this`, but not in `other`, insert the entity
      ///  as a "removed" entity.
      ///  \return Data structure containing the added and removed entities
      protected: EntityComponentManagerDiff ComputeEntityDiff(
                     const EntityComponentManager &_other) const;

      /// \brief Given an entity diff, apply it to this ECM. Note that for
      /// removed entities, this would mark them for removal instead of actually
      /// removing the entities.
      /// \param[in] _other Original EntityComponentManager from which the diff
      /// was computed.
      protected: void ApplyEntityDiff(const EntityComponentManager &_other,
                                      const EntityComponentManagerDiff &_diff);

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

      /// \brief Implementation of CreateComponent.
      /// \param[in] _entity The entity that will be associated with
      /// the component.
      /// \param[in] _componentTypeId Id of the component type.
      /// \param[in] _data Data used to construct the component.
      /// \return True if the component's data needs to be set externally; false
      /// otherwise.
      private: bool CreateComponentImplementation(
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

      /// \brief Find a View that matches the set of ComponentTypeIds. If
      /// a match is not found, then a new view is created.
      /// \tparam ComponentTypeTs All the component types that define a view.
      /// \return A pointer to the view.
      private: template<typename ...ComponentTypeTs>
          detail::View *FindView() const;

      /// \brief Find a view based on the provided component type ids.
      /// \param[in] _types The component type ids that serve as a key into
      /// a map of views.
      /// \return A pair containing a the view itself and a mutex that can be
      /// used for locking the view while entities are being added to it.
      /// If a view defined by _types does not exist, the pair will contain
      /// nullptrs.
      private: std::pair<detail::BaseView *, std::mutex *> FindView(
                   const std::vector<ComponentTypeId> &_types) const;

      /// \brief Add a new view to the set of stored views.
      /// \param[in] _types The set of component type ids that act as the key
      /// for the view.
      /// \param[in] _view The view to add.
      /// \return A pointer to the view.
      private: detail::BaseView *AddView(
                   const detail::ComponentTypeKey &_types,
                   std::unique_ptr<detail::BaseView> _view) const;

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

      /// \brief Set whether views should be locked when entities are being
      /// added to them. This can be used to prevent race conditions in
      /// system PostUpdates, since these are run in parallel (entities are
      /// added to views when the view is used, so if two systems try to access
      /// the same view in PostUpdate, we run the risk of multiple threads
      /// reading/writing from the same data).
      /// \param[in] _lock Whether the views should lock while entities are
      /// being added to them (true) or not (false).
      private: void LockAddingEntitiesToViews(bool _lock);

      /// \brief Get whether views should be locked when entities are being
      /// added to them.
      /// \return True if views should be locked during entitiy addition, false
      /// otherwise.
      private: bool LockAddingEntitiesToViews() const;

      // Make runners friends so that they can manage entity creation and
      // removal. This should be safe since runners are internal
      // to Gazebo.
      friend class GuiRunner;
      friend class SimulationRunner;

      // Make SystemManager friend so it has access to removals
      friend class SystemManager;

      // Make network managers friends so they have control over component
      // states. Like the runners, the managers are internal.
      friend class NetworkManagerPrimary;
      friend class NetworkManagerSecondary;
    };
    }
  }
}

#include "gz/sim/detail/EntityComponentManager.hh"

#endif
