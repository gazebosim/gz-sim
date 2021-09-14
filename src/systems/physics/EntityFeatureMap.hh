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

#ifndef IGNITION_GAZEBO_SYSTEMS_PHYSICS_ENTITY_FEATURE_MAP_HH_
#define IGNITION_GAZEBO_SYSTEMS_PHYSICS_ENTITY_FEATURE_MAP_HH_

#include <tuple>
#include <type_traits>
#include <unordered_map>

#include <ignition/physics/Entity.hh>
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/RequestFeatures.hh>

#include "ignition/gazebo/Entity.hh"

namespace ignition::gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems::physics_system
{
  // \brief Helper class that associates Gazebo entities with Physics entities
  // with required and optional features. It can be used to cast a physics
  // entity with the required features to another physics entity with one of
  // the optional features. This class was created to keep all physics entities
  // in one place so that when a gazebo entity is removed, all the mapped
  // physics entitities can be removed at the same time. This ensures that
  // reference counts are properly zeroed out in the underlying physics engines
  // and the memory associated with the physics entities can be freed.
  //
  // DEV WARNING: There is an implicit conversion between physics EntityPtr and
  // std::size_t in ign-physics. This seems also implicitly convert between
  // EntityPtr and gazebo Entity. Therefore, any member function that takes a
  // gazebo Entity can accidentally take an EntityPtr. To prevent this, for
  // every function that takes a gazebo Entity, we should always have an
  // overload that also takes an EntityPtr with required features. We can do
  // this because there's a 1:1 mapping between the two in maps contained in
  // this class.
  //
  // \tparam PhysicsEntityT Type of entity, such as World, Model, or Link
  // \tparam PolicyT Policy of the physics engine (2D, 3D)
  // \tparam RequiredFeatureList Required features of the physics entity
  // \tparam OptionalFeatureLists One or more optional feature lists of the
  // physics entity.
  template <template <typename, typename> class PhysicsEntityT,
            typename PolicyT, typename RequiredFeatureList,
            typename... OptionalFeatureLists>
  class EntityFeatureMap
  {
    /// \brief Template that's preset with the class's Policy type
    /// \tparam T A FeatureList that is used in creating an EntityPtr
    public: template <typename T>
    using PhysicsEntityPtr = physics::EntityPtr<PhysicsEntityT<PolicyT, T>>;

    /// \brief A EntityPtr with made from RequiredFeatureList
    public: using RequiredEntityPtr = PhysicsEntityPtr<RequiredFeatureList>;

    /// \brief Checks whether type T is in the OptionalFeatureLists
    /// \tparam T A FeatureList to search for in OptionalFeatureLists
    public: template <typename T>
            using HasFeatureList =
                std::disjunction<std::is_same<T, OptionalFeatureLists>...>;

    /// \brief A tuple where the first element is an EntityPtr with made from
    /// RequiredFeatureList and the remaining elements are made from
    /// types in OptionalFeatureLists
    private: using ValueType =
                 std::tuple<RequiredEntityPtr,
                            PhysicsEntityPtr<OptionalFeatureLists>...>;

    /// \brief Helper function to cast from an entity type with minimum features
    /// to an entity with a different set of features. When the entity is cast
    /// successfully, it is added to an internal cache so that subsequent casts
    /// will use the entity from the cache.
    /// \tparam ToFeatureList The list of features of the resulting entity.
    /// \param[in] _entity Gazebo entity.
    /// \return Physics entity with features in ToFeatureList. nullptr if the
    /// entity can't be found or the physics engine doesn't support the
    /// requested feature.
    public: template <typename ToFeatureList>
            PhysicsEntityPtr<ToFeatureList>
            EntityCast(gazebo::Entity _entity) const
    {
      // Using constexpr to limit compiler error message to the static_assert
      // cppcheck-suppress syntaxError
      if constexpr (!HasFeatureList<ToFeatureList>::value) // NOLINT
      {
        static_assert(HasFeatureList<ToFeatureList>::value,
            "Trying to cast to a FeatureList not included in the "
            "optional FeatureLists of this map.");
        return nullptr;
      }
      else
      {
        using ToEntityPtr = PhysicsEntityPtr<ToFeatureList>;
        // Has already been cast
        auto castIt = this->castCache.find(_entity);
        if (castIt != this->castCache.end())
        {
          auto castEntity = std::get<ToEntityPtr>(castIt->second);
          if (nullptr != castEntity)
          {
            return castEntity;
          }
        }

        auto reqEntity = this->Get(_entity);
        if (nullptr == reqEntity)
        {
          return nullptr;
        }

        // Cast
        auto castEntity = physics::RequestFeatures<ToFeatureList>::From(
            this->Get(_entity));

        if (castEntity)
        {
          std::get<ToEntityPtr>(this->castCache[_entity]) = castEntity;
        }

        return castEntity;
      }
    }

    /// \brief Helper function to cast from an entity type with minimum features
    /// to an entity with a different set of features. This overload takes a
    /// physics entity as input
    /// \tparam ToFeatureList The list of features of the resulting entity.
    /// \param[in] _physicsEntity Physics entity with required features.
    /// \return Physics entity with features in ToFeatureList. nullptr if the
    /// entity can't be found or the physics engine doesn't support the
    /// requested feature.
    public: template <typename ToFeatureList>
            PhysicsEntityPtr<ToFeatureList>
            EntityCast(const RequiredEntityPtr &_physicsEntity) const
    {
      auto gzEntity = this->Get(_physicsEntity);
      if (kNullEntity == gzEntity)
      {
        return nullptr;
      }
      return this->EntityCast<ToFeatureList>(gzEntity);
    }

    /// \brief Get the physics entity with required features that corresponds to
    /// the input Gazebo entity
    /// \param[in] _entity Gazebo entity.
    /// \return If found, returns the corresponding physics entity. Otherwise,
    /// nullptr
    public: RequiredEntityPtr Get(const Entity &_entity) const
    {
      auto it = this->entityMap.find(_entity);
      if (it != this->entityMap.end())
      {
        return it->second;
      }
      return nullptr;
    }

    /// \brief Get Gazebo entity that corresponds to the physics entity with
    /// required features
    /// \param[in] _physEntity Physics entity with required features
    /// \return If found, returns the corresponding Gazebo entity. Otherwise,
    /// kNullEntity
    public: Entity Get(const RequiredEntityPtr &_physEntity) const
    {
      auto it = this->reverseMap.find(_physEntity);
      if (it != this->reverseMap.end())
      {
        return it->second;
      }
      return kNullEntity;
    }

    /// \brief Get the physics entity with required features that has a
    /// particular ID
    /// \param[in] _id The ID of the desired physics entity
    /// \return If found, returns the corresponding physics entity. Otherwise,
    /// nullptr
    public: RequiredEntityPtr GetPhysicsEntityPtr(std::size_t _id) const
    {
      auto it = this->physEntityById.find(_id);
      if (it != this->physEntityById.end())
      {
        return it->second;
      }
      return nullptr;
    }

    /// \brief Check whether there is a physics entity associated with the given
    /// Gazebo entity
    /// \param[in] _entity Gazebo entity.
    /// \return True if the there is a physics entity associated with the given
    /// Gazebo entity
    public: bool HasEntity(const Entity &_entity) const
    {
      return this->entityMap.find(_entity) != this->entityMap.end();
    }

    /// \brief Check whether there is a gazebo entity associated with the given
    /// physics entity
    /// \param[in] _physicsEntity physics entity with required features.
    /// \return True if the there is a gazebo entity associated with the given
    /// physics entity
    public: bool HasEntity(const RequiredEntityPtr &_physicsEntity) const
    {
      return this->reverseMap.find(_physicsEntity) != this->reverseMap.end();
    }

    /// \brief Add a mapping between gazebo and physics entities
    /// \param[in] _entity Gazebo entity.
    /// \param[in] _physicsEntity Physics entity with required feature
    public: void AddEntity(const Entity &_entity,
                           const RequiredEntityPtr &_physicsEntity)
    {
      this->entityMap[_entity] = _physicsEntity;
      this->reverseMap[_physicsEntity] = _entity;
      this->physEntityById[_physicsEntity->EntityID()] = _physicsEntity;
    }

    /// \brief Remove entity from all associated maps
    /// \param[in] _entity Gazebo entity.
    /// \return True if the entity was found and removed.
    public: bool Remove(Entity _entity)
    {
      auto it = this->entityMap.find(_entity);
      if (it != this->entityMap.end())
      {
        this->reverseMap.erase(it->second);
        this->physEntityById.erase(it->second->EntityID());
        this->castCache.erase(_entity);
        this->entityMap.erase(it);
        return true;
      }
      return false;
    }

    /// \brief Remove physics entity from all associated maps
    /// \param[in] _physicsEntity Physics entity.
    /// \return True if the entity was found and removed.
    public: bool Remove(const RequiredEntityPtr &_physicsEntity)
    {
      auto it = this->reverseMap.find(_physicsEntity);
      if (it != this->reverseMap.end())
      {
        this->entityMap.erase(it->second);
        this->physEntityById.erase(it->first->EntityID());
        this->castCache.erase(it->second);
        this->reverseMap.erase(it);
        return true;
      }
      return false;
    }

    /// \brief Get the map from Gazebo entity to physics entities with required
    /// features
    /// \return Immumtable entity map
    public: const std::unordered_map<Entity, RequiredEntityPtr> &Map() const
    {
      return this->entityMap;
    }

    /// \brief Get the total number of entries in the maps. Only used for
    /// testing.
    /// \return Number of entries in all the maps.
    public: std::size_t TotalMapEntryCount() const
    {
      return this->entityMap.size() + this->reverseMap.size() +
             this->castCache.size() + this->physEntityById.size();
    }

    /// \brief Map from Gazebo entity to physics entities with required features
    private: std::unordered_map<Entity, RequiredEntityPtr> entityMap;

    /// \brief Reverse map of entityMap
    private: std::unordered_map<RequiredEntityPtr, Entity> reverseMap;

    /// \brief Map of physics entity IDs to the corresponding physics entity
    /// with required features
    private: std::unordered_map<std::size_t, RequiredEntityPtr> physEntityById;

    /// \brief Cache map from Gazebo entity to physics entities with optional
    /// features
    private: mutable std::unordered_map<Entity, ValueType> castCache;
  };

  /// \brief Convenience template that presets EntityFeatureMap with
  /// FeaturePolicy3d
  template <template <typename, typename> class PhysicsEntityT,
            typename RequiredFeatureList, typename... OptionalFeatureLists>
  using EntityFeatureMap3d =
      EntityFeatureMap<PhysicsEntityT, physics::FeaturePolicy3d,
                       RequiredFeatureList, OptionalFeatureLists...>;
}
}
}
#endif
