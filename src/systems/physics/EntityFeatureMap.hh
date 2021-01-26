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
  template <template <typename, typename> class PhysicsEntityT,
            typename PolicyT, typename RequiredFeatureList,
            typename... OptionalFeatureLists>
  class EntityFeatureMap
  {
    public: template <typename T>
    using PhysicsEntityPtr = physics::EntityPtr<PhysicsEntityT<PolicyT, T>>;

    public: using RequiredEntityPtr = PhysicsEntityPtr<RequiredFeatureList>;
    public: template <typename T>
            using HasFeatureList =
                std::disjunction<std::is_same<T, OptionalFeatureLists>...>;

    private: using ValueType =
                std::tuple<RequiredEntityPtr,
                           PhysicsEntityPtr<OptionalFeatureLists>...>;

    /// \brief Helper function to cast from an entity type with minimum features
    /// to an entity with a different set of features. When the entity is cast
    /// successfully, it is added to an internal cache so that subsequent casts
    /// will use the entity from the cache.
    /// \tparam ToFeatureList The list of features of the resulting entity.
    /// \param[in] _entity Entity ID.
    /// \return Physics entity with features in ToFeatureList. nullptr if the
    /// entity can't be found or the physics engine doesn't support the
    /// requested feature.
    public: template <typename ToFeatureList>
            PhysicsEntityPtr<ToFeatureList> EntityCast(Entity _entity)
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

    public: RequiredEntityPtr Get(const Entity &_entity)
    {
      auto it = this->entityMap.find(_entity);
      if (it != this->entityMap.end())
      {
        return it->second;
      }
      return nullptr;
    }
    public: Entity Get(const RequiredEntityPtr &_physEntity)
    {
      auto it = this->reverseMap.find(_physEntity);
      if (it != this->reverseMap.end())
      {
        return it->second;
      }
      return kNullEntity;
    }

    public: bool HasEntity(const Entity &_entity)
    {
      return this->entityMap.find(_entity) != this->entityMap.end();
    }

    public: void AddEntity(const Entity &_entity,
                           RequiredEntityPtr _physicsEntity)
    {
      this->entityMap[_entity] = _physicsEntity;
      this->reverseMap[_physicsEntity] = _entity;
    }

    public: bool Remove(Entity _entity)
    {
      auto it = this->entityMap.find(_entity);
      if (it != this->entityMap.end())
      {
        this->reverseMap.erase(it->second);
        this->entityMap.erase(it);
        this->castCache.erase(_entity) == 1u;
        return true;
      }
      return false;
    }

    public: std::unordered_map<Entity, RequiredEntityPtr> &Map()
    {
      return this->entityMap;
    }

    private: std::unordered_map<Entity, RequiredEntityPtr> entityMap;
    private: std::unordered_map<RequiredEntityPtr, Entity> reverseMap;
    private: std::unordered_map<Entity, ValueType> castCache;
  };

  template <template <typename, typename> class PhysicsEntityT,
            typename RequiredFeatureList, typename... OptionalFeatureLists>
  using EntityFeatureMap3d =
      EntityFeatureMap<PhysicsEntityT, physics::FeaturePolicy3d,
                       RequiredFeatureList, OptionalFeatureLists...>;
}
}
}
#endif
