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
#ifndef IGNITION_GAZEBO_SYSTEMS_PHYSICS_HH_
#define IGNITION_GAZEBO_SYSTEMS_PHYSICS_HH_

#include <memory>
#include <unordered_map>
#include <utility>
#include <ignition/physics/RequestFeatures.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class PhysicsPrivate;

  /// \class Physics Physics.hh ignition/gazebo/systems/Physics.hh
  /// \brief Base class for a System.
  class IGNITION_GAZEBO_VISIBLE Physics:
    public System,
    public ISystemConfigure,
    public ISystemUpdate
  {
    /// \brief Constructor
    public: explicit Physics();

    /// \brief Destructor
    public: ~Physics() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<PhysicsPrivate> dataPtr;
  };

  /// \brief Helper function to cast from an entity type with minimum features
  /// to an entity with a different set of features. When the entity is cast
  /// successfully, it is added to _castMap so that subsequent casts will
  /// use the entity from the map.
  /// \tparam PolicyT The feature policy, such as
  /// `ignition::physics::FeaturePolicy3d`.
  /// \tparam ToFeatureList The list of features of the resulting entity.
  /// \tparam MinimumFeatureList The minimum list of features.
  /// \tparam ToEntity Type of entities with ToFeatureList
  /// \tparam MinimumEntity Type of entities with MinimumFeatureList
  /// \param[in] _entity Entity ID.
  /// \param[in] _minimumEntity Entity pointer with minimum features.
  /// \param[in] _castMap Map to store entities that have already been cast.
  template <
      typename PolicyT,
      typename ToFeatureList,
      typename MinimumFeatureList,
      template <typename, typename> class ToEntity,
      template <typename, typename> class MinimumEntity>
  physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>> entityCast(
      Entity _entity,
      const physics::EntityPtr<MinimumEntity<PolicyT, MinimumFeatureList>>
        &_minimumEntity,
      std::unordered_map<Entity, physics::EntityPtr<
        ToEntity<PolicyT, ToFeatureList>>> &_castMap)
  {
    // Has already been cast
    auto castIt = _castMap.find(_entity);
    if (castIt != _castMap.end())
    {
      return castIt->second;
    }

    physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>> castEntity;

    // Cast
    castEntity =
        physics::RequestFeatures<ToFeatureList>::From(_minimumEntity);

    if (castEntity)
    {
      _castMap.insert(std::make_pair(_entity, castEntity));
    }

    return castEntity;
  }
}
}
}
}
#endif
