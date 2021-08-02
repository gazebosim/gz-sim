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
#ifndef IGNITION_GAZEBO_SYSTEMS_PHYSICS_CANONICAL_LINK_MODEL_TRACKER_HH_
#define IGNITION_GAZEBO_SYSTEMS_PHYSICS_CANONICAL_LINK_MODEL_TRACKER_HH_

#include <set>
#include <unordered_map>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/config.hh"

namespace ignition::gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief Helper class that keeps track of which models have a particular
  /// canonical link. This is useful in the physics system for updating model
  /// poses - if a canonical link moved in the most recent physics step, then
  /// all of the models that have this canonical link should be updated. It's
  /// important to preserve topological ordering of the models in case there's
  /// a nested model that shares the same canonical link (in a case like this,
  /// the parent model pose needs to be updated before updating the child model
  /// pose - see the documentation that explains how model pose updates are
  /// calculated in PhysicsPrivate::UpdateSim to understand why nested model
  /// poses need to be updated in topological order).
  ///
  /// It's possible to loop through all of the models and to update poses if the
  /// model moved using something like EntityComponentManager::Each, but the
  /// performance of this approach is worse than using just the moved canonical
  /// links to determine which model poses should be updated (consider the case
  /// where there are a lot of non-static models in a world, but only a few move
  /// frequently - if using EntityComponentManager::Each, we still need to check
  /// every single non-static model after a physics update to make sure that the
  /// model did not move. If we instead use the updated canonical link
  /// information, then we can skip iterating over/checking the models that
  /// don't need to be updated).
  class CanonicalLinkModelTracker
  {
    /// \brief Save mappings for new models and their canonical links
    /// \param[in] _ecm EntityComponentManager
    public: void AddNewModels(const EntityComponentManager &_ecm);

    /// \brief Get a topological ordering of models that have a particular
    /// canonical link
    /// \param[in] _canonicalLink The canonical link
    /// \return The models that have this link as their canonical link, in
    /// topological order
    public: const std::set<Entity> &CanonicalLinkModels(
                const Entity _canonicalLink) const;

    /// \brief Remove a link from the mapping. This method should be called when
    /// a link is removed from simulation
    /// \param[in] _link The link to remove
    public: void RemoveLink(const Entity &_link);

    /// \brief A mapping of canonical links to the models that have this
    /// canonical link. The key is the canonical link entity, and the value is
    /// the model entities that have this canonical link. The models in the
    /// value are in topological order
    private: std::unordered_map<Entity, std::set<Entity>> linkModelMap;

    /// \brief An empty set of models that is returned from the
    /// CanonicalLinkModels method for links that map to no models
    private: const std::set<Entity> emptyModelOrdering{};
  };

  void CanonicalLinkModelTracker::AddNewModels(
      const EntityComponentManager &_ecm)
  {
    _ecm.EachNew<components::Model, components::ModelCanonicalLink>(
        [this](const Entity &_model, const components::Model *,
          const components::ModelCanonicalLink *_canonicalLinkComp)
        {
          this->linkModelMap[_canonicalLinkComp->Data()].insert(_model);
          return true;
        });
  }

  const std::set<Entity> &CanonicalLinkModelTracker::CanonicalLinkModels(
      const Entity _canonicalLink) const
  {
    auto it = this->linkModelMap.find(_canonicalLink);
    if (it != this->linkModelMap.end())
      return it->second;

    // if an invalid entity was given, it maps to no models
    return this->emptyModelOrdering;
  }

  void CanonicalLinkModelTracker::RemoveLink(const Entity &_link)
  {
    this->linkModelMap.erase(_link);
  }
}
}
}

#endif
