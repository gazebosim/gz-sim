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

#ifndef IGNITION_GAZEBO_LEVELMANAGER_HH
#define IGNITION_GAZEBO_LEVELMANAGER_HH

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <sdf/Element.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class SimulationRunner;

    /// \brief Used to load / unload levels as performers move in a world.
    ///
    /// The manager will create entities to represent each level and performer.
    /// There will also be a default level to hold all entities which are
    /// outside other levels and should be present in simulation at all times.
    ///
    /// Level entities are direct children of the world entity, and performer
    /// entities are direct children of the models they correspond to. The
    /// final hierarchy looks as follows:
    ///
    /// world
    ///  - default level
    ///  - level
    ///  - light
    ///  - model
    ///    - performer
    ///    - link
    ///    - nested model
    ///
    /// The levels feature works with a few assumptions (currently):
    ///
    /// * Entities which are part of a level should not be modified during
    ///   simulation. Any component changes or additions will be ignored
    ///   when the level is reloaded. Likewise, they should not be deleted.
    /// * Entities spawned during simulation are part of the default level.
    ///
    class LevelManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      /// \param[in] _useLevels Whether to use the levels defined. If false, all
      /// will only be loaded for active performers.
      public: LevelManager(SimulationRunner *_runner, bool _useLevels = false);

      /// \brief Load and unload levels
      /// This is where we compute intersections and determine if a performer is
      /// in a level or not. This needs to be called by the simulation runner at
      /// every update cycle
      public: void UpdateLevelsState();

      /// \brief Load entities that have been marked for loading.
      /// \param[in] _namesToLoad List of of entity names to load
      private: void LoadActiveEntities(
          const std::set<std::string> &_namesToLoad);

      /// \brief Unload entities that have been marked for unloading.
      /// \param[in] _namesToUnload List of entity names to unload
      private: void UnloadInactiveEntities(
          const std::set<std::string> &_namesToUnload);

      /// \brief Read level and performer information from the sdf::World
      /// object
      private: void ReadLevelPerformerInfo();

      /// \brief Create performers
      /// Assuming that a simulation runner is performer-centered
      private: void CreatePerformers();

      /// \brief Read information about performers from the sdf Element and
      /// create performer entities
      /// \param[in] _sdf sdf::ElementPtr of the ignition::gazebo plugin tag
      private: void ReadPerformers(const sdf::ElementPtr &_sdf);

      /// \brief Read information about levels from the sdf Element and
      /// create level entities
      /// \param[in] _sdf sdf::ElementPtr of the ignition::gazebo plugin tag
      private: void ReadLevels(const sdf::ElementPtr &_sdf);

      /// \brief Determine which entities belong to the default level and
      /// schedule them to be loaded
      private: void ConfigureDefaultLevel();

      /// \brief Determine if a level is active
      /// \param[in] _entity Entity of level to be checked
      /// \return True of the level is currently active
      private: bool IsLevelActive(const Entity _entity) const;

      /// \brief List of currently active levels
      private: std::vector<Entity> activeLevels;

      /// \brief Names of entities that are currently active (loaded).
      private: std::set<std::string> activeEntityNames;

      /// \brief Pointer to the simulation runner associated with the level
      /// manager.
      private: SimulationRunner *const runner;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, Entity> performerMap;

      /// \brief Names of all entities that have assigned levels
      private: std::set<std::string> entityNamesInLevels;

      /// \brief Entity of the world.
      private: Entity worldEntity{kNullEntity};

      /// \brief Flag whether to use levels or not.
      private: bool useLevels{false};

      /// \brief Entity Creator API.
      private: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_LEVELMANAGER_HH

