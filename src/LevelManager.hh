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

#ifndef GZ_SIM_LEVELMANAGER_HH
#define GZ_SIM_LEVELMANAGER_HH

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <list>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Types.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
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
    class GZ_SIM_VISIBLE LevelManager
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

      /// \brief Read level and performer information from the sdf::World
      /// object
      /// \param[in] _world The SDF world
      public: void ReadLevelPerformerInfo(const sdf::World &_world);

      /// \brief Determine which entities belong to the default level and
      /// schedule them to be loaded
      public: void ConfigureDefaultLevel();

      /// \brief Load entities that have been marked for loading.
      /// \param[in] _namesToLoad List of of entity names to load
      private: void LoadActiveEntities(
          const std::set<std::string> &_namesToLoad);

      /// \brief Unload entities that have been marked for unloading.
      /// \param[in] _namesToUnload List of entity names to unload
      private: void UnloadInactiveEntities(
          const std::set<std::string> &_namesToUnload);

      /// \brief Read information about performers from the sdf Element and
      /// create performer entities
      /// \param[in] _plugin sdf::Plugin of the gz::sim plugin tag
      private: void ReadPerformers(const sdf::Plugin &_plugin);

      /// \brief Read information about levels from the sdf Element and
      /// create level entities
      /// \param[in] _plugin sdf::Plugin of the gz::sim plugin tag
      private: void ReadLevels(const sdf::Plugin &_plugin);

      /// \brief Determine if a level is active
      /// \param[in] _entity Entity of level to be checked
      /// \return True of the level is currently active
      private: bool IsLevelActive(const Entity _entity) const;

      /// \brief Service callback to create a new performer.
      /// \param[in] _req Message that contains perfomer information.
      /// \param[out] _rep Reply message, which is set to true when the
      /// performer has been added.
      /// \return True if the service call completed.
      private: bool OnSetPerformer(const msgs::StringMsg &_req,
                                   msgs::Boolean &_rep);

      /// \brief Helper function that creates a performer entity
      /// based on geometry.
      /// \param[in] _name Name of the performer entity. This should also be
      /// the name of an existing model.
      /// \param[in] _geom SDF geometry that defines the performer's
      /// bounding box.
      /// \return 0 if the performer was added, 1 if the performer is
      /// a duplicate and was not added, -1 if a model with _name could not be
      /// found.
      private: int CreatePerformerEntity(const std::string &_name,
                   const sdf::Geometry &_geom);

      private: void UnloadLevel(const Entity &_entity,
                   const std::set<std::string> &_entityNamesMarked = {});

      /// \brief List of currently active levels
      private: std::set<Entity> activeLevels;

      /// \brief Names of entities that are currently active (loaded).
      private: std::set<std::string> activeEntityNames;

      /// \brief Pointer to the simulation runner associated with the level
      /// manager.
      private: SimulationRunner *const runner;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, Entity> performerMap;

      /// \brief Names of all entities that have assigned levels
      private: std::set<std::string> entityNamesInLevels;

      /// \brief Flag whether to use levels or not.
      private: bool useLevels{false};

      /// \brief Entity Creator API.
      private: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};

      /// \brief Transport node.
      private: gz::transport::Node node;

      /// \brief The list of performers to add.
      private: std::list<std::pair<std::string, sdf::Geometry>> performersToAdd;

      /// \brief Mutex to protect performersToAdd list.
      private: std::mutex performerToAddMutex;
    };
    }
  }
}
// GZ_SIM_LEVELMANAGER_HH
#endif
