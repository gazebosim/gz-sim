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

#include <string>
#include <set>
#include <unordered_map>

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/SystemLoader.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class SimulationRunner;

    class LevelManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      /// \param[in] _useLevels Whether to use the levels defined. If false, all
      /// entities will be added to the default level
      public: LevelManager(SimulationRunner *_runner, bool _useLevels = false);

      /// \brief Configure the level manager
      public: void Configure();

      /// \brief Maintains a state machine for levels and mark them for
      /// loading/unloading depending on the state of performers.
      ///
      /// This is where we compute intersections and determine if a performer is
      /// in a level or not.
      public: void UpdateLevelsState();

      /// \brief Load levels that have been marked for loading.
      public: void LoadActiveLevels();

      /// \brief Load levels that have been marked for unloading.
      public: void UnloadInactiveLevels();

      /// \brief Read level and performer information from the sdf::World object
      private: void ReadLevelPerformerInfo();

      /// \brief Create performers
      /// Assuming that a simulation runner performer-centered
      private: void CreatePerformers();

      /// \brief Create entities and components for a model
      /// \param[in] _model sdf::Model to load
      /// \param[in] _world Parent world entity
      /// \return Created model entity
      private: Entity LoadModel(const sdf::Model &_model,
                                const Entity _worldEntity);

      /// \brief Create entities and components for a light
      /// \param[in] _light sdf::Light to load
      /// \param[in] _world Parent world entity
      /// \return Created light entity
      private: Entity LoadLight(const sdf::Light &_light,
                                const Entity _worldEntity);

      /// \brief Create all entities that exist in the sdf::World object and
      /// load their plugins.
      /// \param[in] _world SDF World object.
      /// \return Created world entity.
      public: Entity CreateEntities(const sdf::World *_world);

      /// \brief Create all entities that exist in the sdf::Model object and
      /// load their plugins.
      /// \param[in] _model SDF model object.
      /// \return Created model entity.
      public: Entity CreateEntities(const sdf::Model *_model);

      /// \brief Create all entities that exist in the sdf::Light object and
      /// load their plugins.
      /// \param[in] _light SDF light object.
      /// \return Created light entity.
      public: Entity CreateEntities(const sdf::Light *_light);

      /// \brief Create all entities that exist in the sdf::Link object and
      /// load their plugins.
      /// \param[in] _link SDF link object.
      /// \return Created link entity.
      public: Entity CreateEntities(const sdf::Link *_link);

      /// \brief Create all entities that exist in the sdf::Joint object and
      /// load their plugins.
      /// \param[in] _joint SDF joint object.
      /// \return Created joint entity.
      public: Entity CreateEntities(const sdf::Joint *_joint);

      /// \brief Create all entities that exist in the sdf::Visual object and
      /// load their plugins.
      /// \param[in] _visual SDF visual object.
      /// \return Created visual entity.
      public: Entity CreateEntities(const sdf::Visual *_visual);

      /// \brief Create all entities that exist in the sdf::Collision object and
      /// load their plugins.
      /// \param[in] _collision SDF collision object.
      /// \return Created collision entity.
      public: Entity CreateEntities(const sdf::Collision *_collision);

      /// \brief Create all entities that exist in the sdf::Sensor object and
      /// load their plugins.
      /// \param[in] _sensor SDF sensor object.
      /// \return Created sensor entity.
      public: Entity CreateEntities(const sdf::Sensor *_sensor);

      /// \brief Load system plugins for a given entity.
      /// \param[in] _sdf SDF element
      /// \param[in] _entity Entity on which the plugin is attached
      public: void LoadPlugins(const sdf::ElementPtr &_sdf,
                               const Entity _entity);

      // \brief Erase the entity and its children recursively
      /// \param[in] _entity Entity to erase
      public: void EraseEntityRecursively(const Entity _entity);

      private: void ReadPerformers(const sdf::ElementPtr &_sdf);
      private: void ReadLevels(const sdf::ElementPtr &_sdf);
      private: void ConfigureDefaultLevel();

      /// \brief Names of entities to currently active (loaded).
      private: std::set<std::string> activeEntityNames;

      /// \brief Names of entities to load. Currently model and lights are the
      /// only entities to be loaded and unloaded
      private: std::set<std::string> entityNamesToLoad;

      /// \brief Names of entities to unload. Currently model and lights are the
      /// only entities to be loaded and unloaded
      private: std::set<std::string> entityNamesToUnload;

      /// \brief Pointer to the simulation runner associated with the level
      /// manager.
      private: SimulationRunner * const runner;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, Entity> performerMap;

      /// \brief Names of all entities that have assigned levels
      private: std::set<std::string> entityNamesInLevels;

      /// \brief Names of all entities that have not been assigned any level.
      /// These belong to the default level
      private: std::set<std::string> entityNamesInDefault;

      /// \brief Graph of entities currenty loaded in the level. This
      /// is useful for erasing entities when a level is unloaded. This
      /// graph won't contain performers
      private: math::graph::DirectedGraph<Entity, bool> entityGraph;

      /// \brief Entity of the world
      private: Entity worldEntity = kNullEntity;

      /// \brief Entity of the world
      private: bool useLevels{false};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_LEVELMANAGER_HH

