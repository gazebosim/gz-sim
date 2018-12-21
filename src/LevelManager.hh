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
#include <unordered_set>
#include <unordered_map>

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
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
      /// level manager
      public: explicit LevelManager(SimulationRunner *_runner);

      /// \brief Read level and performer information from the sdf::World object
      public: void ReadLevelPerformerInfo();

      /// \brief Create performers
      /// Assuming that a simulation runner performer-centered
      public: void CreatePerformers();

      /// \brief Maintains a state machine for levels and mark them for
      /// loading/unloading depending on the state of performers.
      ///
      /// This is where we compute intersections and determine if a performer is
      /// in a level or not.
      public: void UpdateLevels();

      /// \brief Load levels that have been marked for loading.
      public: void LoadActiveLevels();

      /// \brief Load levels that have been marked for unloading.
      public: void UnloadInactiveLevels();

      /// \brief Create entities and components for a model
      /// \param[in] _model sdf::Model to load
      /// \param[in] _world EntityId of the parent world
      /// \return Id of created model entity
      private: EntityId LoadModel(const sdf::Model &_model,
                                  const EntityId _worldEntity);

      /// \brief Create entities and components for a light
      /// \param[in] _light sdf::Light to load
      /// \param[in] _world EntityId of the parent world
      /// \return Id of created light entity
      private: EntityId LoadLight(const sdf::Light &_light,
                                  const EntityId _worldEntity);

      /// \brief Create all entities that exist in the sdf::World object and
      /// load their plugins.
      /// \return Id of world entity.
      public: EntityId CreateEntities(const sdf::World *_world);

      /// \brief Create all entities that exist in the sdf::Model object and
      /// load their plugins.
      /// \param[in] _model SDF model object.
      /// \return Id of model entity.
      public: EntityId CreateEntities(const sdf::Model *_model);

      /// \brief Create all entities that exist in the sdf::Light object and
      /// load their plugins.
      /// \param[in] _light SDF light object.
      /// \return Id of light entity.
      public: EntityId CreateEntities(const sdf::Light *_light);

      /// \brief Create all entities that exist in the sdf::Link object and
      /// load their plugins.
      /// \param[in] _link SDF link object.
      /// \return Id of link entity.
      public: EntityId CreateEntities(const sdf::Link *_link);

      /// \brief Create all entities that exist in the sdf::Joint object and
      /// load their plugins.
      /// \param[in] _joint SDF joint object.
      /// \return Id of joint entity.
      public: EntityId CreateEntities(const sdf::Joint *_joint);

      /// \brief Create all entities that exist in the sdf::Visual object and
      /// load their plugins.
      /// \param[in] _visual SDF visual object.
      /// \return Id of visual entity.
      public: EntityId CreateEntities(const sdf::Visual *_visual);

      /// \brief Create all entities that exist in the sdf::Collision object and
      /// load their plugins.
      /// \param[in] _collision SDF collision object.
      /// \return Id of collision entity.
      public: EntityId CreateEntities(const sdf::Collision *_collision);

      /// \brief Load system plugins for a given entity.
      /// \param[in] _sdf SDF element
      /// \param[in] _id Entity Id
      public: void LoadPlugins(const sdf::ElementPtr &_sdf, const EntityId _id);

      // \brief Erase the entity and its children recursively
      /// \param[in] _id Entity Id
      public: void EraseEntityRecursively(const EntityId _id);

      private: void ReadPerformers(const sdf::ElementPtr &_sdf);
      private: void ReadLevels(const sdf::ElementPtr &_sdf);

      /// \brief Set of currently active (loaded) levels
      private: std::set<EntityId> activeLevels;

      /// \brief Set of levels to load
      private: std::set<EntityId> levelsToLoad;

      /// \brief Set of levels to unload
      private: std::set<EntityId> levelsToUnload;

      /// \brief Pointer to the simulation runner associated with the level
      /// manager.
      private: SimulationRunner * const runner;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, EntityId> performerMap;

      /// \brief Graph of entities currenty loaded in the level. This
      /// is useful for erasing entities when a level is unloaded. This
      /// graph won't contain performers
      private: math::graph::DirectedGraph<EntityId, bool> entityGraph;

      /// \brief EntityId of the world
      private: EntityId worldEntity = kNullEntity;
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_LEVELMANAGER_HH

