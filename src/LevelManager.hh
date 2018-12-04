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

#include <sdf/World.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class EntityComponentManager;

    class LevelManager
    {
      public: LevelManager(EntityComponentManager &_ecm,
                           const sdf::World *_world);

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

      /// \brief Set of currently active (loaded) levels
      private: std::unordered_set<EntityId> activeLevels;

      /// \brief Set of levels to load
      private: std::unordered_set<EntityId> levelsToLoad;

      /// \brief Set of levels to unload
      private: std::unordered_set<EntityId> levelsToUnload;

      /// \brief Pointer to the Entity Component Manager
      private: EntityComponentManager *entityCompMgr;

      /// \brief Pointer to the sdf::World object
      private: const sdf::World *sdfWorld;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, EntityId> performerMap;
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_LEVELMANAGER_HH

