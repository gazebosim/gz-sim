/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_WORLD_HH_
#define IGNITION_GAZEBO_WORLD_HH_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sdf/Atmosphere.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN WorldPrivate;
    //
    /// \class World World.hh ignition/gazebo/World.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a world
    /// entity.
    ///
    /// For example, given a world's entity, to find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    World world(entity);
    ///    std::string name = world.Name(ecm);
    class IGNITION_GAZEBO_VISIBLE World {
      /// \brief Constructor
      /// \param[in] _entity World entity
      public: explicit World(gazebo::Entity _entity = kNullEntity);

      /// \brief Copy constructor
      /// \param[in] _world World to copy.
      public: World(const World &_world);

      /// \brief Move constructor
      /// \param[in] _world World to move.
      public: World(World &&_world) noexcept;

      /// \brief Move assignment operator.
      /// \param[in] _world World component to move.
      /// \return Reference to this.
      public: World &operator=(World &&_world) noexcept;

      /// \brief Copy assignment operator.
      /// \param[in] _world World to copy.
      /// \return Reference to this.
      public: World &operator=(const World &_world);

      /// \brief Destructor
      public: virtual ~World();

      /// \brief Get the entity which this World is related to.
      /// \return World entity.
      public: gazebo::Entity Entity() const;

      /// \brief Check whether this world correctly refers to an entity that
      /// has a components::World.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid world in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the world's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return World's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the gravity in m/s^2.
      /// \param[in] _ecm Entity-component manager.
      /// \return Gravity vector or nullopt if the entity does not
      /// have a components::Gravity component.
      public: std::optional<math::Vector3d> Gravity(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the magnetic field in Tesla.
      /// \param[in] _ecm Entity-component manager.
      /// \return Magnetic field vector or nullopt if the entity does not
      /// have a components::MagneticField component.
      public: std::optional<math::Vector3d> MagneticField(
          const EntityComponentManager &_ecm) const;

      /// \brief Get atmosphere information.
      /// \param[in] _ecm Entity-component manager.
      /// \return Magnetic field vector or nullopt if the entity does not
      /// have a components::Atmosphere component.
      public: std::optional<sdf::Atmosphere> Atmosphere(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the ID of a light entity which is an immediate child of
      /// this world.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Light name.
      /// \return Light entity.
      public: gazebo::Entity LightByName(const EntityComponentManager &_ecm,
          const std::string &_name) const;

      /// \brief Get the ID of a actor entity which is an immediate child of
      /// this world.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Actor name.
      /// \return Actor entity.
      public: gazebo::Entity ActorByName(const EntityComponentManager &_ecm,
          const std::string &_name) const;

      /// \brief Get the ID of a model entity which is an immediate child of
      /// this world.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Model name.
      /// \return Model entity.
      public: gazebo::Entity ModelByName(const EntityComponentManager &_ecm,
          const std::string &_name) const;

      /// \brief Get all lights which are immediate children of this world.
      /// \param[in] _ecm Entity-component manager.
      /// \return All lights in this world.
      public: std::vector<gazebo::Entity> Lights(
          const EntityComponentManager &_ecm) const;

      /// \brief Get all actors which are immediate children of this world.
      /// \param[in] _ecm Entity-component manager.
      /// \return All actors in this world.
      public: std::vector<gazebo::Entity> Actors(
          const EntityComponentManager &_ecm) const;

      /// \brief Get all models which are immediate children of this world.
      /// \param[in] _ecm Entity-component manager.
      /// \return All models in this world.
      public: std::vector<gazebo::Entity> Models(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the number of lights which are immediate children of this
      /// world.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of lights in this world.
      public: uint64_t LightCount(const EntityComponentManager &_ecm) const;

      /// \brief Get the number of actors which are immediate children of this
      /// world.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of actors in this world.
      public: uint64_t ActorCount(const EntityComponentManager &_ecm) const;

      /// \brief Get the number of models which are immediate children of this
      /// world.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of models in this world.
      public: uint64_t ModelCount(const EntityComponentManager &_ecm) const;

      /// \brief Pointer to private data.
      private: std::unique_ptr<WorldPrivate> dataPtr;
    };
    }
  }
}
#endif
