/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_FACTORY_HH_
#define IGNITION_GAZEBO_FACTORY_HH_

#include <memory>

#include <sdf/Collision.hh>
#include <sdf/Gui.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class FactoryPrivate;
    //
    /// \class Factory Factory.hh ignition/gazebo/Factory.hh
    /// \brief Provides convenient functions to spawn entities and load their
    /// plugins from SDF elements.
    ///
    /// This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    class IGNITION_GAZEBO_VISIBLE Factory
    {
      /// \brief Constructor
      /// \param[in] _ecm Entity component manager. This class keeps a pointer
      /// to it, but doesn't assume ownership.
      /// \param[in] _eventManager Event manager. This class keeps a pointer
      /// to it, but doesn't assume ownership.
      public: explicit Factory(EntityComponentManager &_ecm,
          EventManager &_eventManager);

      /// \brief Copy constructor
      /// \param[in] _factory Factory to copy.
      public: Factory(const Factory &_factory);

      /// \brief Move constructor
      /// \param[in] _factory Factory to move.
      public: Factory(Factory &&_factory) noexcept;

      /// \brief Move assignment operator.
      /// \param[in] _factory Factory component to move.
      /// \return Reference to this.
      public: Factory &operator=(Factory &&_factory) noexcept;

      /// \brief Copy assignment operator.
      /// \param[in] _factory Factory to copy.
      /// \return Reference to this.
      public: Factory &operator=(const Factory &_factory);

      /// \brief Destructor.
      public: ~Factory();

      /// \brief Create all entities that exist in the sdf::World object and
      /// load their plugins.
      /// \param[in] _world SDF world object.
      /// \return World entity.
      public: Entity CreateEntities(const sdf::World *_world);

      /// \brief Create all entities that exist in the sdf::Model object and
      /// load their plugins.
      /// \param[in] _model SDF model object.
      /// \return Model entity.
      public: Entity CreateEntities(const sdf::Model *_model);

      /// \brief Create all entities that exist in the sdf::Light object and
      /// load their plugins.
      /// \param[in] _light SDF light object.
      /// \return Light entity.
      public: Entity CreateEntities(const sdf::Light *_light);

      /// \brief Create all entities that exist in the sdf::Link object and
      /// load their plugins.
      /// \param[in] _link SDF link object.
      /// \return Link entity.
      public: Entity CreateEntities(const sdf::Link *_link);

      /// \brief Create all entities that exist in the sdf::Joint object and
      /// load their plugins.
      /// \param[in] _joint SDF joint object.
      /// \return Joint entity.
      public: Entity CreateEntities(const sdf::Joint *_joint);

      /// \brief Create all entities that exist in the sdf::Visual object and
      /// load their plugins.
      /// \param[in] _visual SDF visual object.
      /// \return Visual entity.
      public: Entity CreateEntities(const sdf::Visual *_visual);

      /// \brief Create all entities that exist in the sdf::Collision object and
      /// load their plugins.
      /// \param[in] _collision SDF collision object.
      /// \return Collision entity.
      public: Entity CreateEntities(const sdf::Collision *_collision);

      /// \brief Create all entities that exist in the sdf::Sensor object and
      /// load their plugins.
      /// \param[in] _sensor SDF sensor object.
      /// \return Sensor entity.
      public: Entity CreateEntities(const sdf::Sensor *_sensor);

      /// \brief Pointer to private data.
      private: std::unique_ptr<FactoryPrivate> dataPtr;
    };
    }
  }
}
#endif
