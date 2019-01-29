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

#include <map>
#include <memory>
#include <string>
#include <vector>

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

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \typedef FactoryFn
    /// \brief Prototype for component factory generation
    using FactoryFn = std::unique_ptr<components::Component> (*)();

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

      /// \brief Register a component.
      /// \param[in] _type Type of component to register.
      /// \param[in] _factoryfn Function that generates the component.
      public: static void Register(const std::string &_type,
                                   FactoryFn _factoryfn);

      /// \brief Create a new instance of a component.
      /// \param[in] _type Type of component to create.
      /// \return Pointer to a component. Null if the component
      /// type could not be handled.
      public: template<typename T>
      static std::unique_ptr<T> New(const std::string &_type)
      {
        return std::unique_ptr<T>(static_cast<T*>(New(_type).release()));
      }

      /// \brief Create a new instance of a component.
      /// \param[in] _type Type of component to create.
      /// \return Pointer to a component. Null if the component
      /// type could not be handled.
      public: static std::unique_ptr<components::Component> New(
          const std::string &_type);

      /// \brief Get all the component types.
      /// \param[out] _types Vector of strings of the component types.
      public: static std::vector<std::string> Components();

      /// \brief Pointer to private data.
      private: std::unique_ptr<FactoryPrivate> dataPtr;

      /// \brief A list of registered component types.
      private: inline static std::map<std::string, FactoryFn> compMap;
    };

    /// \brief Static component registration macro.
    ///
    /// Use this macro to register components.
    /// \param[in] _compType Component type name.
    /// \param[in] _classname Class name for component.
    #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
    IGNITION_GAZEBO_VISIBLE \
    std::unique_ptr<ignition::gazebo::components::Component> New##_classname() \
    { \
      return std::unique_ptr<ignition::gazebo::components::_classname>(\
          new ignition::gazebo::components::_classname); \
    } \
    class IGNITION_GAZEBO_VISIBLE IgnGazeboComponents##_classname \
    { \
      public: IgnGazeboComponents##_classname() \
      { \
        ignition::gazebo::Factory::Register(\
          _compType, New##_classname);\
      } \
    }; \
    static IgnGazeboComponents##_classname\
      IgnitionGazeboComponentsInitializer##_classname;
    }
  }
}
#endif
