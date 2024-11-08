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
#ifndef GZ_SIM_CREATEREMOVE_HH_
#define GZ_SIM_CREATEREMOVE_HH_

#include <memory>

#include <sdf/Actor.hh>
#include <sdf/Collision.hh>
#include <sdf/Gui.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParticleEmitter.hh>
#include <sdf/Physics.hh>
#include <sdf/Projector.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Types.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class SdfEntityCreatorPrivate;
    //
    /// \class SdfEntityCreator SdfEntityCreator.hh
    ///      gz/sim/SdfEntityCreator.hh
    /// \brief Provides convenient functions to spawn entities and load their
    /// plugins from SDF elements, to remove them, and to change their
    /// hierarchy.
    ///
    /// This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    class GZ_SIM_VISIBLE SdfEntityCreator
    {
      /// \brief Constructor
      /// \param[in] _ecm Entity component manager. This class keeps a pointer
      /// to it, but doesn't assume ownership.
      /// \param[in] _eventManager Event manager. This class keeps a pointer
      /// to it, but doesn't assume ownership.
      public: explicit SdfEntityCreator(EntityComponentManager &_ecm,
          EventManager &_eventManager);

      /// \brief Copy constructor
      /// \param[in] _creator SdfEntityCreator to copy.
      public: SdfEntityCreator(const SdfEntityCreator &_creator);

      /// \brief Move constructor
      /// \param[in] _creator SdfEntityCreator to move.
      public: SdfEntityCreator(SdfEntityCreator &&_creator) noexcept;

      /// \brief Move assignment operator.
      /// \param[in] _creator SdfEntityCreator component to move.
      /// \return Reference to this.
      public: SdfEntityCreator &operator=(SdfEntityCreator &&_creator) noexcept;

      /// \brief Copy assignment operator.
      /// \param[in] _creator SdfEntityCreator to copy.
      /// \return Reference to this.
      public: SdfEntityCreator &operator=(const SdfEntityCreator &_creator);

      /// \brief Destructor.
      public: ~SdfEntityCreator();

      /// \brief Create all entities that exist in the sdf::World object and
      /// load their plugins.
      /// \param[in] _world SDF world object.
      /// \return World entity.
      public: Entity CreateEntities(const sdf::World *_world);

      /// \brief Create all entities that exist in the sdf::World object and
      /// load their plugins.
      /// \param[in] _world SDF world object.
      /// \param[in] _worldEntity The world entity object.
      public: void CreateEntities(const sdf::World *_world,
                  Entity _worldEntity);

      /// \brief Create all entities that exist in the sdf::Model object and
      /// load their plugins. Also loads plugins of child sensors.
      /// \param[in] _model SDF model object.
      /// \return Model entity.
      public: Entity CreateEntities(const sdf::Model *_model);

      /// \brief Create all entities that exist in the sdf::Actor object and
      /// load their plugins.
      /// \param[in] _actor SDF actor object.
      /// \return Actor entity.
      public: Entity CreateEntities(const sdf::Actor *_actor);

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

      /// \brief Create all entities that exist in the sdf::Joint object and
      /// load their plugins.
      /// \param[in] _joint SDF joint object.
      /// \param[in] _resolved True if all frames are already resolved
      /// \return Joint entity.
      public: Entity CreateEntities(const sdf::Joint *_joint, bool _resolved);

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

      /// \brief Create all entities that exist in the sdf::Sensor object.
      /// Sensor plugins won't be directly loaded by this function.
      /// \param[in] _sensor SDF sensor object.
      /// \return Sensor entity.
      /// \sa CreateEntities(const sdf::Model *)
      public: Entity CreateEntities(const sdf::Sensor *_sensor);

      /// \brief Create all entities that exist in the
      /// sdf::ParticleEmitter object.
      /// \param[in] _emitter SDF ParticleEmitter object.
      /// \return ParticleEmitter entity.
      /// \sa CreateEntities(const sdf::Link *)
      public: Entity CreateEntities(const sdf::ParticleEmitter *_emitter);

      /// \brief Create all entities that exist in the
      /// sdf::Projector object.
      /// \param[in] _projector SDF Projector object.
      /// \return Projector entity.
      public: Entity CreateEntities(const sdf::Projector *_projector);

      /// \brief Request an entity deletion. This will insert the request
      /// into a queue. The queue is processed toward the end of a simulation
      /// update step.
      /// \param[in] _entity Entity to be removed.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      public: void RequestRemoveEntity(const Entity _entity,
          bool _recursive = true);

      /// \brief Set an entity's parent entity. This function takes care of
      /// updating the `EntityComponentManager` and necessary components.
      /// \param[in] _child Entity which should be parented.
      /// \param[in] _parent Entity which should be _child's parent.
      public: void SetParent(Entity _child, Entity _parent);

      /// \brief Overloaded function to recursively create model entities
      /// making sure to override the nested model's static property to true if
      /// its parent is static
      /// \param[in] _model SDF model object.
      /// \param[in] _staticParent True if parent is static, false otherwise.
      /// \return Model entity.
      private: Entity CreateEntities(const sdf::Model *_model,
                                     bool _staticParent);

      /// \brief Load plugins for all models
      private: void LoadModelPlugins();

      /// \brief Pointer to private data.
      private: std::unique_ptr<SdfEntityCreatorPrivate> dataPtr;
    };
    }
  }
}
#endif
