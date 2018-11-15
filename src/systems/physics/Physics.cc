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

#include <iostream>

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/plugin/Register.hh>

// Features
#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/SphereShape.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructVisual.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

// SDF
#include <sdf/Collision.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
// Components
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "Physics.hh"

using namespace ignition::gazebo::systems;
namespace components = ignition::gazebo::components;


// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
  public: using MinimumFeatureList = ignition::physics::FeatureList<
          ignition::physics::LinkFrameSemantics,
          ignition::physics::ForwardStep,
          ignition::physics::GetEntities,
          ignition::physics::sdf::ConstructSdfCollision,
          ignition::physics::sdf::ConstructSdfLink,
          ignition::physics::sdf::ConstructSdfModel,
          ignition::physics::sdf::ConstructSdfVisual,
          ignition::physics::sdf::ConstructSdfWorld
          >;


  public: using EnginePtrType = ignition::physics::EnginePtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using WorldPtrType = ignition::physics::WorldPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using ModelPtrType = ignition::physics::ModelPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using LinkPtrType = ignition::physics::LinkPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Create physics entities
  public: void CreatePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Step the simulationrfor each world
  public: void Step(const std::chrono::steady_clock::duration &_dt);

  /// \brief Step the simulation for each world
  public: void UpdateSim(EntityComponentManager &_ecm) const;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// ign-physics.
  public: std::unordered_map<EntityId, WorldPtrType> entityWorldMap;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// ign-physics.
  public: std::unordered_map<EntityId, ModelPtrType> entityModelMap;

  /// \brief A map between model entity ids and their canonical links. For now
  /// we assume the first link we encounter for a given model is it's canonical
  /// link. The key is the Model entity ID.
  public: std::unordered_map<EntityId, EntityId> canonicalLinkMap;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics.
  public: std::unordered_map<EntityId, LinkPtrType> entityLinkMap;

  /// \brief used to store whether physics objects have been created.
  public: bool initialized = false;

  /// \brief Pointer to the underlying ign-physics Engine entity.
  public: EnginePtrType engine = nullptr;
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
  ignition::plugin::Loader pl;
  // dartsim_plugin_LIB is defined by cmake
  std::unordered_set<std::string> plugins = pl.LoadLibrary(dartsim_plugin_LIB);
  if (!plugins.empty())
  {
    const std::string className = "ignition::physics::dartsim::Plugin";
    ignition::plugin::PluginPtr plugin = pl.Instantiate(className);

    if (plugin)
    {
      this->dataPtr->engine = ignition::physics::RequestEngine<
        ignition::physics::FeaturePolicy3d,
        PhysicsPrivate::MinimumFeatureList>::From(plugin);
    }
    else
    {
      ignerr << "Unable to instantiate " << className << ".\n";
    }
  }
  else
  {
    ignerr << "Unable to load the " << dartsim_plugin_LIB << " library.\n";
    return;
  }
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (this->dataPtr->engine)
  {
    if (!this->dataPtr->initialized)
    {
      this->dataPtr->CreatePhysicsEntities(_ecm);
      this->dataPtr->initialized = true;
    }

    // Only step if not paused.
    if (!_info.paused)
    {
      this->dataPtr->Step(_info.dt);
      this->dataPtr->UpdateSim(_ecm);
    }
  }
}

//////////////////////////////////////////////////
void Physics::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreatePhysicsEntities(const EntityComponentManager &_ecm)
{
    // Get all the worlds
  _ecm.Each<components::World, components::Name>(
      [&](const EntityId &_entity,
        const components::World * /* _world */,
        const components::Name *_name)->bool
      {
        if (this->entityWorldMap.find(_entity) == this->entityWorldMap.end())
        {
          sdf::World world;
          world.SetName(_name->Data());
          auto worldPtrPhys = this->engine->ConstructWorld(world);
          this->entityWorldMap.insert(std::make_pair(_entity, worldPtrPhys));
        }
        return true;
      });

  _ecm.Each<components::Model, components::Name, components::Pose,
            components::ParentEntity, components::Static>(
      [&](const EntityId &_entity,
        const components::Model * /* _model */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Static *_static)->bool
      {
        if (this->entityModelMap.find(_entity) == this->entityModelMap.end())
        {
          sdf::Model model;
          model.SetName(_name->Data());
          model.SetPose(_pose->Data());
          model.SetStatic(_static->Data());
          auto worldPtrPhys = this->entityWorldMap.at(_parent->Data());
          auto modelPtrPhys = worldPtrPhys->ConstructModel(model);
          this->entityModelMap.insert(std::make_pair(_entity, modelPtrPhys));
        }
        return true;
      });

  _ecm.Each<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const EntityId &_entity,
        const components::Link * /* _link */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        if (this->entityLinkMap.find(_entity) == this->entityLinkMap.end())
        {
          sdf::Link link;
          link.SetName(_name->Data());
          link.SetPose(_pose->Data());

          // get link inertial
          auto inertial = _ecm.Component<components::Inertial>(_entity);
          if (inertial)
          {
            link.SetInertial(inertial->Data());
          }

          auto modelPtrPhys = this->entityModelMap.at(_parent->Data());
          auto linkPtrPhys = modelPtrPhys->ConstructLink(link);
          this->entityLinkMap.insert(std::make_pair(_entity, linkPtrPhys));
          auto canonLinkIt = this->canonicalLinkMap.find(_parent->Data());
          // Assume canonical link if the key is not found
          if (canonLinkIt == this->canonicalLinkMap.end())
          {
            this->canonicalLinkMap[_parent->Data()] = _entity;
          }
        }
        return true;
      });

  // We don't need to add visuals to the physics engine.

  // collisions
  _ecm.Each<components::Collision, components::Name, components::Pose,
            components::Geometry, components::ParentEntity>(
      [&](const EntityId & /* _entity */,
        const components::Collision * /* _collision */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::Geometry *_geom,
        const components::ParentEntity *_parent)->bool
      {
        sdf::Collision collision;
        collision.SetName(_name->Data());
        collision.SetPose(_pose->Data());
        collision.SetGeom(_geom->Data());
        auto linkPtrPhys = this->entityLinkMap.at(_parent->Data());
        linkPtrPhys->ConstructCollision(collision);
        // for now, we won't have a map to the collision once it's added
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::Step(const std::chrono::steady_clock::duration &_dt)
{
  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

  input.Get<std::chrono::steady_clock::duration>() = _dt;

  for (auto &[entity, world] : this->entityWorldMap)
  {
    world->Step(output, state, input);
  }
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateSim(EntityComponentManager &_ecm) const
{
  _ecm.Each<components::Link, components::Pose, components::ParentEntity>(
      [&](const EntityId &_entity, components::Link * /*_link*/,
          components::Pose *_pose, components::ParentEntity *_parent)->bool
      {
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          auto canonLinkIt = this->canonicalLinkMap.find(_parent->Data());

          // The model that contains this link must have a canonical link.
          // Otherwise something is wrong, so return
          if (canonLinkIt == this->canonicalLinkMap.end())
          {
            ignerr << "The model " << _parent->Data() << " that contains this"
                   << " link should have a canonical link\n";
            return true;
          }

          if (canonLinkIt->second == _entity)
          {
            // This is the canonical link, update the model
            // get the pose component of the parent model
            auto parentPose =
                _ecm.Component<components::Pose>(_parent->Data());
            // if the parentPose is a nullptr, something is wrong with ECS
            // creation
            if (parentPose)
            {
              auto pose = linkIt->second->FrameDataRelativeToWorld().pose;
              // the Pose component, _pose, of this link is the initial
              // transform of the link w.r.t its model. This component never
              // changes because it's "fixed" to the model. Instead, we change
              // the model's pose here. The physics engine gives us the pose of
              // this link relative to world so to set the model's pose, we have
              // to premultiply it by the inverse of the initial transform of
              // the link w.r.t to its model.
              *parentPose = components::Pose(_pose->Data().Inverse() *
                                             math::eigen3::convert(pose));
            }
          }
          else
          {
            // Not the canonical link, so get the link's relative pose
            // \NOTE(addisu) Once ModelFrameSemantics are available, we should
            // resolve the relative pose by passing the model as the parent
            // frame.

            // Find the canonical link of the model that contains this link
            auto canonLinkPhys = this->entityLinkMap.at(canonLinkIt->second);
            auto canonFrame = canonLinkPhys->GetFrameID();
            // Get the pose relative to the canonical link
            auto pose = linkIt->second->FrameDataRelativeTo(canonFrame).pose;
            *_pose = components::Pose(math::eigen3::convert(pose));
          }
        }
        else
        {
          ignwarn << "Unknown link with id " << _entity << " found\n";
        }
        return true;
      });
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System,
                    Physics::ISystemUpdate)
