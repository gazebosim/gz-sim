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

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/plugin/Register.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructVisual.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

// SDF
#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
// Components
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "ignition/gazebo/systems/Physics.hh"

using namespace ignition::gazebo::systems;
namespace components = ignition::gazebo::components;


// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
  public: using MinimumFeatureList = ignition::physics::FeatureList<
          ignition::physics::GetEntities,
          ignition::physics::sdf::ConstructSdfCollision,
          ignition::physics::sdf::ConstructSdfJoint,
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

  public: using JointPtrType = ignition::physics::JointPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;


  /// \brief a map between world entity ids in the ECM to World Entities in 
  /// ign-physics
  public: std::unordered_map<EntityId, WorldPtrType> entityWorldMap;

  /// \brief a map between model entity ids in the ECM to Model Entities in 
  /// ign-physics
  public: std::unordered_map<EntityId, ModelPtrType> entityModelMap;

  /// \brief a map between link entity ids in the ECM to Link Entities in 
  /// ign-physics
  public: std::unordered_map<EntityId, LinkPtrType> entityLinkMap;

  /// \brief a map between joint entity ids in the ECM to Joint Entities in 
  /// ign-physics
  public: std::unordered_map<EntityId, JointPtrType> entityJointMap;

  /// \brief used to store whether physics objects have been created
  public: bool initialized = false;
          
  /// \brief pointer to the underlying ign-physics Engine entity
  public: EnginePtrType engine;
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Init()
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLibrary("libignition-physics0-dartsim-plugin.so");
  const std::string className = "ignition::physics::dartsim::Plugin";
  ignition::plugin::PluginPtr plugin = pl.Instantiate(className);

  this->dataPtr->engine = ignition::physics::RequestEngine<
      ignition::physics::FeaturePolicy3d,
      PhysicsPrivate::MinimumFeatureList>::
      From(plugin);
  std::cout << "Found: " << this->dataPtr->engine->GetName() << std::endl;
}

//////////////////////////////////////////////////
void Physics::EntityAdded(const Entity &_entity,
                          const EntityComponentManager &_ecm)
{
  (void)_entity;
  (void)_ecm;
}

//////////////////////////////////////////////////
void Physics::EntityRemoved(const Entity &_entity,
                            const EntityComponentManager &_ecm)
{
  (void)_entity;
  (void)_ecm;
}

void Physics::PreUpdate(const std::chrono::steady_clock::duration &/* _dt */,
                        const EntityComponentManager &_ecm)
{

  if (!this->dataPtr->initialized)
  {
    // get worlds
    _ecm.Each<components::World, components::Name>(
        [&](const EntityId &_entity,
          const components::World * /* _world */,
          const components::Name *_name)
        {
          sdf::World world;
          world.SetName(_name->Data());
          std::cout << "Creating world: " << _name->Data() << std::endl;
          auto worldPtrPhys = this->dataPtr->engine->ConstructWorld(world);

          this->dataPtr->entityWorldMap.insert(
              std::make_pair(_entity, worldPtrPhys));
        });

    _ecm.Each<components::Model, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const EntityId &_entity,
          const components::Model * /* _model */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)
        {
          sdf::Model model;
          model.SetName(_name->Data());
          model.SetPose(_pose->Data());
          auto worldPtrPhys = this->dataPtr->entityWorldMap.at(_parent->Id());
          std::cout << "Creating model: " << worldPtrPhys->GetName() << ":"
                    << _name->Data() << std::endl;
          auto modelPtrPhys = worldPtrPhys->ConstructModel(model);
          this->dataPtr->entityModelMap.insert(
              std::make_pair(_entity, modelPtrPhys));

        });

    _ecm.Each<components::Link, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const EntityId &_entity,
          const components::Link * /* _link */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)
        {
          sdf::Link link;
          link.SetName(_name->Data());
          link.SetPose(_pose->Data());
          auto modelPtrPhys = this->dataPtr->entityModelMap.at(_parent->Id());
          std::cout << "Creating link: " << modelPtrPhys->GetName() << ":"
                    << _name->Data() << std::endl;
          auto linkPtrPhys = modelPtrPhys->ConstructLink(link);
          this->dataPtr->entityLinkMap.insert(
              std::make_pair(_entity, linkPtrPhys));

        });

    // visuals
    _ecm.Each<components::Visual, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const EntityId  &/* _entity */,
          const components::Visual * /* _visual */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetPose(_pose->Data());
          auto linkPtrPhys = this->dataPtr->entityLinkMap.at(_parent->Id());
          std::cout << "Creating visual: " << linkPtrPhys->GetName() << ":"
                    << _name->Data() << std::endl;
          linkPtrPhys->ConstructVisual(visual);
          // for now, we won't have a map to the visual once it's added
        });

    // collisions
    _ecm.Each<components::Collision, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const EntityId &/* _entity */,
          const components::Collision * /* _collision */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)
        {
          sdf::Collision collision;
          collision.SetName(_name->Data());
          collision.SetPose(_pose->Data());
          auto linkPtrPhys = this->dataPtr->entityLinkMap.at(_parent->Id());
          std::cout << "Creating collision: " << linkPtrPhys->GetName() << ":"
                    << _name->Data() << std::endl;
          linkPtrPhys->ConstructCollision(collision);
          // for now, we won't have a map to the collision once it's added
        });

    this->dataPtr->initialized = true;
  }
}

void Physics::Update(const std::chrono::steady_clock::duration &_dt,
                     EntityComponentManager &_ecm)
{
  (void)_dt;
  (void)_ecm;
}

void Physics::PostUpdate(const std::chrono::steady_clock::duration &_dt,
                         const EntityComponentManager &_ecm)
{
  (void)_dt;
  (void)_ecm;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System)
