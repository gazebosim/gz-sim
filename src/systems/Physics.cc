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
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"

#include "ignition/gazebo/systems/Physics.hh"

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

  /// \brief Create physics entities
  public: void CreatePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Step the simulationrfor each world
  public: void Step(const std::chrono::steady_clock::duration &_dt);

  /// \brief Step the simulationrfor each world
  public: void UpdateECS(EntityComponentManager &_ecm) const;

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

  /// \brief a map between visual entity ids in the ECM to thier initial offset
  /// from their parent link
  public: std::unordered_map<EntityId, math::Pose3d> visualOffsetMap;

  /// \brief a map between collision entity ids in the ECM to thier initial
  /// offset from their parent link
  public: std::unordered_map<EntityId, math::Pose3d> collisionOffsetMap;

  /// \brief used to store whether physics objects have been created
  public: bool initialized = false;

  /// \brief pointer to the underlying ign-physics Engine entity
  public: EnginePtrType engine;
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
  ignition::plugin::Loader pl;
  // dartsim_plugin_LIB is defined by cmake
  auto plugins = pl.LoadLibrary(dartsim_plugin_LIB);
  const std::string className = "ignition::physics::dartsim::Plugin";
  ignition::plugin::PluginPtr plugin = pl.Instantiate(className);

  this->dataPtr->engine = ignition::physics::RequestEngine<
      ignition::physics::FeaturePolicy3d,
      PhysicsPrivate::MinimumFeatureList>::
      From(plugin);
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->CreatePhysicsEntities(_ecm);
    this->dataPtr->initialized = true;
  }

  this->dataPtr->Step(_info.dt);
  this->dataPtr->UpdateECS(_ecm);
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
    // get worlds
  _ecm.Each<components::World, components::Name>(
      [&](const EntityId &_entity,
        const components::World * /* _world */,
        const components::Name *_name)
      {
        if (this->entityWorldMap.find(_entity) == this->entityWorldMap.end())
        {
          sdf::World world;
          world.SetName(_name->Data());
          auto worldPtrPhys = this->engine->ConstructWorld(world);
          this->entityWorldMap.insert(std::make_pair(_entity, worldPtrPhys));
        }
      });

  _ecm.Each<components::Model, components::Name, components::Pose,
            components::ParentEntity, components::Static>(
      [&](const EntityId &_entity,
        const components::Model * /* _model */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Static *_static)
      {
        if (this->entityModelMap.find(_entity) == this->entityModelMap.end())
        {
          sdf::Model model;
          model.SetName(_name->Data());
          model.SetPose(_pose->Data());
          model.SetStatic(_static->Data());
          auto worldPtrPhys = this->entityWorldMap.at(_parent->Id());
          auto modelPtrPhys = worldPtrPhys->ConstructModel(model);
          this->entityModelMap.insert(std::make_pair(_entity, modelPtrPhys));
        }
      });

  _ecm.Each<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const EntityId &_entity,
        const components::Link * /* _link */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)
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

          auto modelPtrPhys = this->entityModelMap.at(_parent->Id());
          auto linkPtrPhys = modelPtrPhys->ConstructLink(link);
          this->entityLinkMap.insert(std::make_pair(_entity, linkPtrPhys));
        }
      });

  // visuals
  _ecm.Each<components::Visual,  components::Pose>(
      [&](const EntityId  &_entity,
        const components::Visual * /* _visual */,
        const components::Pose *_pose)
      {
        // We don't need to add visuals to the physics engine, but we need to
        // save their relative poses so we can transform them into world poses
        // once the simulation is running.
        visualOffsetMap.insert(std::make_pair(_entity, _pose->Data()));
      });

  // collisions
  _ecm.Each<components::Collision, components::Name, components::Pose,
            components::Geometry, components::ParentEntity>(
      [&](const EntityId & _entity,
        const components::Collision * /* _collision */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::Geometry *_geom,
        const components::ParentEntity *_parent)
      {
        sdf::Collision collision;
        collision.SetName(_name->Data());
        collision.SetPose(_pose->Data());
        collision.SetGeom(_geom->Data());
        auto linkPtrPhys = this->entityLinkMap.at(_parent->Id());
        linkPtrPhys->ConstructCollision(collision);
        // for now, we won't have a map to the collision once it's added
        collisionOffsetMap.insert(std::make_pair(_entity, _pose->Data()));
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
void PhysicsPrivate::UpdateECS(EntityComponentManager &_ecm) const
{
  _ecm.EachMutable<components::Link, components::Pose>(
      [&](const EntityId &_entity, components::Link * /*_link*/,
          components::Pose *_pose)
      {
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          math::Pose3d pose = math::eigen3::convert(
              linkIt->second->FrameDataRelativeToWorld().pose);
          *_pose = components::Pose(pose);
        }
        else
        {
          ignwarn << "Unknown link with id " << _entity << " found\n";
        }
      });

  _ecm.EachMutable<components::Visual, components::Pose,
                   components::ParentEntity>(
      [&](const EntityId &_entity, components::Visual * /*_visual*/,
          components::Pose *_pose, components::ParentEntity *_parent)
      {
        auto linkIt = this->entityLinkMap.find(_parent->Id());
        if (linkIt != this->entityLinkMap.end())
        {
          math::Pose3d parentPose = math::eigen3::convert(
              linkIt->second->FrameDataRelativeToWorld().pose);

          auto offsetIt = this->visualOffsetMap.find(_entity);
          if (offsetIt != this->visualOffsetMap.end())
          {
            *_pose = components::Pose(parentPose * offsetIt->second);
          }
        }
        else
        {
          ignwarn << "Visual with id " << _entity
                  << " does not have a valid parent link\n";
        }
      });

  _ecm.EachMutable<components::Collision, components::Pose,
                   components::ParentEntity>(
      [&](const EntityId &_entity, components::Collision * /*_collision*/,
          components::Pose *_pose, components::ParentEntity *_parent)
      {
        auto linkIt = this->entityLinkMap.find(_parent->Id());
        if (linkIt != this->entityLinkMap.end())
        {
          math::Pose3d parentPose = math::eigen3::convert(
              linkIt->second->FrameDataRelativeToWorld().pose);

          auto offsetIt = this->collisionOffsetMap.find(_entity);
          if (offsetIt != this->collisionOffsetMap.end())
          {
            *_pose = components::Pose(parentPose * offsetIt->second);
          }
        }
        else
        {
          ignwarn << "Collision with id " << _entity
                  << " does not have a valid parent link\n";
        }
      });
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System)
