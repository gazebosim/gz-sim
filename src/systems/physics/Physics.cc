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

#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/entity.pb.h>

#include <iostream>
#include <deque>
#include <unordered_map>

#include <ignition/common/Profiler.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/RelativeQuantity.hh>
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
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/SphereShape.hh>
#include <ignition/physics/mesh/MeshShape.hh>
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
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
// Components
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

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
          ignition::physics::GetContactsFromLastStepFeature,
          ignition::physics::RemoveEntities,
          ignition::physics::mesh::AttachMeshShapeFeature,
          ignition::physics::SetBasicJointState,
          ignition::physics::sdf::ConstructSdfCollision,
          ignition::physics::sdf::ConstructSdfJoint,
          ignition::physics::sdf::ConstructSdfLink,
          ignition::physics::sdf::ConstructSdfModel,
          ignition::physics::sdf::ConstructSdfVisual,
          ignition::physics::sdf::ConstructSdfWorld
          >;


  public: using EnginePtrType = ignition::physics::EnginePtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using WorldType = ignition::physics::World<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using WorldPtrType = ignition::physics::WorldPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using ModelPtrType = ignition::physics::ModelPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using LinkPtrType = ignition::physics::LinkPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using ShapePtrType = ignition::physics::ShapePtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  public: using JointPtrType = ignition::physics::JointPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Create physics entities
  /// \param[in] _ecm Constant reference to ECM.
  public: void CreatePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Remove physics entities if they are removed from the ECM
  /// \param[in] _ecm Constant reference to ECM.
  public: void RemovePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Update physics from components
  /// \param[in] _ecm Constant reference to ECM.
  public: void UpdatePhysics(const EntityComponentManager &_ecm);

  /// \brief Step the simulationrfor each world
  /// \param[in] _dt Duration
  public: void Step(const std::chrono::steady_clock::duration &_dt);

  /// \brief Update components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateSim(EntityComponentManager &_ecm) const;

  /// \brief Update collision components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateCollisions(EntityComponentManager &_ecm) const;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, WorldPtrType> entityWorldMap;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, ModelPtrType> entityModelMap;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, LinkPtrType> entityLinkMap;

  /// \brief A map between collision entity ids in the ECM to Shape Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, ShapePtrType> entityCollisionMap;

  /// \brief A map between shape entities in ign-physics to collision entities
  /// in the ECM. This is the reverse map of entityCollisionMap.
  public: std::unordered_map<ShapePtrType, Entity> collisionEntityMap;

  /// \brief a map between joint entity ids in the ECM to Joint Entities in
  /// ign-physics
  public: std::unordered_map<Entity, JointPtrType> entityJointMap;

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
  std::unordered_set<std::string> plugins = pl.LoadLib(dartsim_plugin_LIB);
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
Physics::~Physics() = default;

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  IGN_PROFILE("Physics::Update");
  if (this->dataPtr->engine)
  {
    this->dataPtr->CreatePhysicsEntities(_ecm);
    // Only step if not paused.
    if (!_info.paused)
    {
      this->dataPtr->UpdatePhysics(_ecm);
      this->dataPtr->Step(_info.dt);
      this->dataPtr->UpdateSim(_ecm);
    }

    // Entities scheduled to be removed should be removed from physics after the
    // simulation step. Otherwise, since the to-be-removed entity still shows up
    // in the ECM::Each the UpdatePhysics and UpdateSim calls will have an error
    this->dataPtr->RemovePhysicsEntities(_ecm);
  }
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreatePhysicsEntities(const EntityComponentManager &_ecm)
{
  // Get all the new worlds
  _ecm.EachNew<components::World, components::Name, components::Gravity>(
      [&](const Entity &_entity,
        const components::World * /* _world */,
        const components::Name *_name,
        const components::Gravity *_gravity)->bool
      {
        // Check if world already exists
        if (this->entityWorldMap.find(_entity) != this->entityWorldMap.end())
        {
          ignwarn << "World entity [" << _entity
                  << "] marked as new, but it's already on the map."
                  << std::endl;
          return true;
        }

        sdf::World world;
        world.SetName(_name->Data());
        world.SetGravity(_gravity->Data());
        auto worldPtrPhys = this->engine->ConstructWorld(world);
        this->entityWorldMap.insert(std::make_pair(_entity, worldPtrPhys));

        return true;
      });

  _ecm.EachNew<components::Model, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model * /* _model */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        // Check if model already exists
        if (this->entityModelMap.find(_entity) != this->entityModelMap.end())
        {
          ignwarn << "Model entity [" << _entity
                  << "] marked as new, but it's already on the map."
                  << std::endl;
          return true;
        }

        // Check if parent world exists
        // TODO(louise): Support nested models, see
        // https://bitbucket.org/ignitionrobotics/ign-physics/issues/10
        if (this->entityWorldMap.find(_parent->Data())
            == this->entityWorldMap.end())
        {
          ignwarn << "Model's parent entity [" << _parent->Data()
                  << "] not found on world map." << std::endl;
          return true;
        }
        auto worldPtrPhys = this->entityWorldMap.at(_parent->Data());

        sdf::Model model;
        model.SetName(_name->Data());
        model.SetPose(_pose->Data());

        auto staticComp = _ecm.Component<components::Static>(_entity);
        if (staticComp && staticComp->Data())
        {
          model.SetStatic(staticComp->Data());
        }

        auto modelPtrPhys = worldPtrPhys->ConstructModel(model);
        this->entityModelMap.insert(std::make_pair(_entity, modelPtrPhys));

        return true;
      });

  _ecm.EachNew<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Link * /* _link */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        // Check if link already exists
        if (this->entityLinkMap.find(_entity) != this->entityLinkMap.end())
        {
          ignwarn << "Link entity [" << _entity
                  << "] marked as new, but it's already on the map."
                  << std::endl;
          return true;
        }

        // Check if parent model exists
        if (this->entityModelMap.find(_parent->Data())
            == this->entityModelMap.end())
        {
          ignwarn << "Link's parent entity [" << _parent->Data()
                  << "] not found on model map." << std::endl;
          return true;
        }
        auto modelPtrPhys = this->entityModelMap.at(_parent->Data());

        sdf::Link link;
        link.SetName(_name->Data());
        link.SetPose(_pose->Data());

        // get link inertial
        auto inertial = _ecm.Component<components::Inertial>(_entity);
        if (inertial)
        {
          link.SetInertial(inertial->Data());
        }

        auto linkPtrPhys = modelPtrPhys->ConstructLink(link);
        this->entityLinkMap.insert(std::make_pair(_entity, linkPtrPhys));

        return true;
      });

  // We don't need to add visuals to the physics engine.

  // collisions
  _ecm.EachNew<components::Collision, components::Name, components::Pose,
            components::Geometry, components::CollisionElement,
            components::ParentEntity>(
      [&](const Entity &  _entity,
          const components::Collision * /* _collision */,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::Geometry *_geom,
          const components::CollisionElement *_collElement,
          const components::ParentEntity *_parent) -> bool
      {
        if (this->entityCollisionMap.find(_entity) !=
            this->entityCollisionMap.end())
        {
           ignwarn << "Collision entity [" << _entity
                   << "] marked as new, but it's already on the map."
                   << std::endl;
          return true;
        }

        // Check if parent link exists
        if (this->entityLinkMap.find(_parent->Data())
            == this->entityLinkMap.end())
        {
          ignwarn << "Collision's parent entity [" << _parent->Data()
                  << "] not found on link map." << std::endl;
          return true;
        }
        auto linkPtrPhys = this->entityLinkMap.at(_parent->Data());

        sdf::Collision collision;
        collision.Load(_collElement->Data());

        ShapePtrType collisionPtrPhys;
        if (_geom->Data().Type() == sdf::GeometryType::MESH)
        {
          const sdf::Mesh *meshSdf = _geom->Data().MeshShape();
          if (nullptr == meshSdf)
          {
            ignwarn << "Mesh geometry for collision [" << _name->Data()
                    << "] missing mesh shape." << std::endl;
            return true;
          }

          auto &meshManager = *ignition::common::MeshManager::Instance();
          auto *mesh = meshManager.Load(meshSdf->Uri());
          if (nullptr == mesh)
          {
            ignwarn << "Failed to load mesh from [" << meshSdf->Uri()
                    << "]." << std::endl;
            return true;
          }

          collisionPtrPhys = linkPtrPhys->AttachMeshShape(_name->Data(), *mesh,
              ignition::math::eigen3::convert(_pose->Data()),
              ignition::math::eigen3::convert(meshSdf->Scale()));
        }
        else
        {
          collisionPtrPhys = linkPtrPhys->ConstructCollision(collision);
        }

        this->entityCollisionMap.insert(
            std::make_pair(_entity, collisionPtrPhys));
        this->collisionEntityMap.insert(
            std::make_pair(collisionPtrPhys, _entity));
        return true;
      });

  // joints
  _ecm.EachNew<components::Joint, components::Name, components::JointType,
               components::Pose, components::ThreadPitch,
               components::ParentEntity, components::ParentLinkName,
               components::ChildLinkName>(
      [&](const Entity &_entity,
          const components::Joint * /* _joint */,
          const components::Name *_name,
          const components::JointType *_jointType,
          const components::Pose *_pose,
          const components::ThreadPitch *_threadPitch,
          const components::ParentEntity *_parentModel,
          const components::ParentLinkName *_parentLinkName,
          const components::ChildLinkName *_childLinkName) -> bool
      {
        // Check if joint already exists
        if (this->entityJointMap.find(_entity) != this->entityJointMap.end())
        {
          ignwarn << "Joint entity [" << _entity
                  << "] marked as new, but it's already on the map."
                  << std::endl;
          return true;
        }

        // Check if parent model exists
        if (this->entityModelMap.find(_parentModel->Data())
            == this->entityModelMap.end())
        {
          ignwarn << "Joint's parent entity [" << _parentModel->Data()
                  << "] not found on model map." << std::endl;
          return true;
        }
        auto modelPtrPhys = this->entityModelMap.at(_parentModel->Data());

        sdf::Joint joint;
        joint.SetName(_name->Data());
        joint.SetType(_jointType->Data());
        joint.SetPose(_pose->Data());
        joint.SetThreadPitch(_threadPitch->Data());

        joint.SetParentLinkName(_parentLinkName->Data());
        joint.SetChildLinkName(_childLinkName->Data());

        auto jointAxis = _ecm.Component<components::JointAxis>(_entity);
        auto jointAxis2 = _ecm.Component<components::JointAxis2>(_entity);

        if (jointAxis)
          joint.SetAxis(0, jointAxis->Data());
        if (jointAxis2)
          joint.SetAxis(1, jointAxis2->Data());

        // Use the parent link's parent model as the model of this joint
        auto jointPtrPhys = modelPtrPhys->ConstructJoint(joint);

        this->entityJointMap.insert(std::make_pair(_entity, jointPtrPhys));
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::RemovePhysicsEntities(const EntityComponentManager &_ecm)
{
  // Assume the world will not be erased
  // Only removing models is supported by ign-physics right now so we only
  // remove links, joints and collisions if they are children of the removed
  // model.
  // We assume the links, joints and collisions will be removed from the
  // physics engine when the containing model gets removed so, here, we only
  // remove the entities from the gazebo entity->physics entity map.
  _ecm.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *
          /* _model */) -> bool
      {
        // Remove model if found
        auto modelIt = this->entityModelMap.find(_entity);
        if (modelIt != this->entityModelMap.end())
        {
          // Remove child links, collisions and joints first
          for (const auto &childLink :
               _ecm.ChildrenByComponents(_entity, components::Link()))
          {
            for (const auto &childCollision :
                 _ecm.ChildrenByComponents(childLink, components::Collision()))
            {
              auto collIt = this->entityCollisionMap.find(childCollision);
              if (collIt != this->entityCollisionMap.end())
              {
                this->collisionEntityMap.erase(collIt->second);
                this->entityCollisionMap.erase(collIt);
              }
            }
            this->entityLinkMap.erase(childLink);
          }

          for (const auto &childJoint :
               _ecm.ChildrenByComponents(_entity, components::Joint()))
          {
            this->entityJointMap.erase(childJoint);
          }

          // Remove the model from the physics engine
          modelIt->second->Remove();
          this->entityModelMap.erase(_entity);
        }
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdatePhysics(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("PhysicsPrivate::UpdatePhysics");
  // Handle joint state
  _ecm.Each<components::Joint>(
      [&](const Entity &_entity, const components::Joint *)
      {
        auto jointIt = this->entityJointMap.find(_entity);
        if (jointIt == this->entityJointMap.end())
          return true;

        auto vel1 = _ecm.Component<components::JointVelocity>(_entity);
        if (vel1)
          jointIt->second->SetVelocity(0, vel1->Data());

        auto vel2 = _ecm.Component<components::JointVelocity2>(_entity);
        if (vel2)
          jointIt->second->SetVelocity(1, vel2->Data());

        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::Step(const std::chrono::steady_clock::duration &_dt)
{
  IGN_PROFILE("PhysicsPrivate::Step");
  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

  input.Get<std::chrono::steady_clock::duration>() = _dt;

  for (auto &world : this->entityWorldMap)
  {
    world.second->Step(output, state, input);
  }
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateSim(EntityComponentManager &_ecm) const
{
  IGN_PROFILE("PhysicsPrivate::UpdateSim");
  _ecm.Each<components::Link, components::Pose, components::ParentEntity>(
      [&](const Entity &_entity, components::Link * /*_link*/,
          components::Pose *_pose, components::ParentEntity *_parent)->bool
      {
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          auto canonicalLink =
              _ecm.Component<components::CanonicalLink>(_entity);

          // get the pose component of the parent model
          auto parentPose =
              _ecm.Component<components::Pose>(_parent->Data());

          auto worldPose = linkIt->second->FrameDataRelativeToWorld().pose;

          // if the parentPose is a nullptr, something is wrong with ECS
          // creation
          if (!parentPose)
          {
            ignerr << "The pose component of " << _parent->Data()
                   << " could not be found. This should never happen!\n";
            return true;
          }
          if (canonicalLink)
          {
            // This is the canonical link, update the model
            // The Pose component, _pose, of this link is the initial
            // transform of the link w.r.t its model. This component never
            // changes because it's "fixed" to the model. Instead, we change
            // the model's pose here. The physics engine gives us the pose of
            // this link relative to world so to set the model's pose, we have
            // to premultiply it by the inverse of the initial transform of
            // the link w.r.t to its model.
            *parentPose = components::Pose(_pose->Data().Inverse() +
                                           math::eigen3::convert(worldPose));
          }
          else
          {
            // Compute the relative pose of this link from the model
            *_pose = components::Pose(math::eigen3::convert(worldPose) +
                                      parentPose->Data().Inverse());
          }
        }
        else
        {
          ignwarn << "Unknown link with id " << _entity << " found\n";
        }
        return true;
      });


  // world pose
  _ecm.Each<components::Pose, components::WorldPose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          components::Pose *_pose, components::WorldPose *_worldPose,
          components::ParentEntity *_parent)->bool
      {
        // check if entity is a link
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          auto frameDataWorld = linkIt->second->FrameDataRelativeToWorld();
          *_worldPose = components::WorldPose(
              math::eigen3::convert(frameDataWorld.pose));
          return true;
        }

        // check if parent entity is a link, e.g. entity is sensor / collision
        linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          auto frameDataWorld = linkIt->second->FrameDataRelativeToWorld();
          auto linkWorldPose = frameDataWorld.pose;
          auto entityWorldPose = _pose->Data() +
              math::eigen3::convert(linkWorldPose);
          *_worldPose = components::WorldPose(entityWorldPose);
          return true;
        }

        return true;
      });

  // world linear velocity
  _ecm.Each<components::Pose, components::WorldLinearVelocity,
            components::ParentEntity>(
      [&](const Entity &_entity,
          components::Pose *_pose,
          components::WorldLinearVelocity *_worldLinearVel,
          components::ParentEntity *_parent)->bool
      {
        // check if entity is a link
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          auto frameDataWorld = linkIt->second->FrameDataRelativeToWorld();
          *_worldLinearVel = components::WorldLinearVelocity(
              math::eigen3::convert(frameDataWorld.linearVelocity));
          return true;
        }

        // check if parent entity is a link, e.g. entity is sensor / collision
        linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          // offset is entity pos relative to parent link
          physics::FrameData3d entityFromLink;
          math::Vector3d offset = _pose->Data().Pos();
          entityFromLink.pose.translation() = math::eigen3::convert(offset);
          entityFromLink.pose.linear() =
              math::eigen3::convert(math::Matrix3d::Identity);

          physics::RelativeFrameData3d relFrameData(
              linkIt->second->GetFrameID(), entityFromLink);
          auto entityFrameData =
              this->engine->Resolve(relFrameData, physics::FrameID::World());

          // set entity world linear velocity
          *_worldLinearVel = components::WorldLinearVelocity(
              math::eigen3::convert(entityFrameData.linearVelocity));

          return true;
        }

        return true;
      });

  // body angular velocity
  _ecm.Each<components::Pose, components::AngularVelocity,
            components::ParentEntity>(
      [&](const Entity &_entity,
          components::Pose *_pose,
          components::AngularVelocity *_angularVel,
          components::ParentEntity *_parent)->bool
      {
        physics::FrameData3d entityFrameData;
        ignition::math::Pose3d entityWorldPose;

        // check if entity is a link
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          entityFrameData = linkIt->second->FrameDataRelativeToWorld();
          entityWorldPose = math::eigen3::convert(entityFrameData.pose);
        }
        else
        {
          // check if parent entity is a link, e.g. entity is sensor / collision
          linkIt = this->entityLinkMap.find(_parent->Data());
          if (linkIt != this->entityLinkMap.end())
          {
            // offset is entity pos relative to parent link
            physics::FrameData3d entityFromLink;
            math::Vector3d offset = _pose->Data().Pos();
            entityFromLink.pose.translation() = math::eigen3::convert(offset);
            entityFromLink.pose.linear() =
                math::eigen3::convert(math::Matrix3d::Identity);

            physics::RelativeFrameData3d relFrameData(
                linkIt->second->GetFrameID(), entityFromLink);
            entityFrameData =
                this->engine->Resolve(relFrameData, physics::FrameID::World());
            entityWorldPose = _pose->Data() +
                math::eigen3::convert(entityFrameData.pose);
          }
        }

        if (linkIt != this->entityLinkMap.end())
        {
          ignition::math::Vector3d entityWorldAngularVel =
            math::eigen3::convert(entityFrameData.angularVelocity);

          auto entityBodyAngularVel =
            _pose->Data().Rot().Inverse().RotateVector(
              entityWorldAngularVel);
          *_angularVel = components::AngularVelocity(entityBodyAngularVel);
        }

        return true;
      });

  // body linear acceleration
  _ecm.Each<components::Pose, components::LinearAcceleration,
            components::ParentEntity>(
      [&](const Entity &_entity,
          components::Pose *_pose,
          components::LinearAcceleration *_linearAcc,
          components::ParentEntity *_parent)->bool
      {
        physics::FrameData3d entityFrameData;
        ignition::math::Pose3d entityWorldPose;

        // check if entity is a link
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          entityFrameData = linkIt->second->FrameDataRelativeToWorld();
          entityWorldPose = math::eigen3::convert(entityFrameData.pose);
        }
        else
        {
          // check if parent entity is a link, e.g. entity is sensor / collision
          linkIt = this->entityLinkMap.find(_parent->Data());
          if (linkIt != this->entityLinkMap.end())
          {
            // offset is entity pos relative to parent link
            physics::FrameData3d entityFromLink;
            math::Vector3d offset = _pose->Data().Pos();
            entityFromLink.pose.translation() = math::eigen3::convert(offset);
            entityFromLink.pose.linear() =
                math::eigen3::convert(math::Matrix3d::Identity);

            physics::RelativeFrameData3d relFrameData(
                linkIt->second->GetFrameID(), entityFromLink);
            entityFrameData =
                this->engine->Resolve(relFrameData, physics::FrameID::World());
            entityWorldPose = _pose->Data() +
                math::eigen3::convert(entityFrameData.pose);
          }
        }

        if (linkIt != this->entityLinkMap.end())
        {
          ignition::math::Vector3d entityWorldLinearAcc = math::eigen3::convert(
              entityFrameData.linearAcceleration);

          auto entityBodyLinearAcc = _pose->Data().Rot().Inverse().RotateVector(
              entityWorldLinearAcc);
          *_linearAcc = components::LinearAcceleration(entityBodyLinearAcc);
        }

        return true;
      });

  this->UpdateCollisions(_ecm);
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateCollisions(EntityComponentManager &_ecm) const
{
  IGN_PROFILE("PhysicsPrivate::UpdateCollisions");
  // Quit early if the ContactData component hasn't been created. This means
  // there are no systems that need contact information
  if (!_ecm.HasComponentType(components::ContactSensorData::typeId))
    return;

  // TODO(addisu) If systems are assumed to only have one world, we should
  // capture the world Entity in a Configure call
  Entity worldEntity = _ecm.EntityByComponents(components::World());

  if (worldEntity == kNullEntity)
  {
    ignerr << "Missing world entity.\n";
    return;
  }

  // Safe to assume this won't throw because the world entity should always be
  // available
  auto worldPhys = this->entityWorldMap.at(worldEntity);

  // Each contact object we get from ign-physics contains the EntityPtrs of the
  // two colliding entities and other data about the contact such as the
  // position. This map groups contacts so that it is easy to query all the
  // contacts of one entity.
  using EntityContactMap =
      std::unordered_map<Entity, std::deque<const WorldType::ContactPoint *>>;

  // This data structure is essentially a mapping between a pair of entities and
  // a list of pointers to their contact object. We use a map inside a map to
  // create msgs::Contact objects conveniently later on.
  std::unordered_map<Entity, EntityContactMap> entityContactMap;

  // Note that we are temporarily storing pointers to elements in this
  // ("allContacts") container. Thus, we must make sure it doesn't get destroyed
  // until the end of this function.
  auto allContacts = worldPhys->GetContactsFromLastStep();
  for (const auto &contactComposite : allContacts)
  {
    const auto &contact = contactComposite.Get<WorldType::ContactPoint>();
    auto coll1It = this->collisionEntityMap.find(contact.collision1);
    auto coll2It = this->collisionEntityMap.find(contact.collision2);

    if ((coll1It != this->collisionEntityMap.end()) &&
        (coll2It != this->collisionEntityMap.end()))
    {
      entityContactMap[coll1It->second][coll2It->second].push_back(&contact);
      entityContactMap[coll2It->second][coll1It->second].push_back(&contact);
    }
  }

  // Go through each collision entity that has a ContactData component and
  // set the component value to the list of contacts that correspond to
  // the collision entity
  _ecm.Each<components::Collision, components::ContactSensorData>(
      [&](const Entity &_collEntity1, components::Collision *,
          components::ContactSensorData *_contacts) -> bool
      {
        if (entityContactMap.find(_collEntity1) == entityContactMap.end())
        {
          // Clear the last contact data
          *_contacts = components::ContactSensorData();
          return true;
        }

        const auto &contactMap = entityContactMap[_collEntity1];

        msgs::Contacts contactsComp;

        for (const auto &[collEntity2, contactData] : contactMap)
        {
          msgs::Contact *contactMsg = contactsComp.add_contact();
          contactMsg->mutable_collision1()->set_id(_collEntity1);
          contactMsg->mutable_collision2()->set_id(collEntity2);
          for (const auto &contact : contactData)
          {
            auto *position = contactMsg->add_position();
            position->set_x(contact->point.x());
            position->set_y(contact->point.y());
            position->set_z(contact->point.z());
          }
        }
        *_contacts = components::ContactSensorData(std::move(contactsComp));

        return true;
      });
}

IGNITION_ADD_PLUGIN(Physics,
                    ignition::gazebo::System,
                    Physics::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Physics, "ignition::gazebo::systems::Physics")
