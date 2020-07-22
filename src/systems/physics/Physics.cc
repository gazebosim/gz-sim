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
#include <ignition/msgs/Utility.hh>

#include <iostream>
#include <deque>
#include <unordered_map>

#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/physics/config.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/FindFeatures.hh>
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
#include <ignition/physics/FreeGroup.hh>
#include <ignition/physics/FixedJoint.hh>
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/SphereShape.hh>
#include <ignition/physics/mesh/MeshShape.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

// SDF
#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Surface.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

// Components
#include "ignition/gazebo/components/AngularAcceleration.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/AxisAlignedBox.hh"
#include "ignition/gazebo/components/BatterySoC.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointVelocityReset.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/PhysicsEnginePlugin.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/SelfCollide.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/World.hh"

#include "Physics.hh"

using namespace ignition;
using namespace ignition::gazebo::systems;
namespace components = ignition::gazebo::components;


// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
  /// \brief This is the minimum set of features that any physics engine must
  /// implement to be supported by this system.
  /// New features can't be added to this list in minor / patch releases, in
  /// order to maintain backwards compatibility with downstream physics plugins.
  public: struct MinimumFeatureList : ignition::physics::FeatureList<
          ignition::physics::FindFreeGroupFeature,
          ignition::physics::SetFreeGroupWorldPose,
          ignition::physics::FreeGroupFrameSemantics,
          ignition::physics::LinkFrameSemantics,
          ignition::physics::ForwardStep,
          ignition::physics::RemoveEntities,
          ignition::physics::sdf::ConstructSdfLink,
          ignition::physics::sdf::ConstructSdfModel,
          ignition::physics::sdf::ConstructSdfWorld
          >{};

  /// \brief Engine type with just the minimum features.
  public: using EnginePtrType = ignition::physics::EnginePtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief World type with just the minimum features.
  public: using WorldPtrType = ignition::physics::WorldPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Model type with just the minimum features.
  public: using ModelPtrType = ignition::physics::ModelPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Link type with just the minimum features.
  public: using LinkPtrType = ignition::physics::LinkPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Free group type with just the minimum features.
  public: using FreeGroupPtrType = ignition::physics::FreeGroupPtr<
            ignition::physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Create physics entities
  /// \param[in] _ecm Constant reference to ECM.
  public: void CreatePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Remove physics entities if they are removed from the ECM
  /// \param[in] _ecm Constant reference to ECM.
  public: void RemovePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Update physics from components
  /// \param[in] _ecm Constant reference to ECM.
  public: void UpdatePhysics(EntityComponentManager &_ecm);

  /// \brief Step the simulationrfor each world
  /// \param[in] _dt Duration
  public: void Step(const std::chrono::steady_clock::duration &_dt);

  /// \brief Update components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateSim(EntityComponentManager &_ecm);

  /// \brief Update collision components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateCollisions(EntityComponentManager &_ecm);

  /// \brief FrameData relative to world at a given offset pose
  /// \param[in] _link ign-physics link
  /// \param[in] _pose Offset pose in which to compute the frame data
  /// \returns FrameData at the given offset pose
  public: physics::FrameData3d LinkFrameDataAtOffset(
      const LinkPtrType &_link, const math::Pose3d &_pose) const;

  /// \brief Get the top level model of an entity
  /// \param[in] _entity Input entity
  /// \param[in] _ecm Constant reference to ECM.
  /// \return Entity of top level model
  public: ignition::gazebo::Entity TopLevelModel(const Entity &_entity,
      const EntityComponentManager &_ecm) const;

  /// \brief Get transform from one ancestor entity to a descendant entity
  /// that are in the same model.
  /// \param[in] _from An ancestor of the _to entity.
  /// \param[in] _to A descendant of the _from entity.
  /// \return Pose transform between the two entities
  public: ignition::math::Pose3d RelativePose(const Entity &_from,
      const Entity &_to, const EntityComponentManager &_ecm) const;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, WorldPtrType> entityWorldMap;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, ModelPtrType> entityModelMap;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, LinkPtrType> entityLinkMap;

  /// \brief Reverse of entityLinkMap. This is used for finding the Entity
  /// associated with a physics Link
  public: std::unordered_map<LinkPtrType, Entity> linkEntityMap;

  /// \brief A map between model entity ids in the ECM to whether its battery
  /// has drained.
  public: std::unordered_map<Entity, bool> entityOffMap;

  /// \brief used to store whether physics objects have been created.
  public: bool initialized = false;

  /// \brief Pointer to the underlying ign-physics Engine entity.
  public: EnginePtrType engine = nullptr;

  /// \brief Vector3d equality comparison function.
  public: std::function<bool(const math::Vector3d &, const math::Vector3d &)>
          vec3Eql { [](const math::Vector3d &_a, const math::Vector3d &_b)
                    {
                      return _a.Equal(_b, 1e-6);
                    }};

  /// \brief Pose3d equality comparison function.
  public: std::function<bool(const math::Pose3d &, const math::Pose3d &)>
          pose3Eql { [](const math::Pose3d &_a, const math::Pose3d &_b)
                     {
                       return _a.Pos().Equal(_b.Pos(), 1e-6) &&
                         math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
                         math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
                         math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
                         math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
                     }};

  /// \brief AxisAlignedBox equality comparison function.
  public: std::function<bool(const math::AxisAlignedBox &,
          const math::AxisAlignedBox&)>
          axisAlignedBoxEql { [](const math::AxisAlignedBox &_a,
                                 const math::AxisAlignedBox &_b)
                     {
                       return _a == _b;
                     }};

  /// \brief Environment variable which holds paths to look for engine plugins
  public: std::string pluginPathEnv = "IGN_GAZEBO_PHYSICS_ENGINE_PATH";

  //////////////////////////////////////////////////
  // Joints

  /// \brief Feature list to handle joints.
  public: using JointFeatureList = ignition::physics::FeatureList<
            MinimumFeatureList,
            ignition::physics::GetBasicJointProperties,
            ignition::physics::GetBasicJointState,
            ignition::physics::SetBasicJointState,
            ignition::physics::sdf::ConstructSdfJoint>;

  /// \brief Joint type with joint features.
  public: using JointPtrType = ignition::physics::JointPtr<
            ignition::physics::FeaturePolicy3d, JointFeatureList>;

  /// \brief Model type with joint features (models to attach to).
  public: using ModelJointPtrType = ignition::physics::ModelPtr<
            ignition::physics::FeaturePolicy3d, JointFeatureList>;

  /// \brief A map between joint entity ids in the ECM to Joint Entities in
  /// ign-physics
  public: std::unordered_map<Entity, JointPtrType> entityJointMap;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// ign-physics, with attach feature.
  /// All models on this map are also in `entityModelMap`. The difference is
  /// that here they've been casted for `JointFeatureList`.
  public: std::unordered_map<Entity, ModelJointPtrType> entityModelJointMap;

  //////////////////////////////////////////////////
  // Detachable joints

  /// \brief Feature list to process `DetachableJoint` components.
  public: using DetachableJointFeatureList = physics::FeatureList<
            JointFeatureList,
            physics::AttachFixedJointFeature,
            physics::DetachJointFeature,
            physics::SetJointTransformFromParentFeature>;

  /// \brief Joint type with detachable joint features.
  public: using JointDetachableJointPtrType = physics::JointPtr<
            physics::FeaturePolicy3d, DetachableJointFeatureList>;

  /// \brief Link type with detachable joint features (links to attach to).
  public: using LinkDetachableJointPtrType = physics::LinkPtr<
            physics::FeaturePolicy3d, DetachableJointFeatureList>;

  /// \brief A map between joint entity ids in the ECM to Joint Entities in
  /// ign-physics, with detach feature.
  /// All joints on this map are also in `entityJointMap`. The difference is
  /// that here they've been casted for `physics::DetachJointFeature`.
  public: std::unordered_map<Entity, JointDetachableJointPtrType>
      entityJointDetachableJointMap;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics, with attach feature.
  /// All links on this map are also in `entityLinkMap`. The difference is
  /// that here they've been casted for `DetachableJointFeatureList`.
  public: std::unordered_map<Entity, LinkDetachableJointPtrType>
      entityLinkDetachableJointMap;

  //////////////////////////////////////////////////
  // Collisions

  /// \brief Feature list to handle collisions.
  public: using CollisionFeatureList = ignition::physics::FeatureList<
            MinimumFeatureList,
            ignition::physics::GetContactsFromLastStepFeature,
            ignition::physics::sdf::ConstructSdfCollision>;

  /// \brief Collision type with collision features.
  public: using ShapePtrType = ignition::physics::ShapePtr<
            ignition::physics::FeaturePolicy3d, CollisionFeatureList>;

  /// \brief Link type with collision features.
  public: using LinkShapePtrType = ignition::physics::LinkPtr<
            ignition::physics::FeaturePolicy3d, CollisionFeatureList>;

  /// \brief World type with collision features.
  public: using WorldShapePtrType = ignition::physics::WorldPtr<
            ignition::physics::FeaturePolicy3d, CollisionFeatureList>;

  /// \brief World type with just the minimum features. Non-pointer.
  public: using WorldShapeType = ignition::physics::World<
            ignition::physics::FeaturePolicy3d, CollisionFeatureList>;

  /// \brief A map between collision entity ids in the ECM to Shape Entities in
  /// ign-physics.
  public: std::unordered_map<Entity, ShapePtrType> entityCollisionMap;

  /// \brief A map between shape entities in ign-physics to collision entities
  /// in the ECM. This is the reverse map of entityCollisionMap.
  public: std::unordered_map<ShapePtrType, Entity> collisionEntityMap;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics, with attach feature.
  /// All links on this map are also in `entityLinkMap`. The difference is
  /// that here they've been casted for `CollisionFeatureList`.
  public: std::unordered_map<Entity, LinkShapePtrType> entityLinkCollisionMap;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// ign-physics, with attach feature.
  /// All worlds on this map are also in `entityWorldMap`. The difference is
  /// that here they've been casted for `CollisionFeatureList`.
  public: std::unordered_map<Entity, WorldShapePtrType> entityWorldCollisionMap;

  //////////////////////////////////////////////////
  // Collision filtering with bitmasks

  /// \brief Feature list to filter collisions with bitmasks.
  public: using CollisionMaskFeatureList = ignition::physics::FeatureList<
          CollisionFeatureList,
          ignition::physics::CollisionFilterMaskFeature>;

  /// \brief Collision type with collision filtering features.
  public: using ShapeFilterMaskPtrType = ignition::physics::ShapePtr<
            ignition::physics::FeaturePolicy3d, CollisionMaskFeatureList>;

  /// \brief A map between collision entity ids in the ECM to Shape Entities in
  /// ign-physics, with collision filtering feature.
  /// All links on this map are also in `entityCollisionMap`. The difference is
  /// that here they've been casted for `CollisionMaskFeatureList`.
  public: std::unordered_map<Entity, ShapeFilterMaskPtrType> entityShapeMaskMap;

  //////////////////////////////////////////////////
  // Link force

  /// \brief Feature list for applying forces to links.
  public: using LinkForceFeatureList = ignition::physics::FeatureList<
            ignition::physics::AddLinkExternalForceTorque>;

  /// \brief Link type with bounding box feature.
  public: using LinkForcePtrType = ignition::physics::LinkPtr<
            ignition::physics::FeaturePolicy3d, LinkForceFeatureList>;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics, with force feature.
  /// All links on this map are also in `entityLinkMap`. The difference is
  /// that here they've been casted for `LinkForceFeatureList`.
  public: std::unordered_map<Entity, LinkForcePtrType> entityLinkForceMap;

  //////////////////////////////////////////////////
  // Bounding box

  /// \brief Feature list for model bounding box.
  public: using BoundingBoxFeatureList = ignition::physics::FeatureList<
            MinimumFeatureList,
            ignition::physics::GetModelBoundingBox>;

  /// \brief Model type with bounding box feature.
  public: using ModelBoundingBoxPtrType = ignition::physics::ModelPtr<
            ignition::physics::FeaturePolicy3d, BoundingBoxFeatureList>;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// ign-physics, with bounding box feature.
  /// All models on this map are also in `entityModelMap`. The difference is
  /// that here they've been casted for `BoundingBoxFeatureList`.
  public: std::unordered_map<Entity, ModelBoundingBoxPtrType>
      entityModelBoundingBoxMap;

  //////////////////////////////////////////////////
  // Joint velocity command

  /// \brief Feature list for set joint velocity command.
  public: using JointVelocityCommandFeatureList = physics::FeatureList<
            physics::SetJointVelocityCommandFeature>;

  /// \brief Joint type with set joint velocity command.
  public: using JointVelocityCommandPtrType = physics::JointPtr<
            physics::FeaturePolicy3d, JointVelocityCommandFeatureList>;

  /// \brief A map between joint entity ids in the ECM to Joint Entities in
  /// ign-physics, with velocity command feature.
  /// All joints on this map are also in `entityJointMap`. The difference is
  /// that here they've been casted for `JointVelocityCommandFeatureList`.
  public: std::unordered_map<Entity, JointVelocityCommandPtrType>
      entityJointVelocityCommandMap;

  //////////////////////////////////////////////////
  // World velocity command
  public: using WorldVelocityCommandFeatureList =
            ignition::physics::FeatureList<
              ignition::physics::SetFreeGroupWorldVelocity>;

  /// \brief Free group type with world velocity command.
  public: using FreeGroupVelocityCmdPtrType =
            ignition::physics::FreeGroupPtr<
              ignition::physics::FeaturePolicy3d,
                WorldVelocityCommandFeatureList>;

  /// \brief A map between free group entity ids in the ECM
  /// to FreeGroup Entities in ign-physics, with velocity command feature.
  /// All FreeGroup on this map are casted for
  /// `WorldVelocityCommandFeatureList`.
  public: std::unordered_map<Entity, FreeGroupVelocityCmdPtrType>
      entityWorldVelocityCommandMap;

  //////////////////////////////////////////////////
  // Meshes

  /// \brief Feature list for meshes.
  /// Include MinimumFeatureList so created collision can be automatically
  /// up-cast.
  public: using MeshFeatureList = physics::FeatureList<
            CollisionFeatureList,
            physics::mesh::AttachMeshShapeFeature>;

  /// \brief Link type with meshes.
  public: using LinkMeshPtrType = physics::LinkPtr<
            physics::FeaturePolicy3d, MeshFeatureList>;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// ign-physics, with mesh feature.
  /// All links on this map are also in `entityLinkMap`. The difference is
  /// that here they've been casted for `MeshFeatureList`.
  public: std::unordered_map<Entity, LinkMeshPtrType>
      entityLinkMeshMap;
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
}

//////////////////////////////////////////////////
void Physics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  std::string pluginLib;

  // 1. Engine from component (from command line / ServerConfig)
  auto engineComp = _ecm.Component<components::PhysicsEnginePlugin>(_entity);
  if (engineComp && !engineComp->Data().empty())
  {
    pluginLib = engineComp->Data();
  }
  // 2. Engine from SDF
  else if (_sdf->HasElement("engine"))
  {
    auto sdfClone = _sdf->Clone();
    auto engineElem = sdfClone->GetElement("engine");
    pluginLib = engineElem->Get<std::string>("filename", pluginLib).first;
  }

  // 3. Use DART by default
  if (pluginLib.empty())
  {
    pluginLib = "libignition-physics-dartsim-plugin.so";
  }

  // Update component
  if (!engineComp)
  {
    _ecm.CreateComponent(_entity, components::PhysicsEnginePlugin(pluginLib));
  }
  else
  {
    engineComp->SetData(pluginLib,
        [](const std::string &_a, const std::string &_b){return _a == _b;});
  }

  // Find engine shared library
  // Look in:
  // * Paths from environment variable
  // * Engines installed with ign-physics
  common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv(this->dataPtr->pluginPathEnv);
  systemPaths.AddPluginPaths({IGNITION_PHYSICS_ENGINE_INSTALL_DIR});

  auto pathToLib = systemPaths.FindSharedLibrary(pluginLib);
  if (pathToLib.empty())
  {
    ignerr << "Failed to find plugin [" << pluginLib
           << "]. Have you checked the " << this->dataPtr->pluginPathEnv
           << " environment variable?" << std::endl;
    return;
  }

  // Load engine plugin
  ignition::plugin::Loader pluginLoader;
  auto plugins = pluginLoader.LoadLib(pathToLib);
  if (plugins.empty())
  {
    ignerr << "Unable to load the [" << pathToLib << "] library.\n";
    return;
  }

  auto classNames = pluginLoader.AllPlugins();
  if (classNames.empty())
  {
    ignerr << "No plugins found in library [" << pathToLib << "]." << std::endl;
    return;
  }

  // Get the first plugin that works
  for (auto className : classNames)
  {
    auto plugin = pluginLoader.Instantiate(className);

    if (!plugin)
      continue;

    this->dataPtr->engine = ignition::physics::RequestEngine<
      ignition::physics::FeaturePolicy3d,
      PhysicsPrivate::MinimumFeatureList>::From(plugin);

    if (nullptr != this->dataPtr->engine)
    {
      igndbg << "Loaded [" << className << "] from library ["
             << pathToLib << "]" << std::endl;
      break;
    }

    auto missingFeatures = ignition::physics::RequestEngine<
        ignition::physics::FeaturePolicy3d,
        PhysicsPrivate::MinimumFeatureList>::MissingFeatureNames(plugin);

    std::stringstream msg;
    msg << "Plugin [" << className << "] misses required features:"
        << std::endl;
    for (auto feature : missingFeatures)
    {
      msg << "- " << feature << std::endl;
    }
    ignwarn << msg.str();
  }

  if (nullptr == this->dataPtr->engine)
  {
    ignerr << "Failed to load a valid physics engine from [" << pathToLib
           << "]."
           << std::endl;
  }
}

//////////////////////////////////////////////////
Physics::~Physics() = default;

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  IGN_PROFILE("Physics::Update");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (this->dataPtr->engine)
  {
    this->dataPtr->CreatePhysicsEntities(_ecm);
    this->dataPtr->UpdatePhysics(_ecm);
    // Only step if not paused.
    if (!_info.paused)
    {
      this->dataPtr->Step(_info.dt);
    }
    this->dataPtr->UpdateSim(_ecm);

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
          const components::Model *,
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

        // TODO(anyone) Don't load models unless they have collisions

        // Check if parent world / model exists
        sdf::Model model;
        model.SetName(_name->Data());
        model.SetRawPose(_pose->Data());
        auto staticComp = _ecm.Component<components::Static>(_entity);
        if (staticComp && staticComp->Data())
        {
          model.SetStatic(staticComp->Data());
        }
        auto selfCollideComp = _ecm.Component<components::SelfCollide>(_entity);
        if (selfCollideComp && selfCollideComp ->Data())
        {
          model.SetSelfCollide(selfCollideComp->Data());
        }

        // check if parent is a world
        auto worldIt = this->entityWorldMap.find(_parent->Data());
        if (worldIt != this->entityWorldMap.end())
        {
          auto worldPtrPhys = worldIt->second;
          auto modelPtrPhys = worldPtrPhys->ConstructModel(model);
          this->entityModelMap.insert(std::make_pair(_entity, modelPtrPhys));
        }
        // check if parent is a model (nested model)
        else
        {
          auto parentIt = this->entityModelMap.find(_parent->Data());
          if (parentIt != this->entityModelMap.end())
          {
            auto parentPtrPhys = parentIt->second;
            auto modelPtrPhys = parentPtrPhys->ConstructModel(model);
            if (modelPtrPhys)
            {
              this->entityModelMap.insert(std::make_pair(_entity, modelPtrPhys));
            }
            else
            {
              ignerr << "Model: '" << _name->Data() << "' not loaded. "
                     << "Nested model not supported by physics engine. " << std::endl;
            }
          }
          else
          {
            ignwarn << "Model's parent entity [" << _parent->Data()
                    << "] not found on world / model map." << std::endl;
            return true;
          }
        }

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

        // TODO(anyone) Don't load links unless they have collisions

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
        link.SetRawPose(_pose->Data());

        // get link inertial
        auto inertial = _ecm.Component<components::Inertial>(_entity);
        if (inertial)
        {
          link.SetInertial(inertial->Data());
        }

        auto linkPtrPhys = modelPtrPhys->ConstructLink(link);
        this->entityLinkMap.insert(std::make_pair(_entity, linkPtrPhys));
        this->linkEntityMap.insert(std::make_pair(linkPtrPhys, _entity));

        return true;
      });

  // We don't need to add visuals to the physics engine.

  // collisions
  _ecm.EachNew<components::Collision, components::Name, components::Pose,
            components::Geometry, components::CollisionElement,
            components::ParentEntity>(
      [&](const Entity &  _entity,
          const components::Collision *,
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

        // Make a copy of the collision DOM so we can set its pose which has
        // been resolved and is now expressed w.r.t the parent link of the
        // collision.
        sdf::Collision collision = _collElement->Data();
        collision.SetRawPose(_pose->Data());
        collision.SetPoseRelativeTo("");
        auto collideBitmask = collision.Surface()->Contact()->CollideBitmask();

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
          auto fullPath = asFullPath(meshSdf->Uri(), meshSdf->FilePath());
          auto *mesh = meshManager.Load(fullPath);
          if (nullptr == mesh)
          {
            ignwarn << "Failed to load mesh from [" << fullPath
                    << "]." << std::endl;
            return true;
          }

          auto linkMeshFeature = entityCast(_parent->Data(), linkPtrPhys,
              this->entityLinkMeshMap);
          if (!linkMeshFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              igndbg << "Attempting to process mesh geometries, but the physics"
                     << " engine doesn't support feature "
                     << "[AttachMeshShapeFeature]. Meshes will be ignored."
                     << std::endl;
              informed = true;
            }
            return true;
          }

          collisionPtrPhys = linkMeshFeature->AttachMeshShape(_name->Data(),
              *mesh,
              math::eigen3::convert(_pose->Data()),
              math::eigen3::convert(meshSdf->Scale()));
        }
        else
        {
          auto linkCollisionFeature = entityCast(_parent->Data(), linkPtrPhys,
              this->entityLinkCollisionMap);
          if (!linkCollisionFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              igndbg << "Attempting to process collisions, but the physics "
                     << "engine doesn't support feature "
                     << "[ConstructSdfCollision]. Collisions will be ignored."
                     << std::endl;
              informed = true;
            }
            return true;
          }

          collisionPtrPhys =
              linkCollisionFeature->ConstructCollision(collision);
        }
        // Check that the physics engine has a filter mask feature
        // Set the collide_bitmask if it does
        auto filterMaskFeature = entityCast(_parent->Data(), collisionPtrPhys,
            entityShapeMaskMap);
        if (filterMaskFeature)
        {
          filterMaskFeature->SetCollisionFilterMask(collideBitmask);
        }
        else
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to set collision bitmasks, but the physics "
                   << "engine doesn't support feature [CollisionFilterMask]. "
                   << "Collision bitmasks will be ignored." << std::endl;
            informed = true;
          }
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

        auto modelJointFeature = entityCast(_parentModel->Data(), modelPtrPhys,
            this->entityModelJointMap);
        if (!modelJointFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to process joints, but the physics "
                   << "engine doesn't support joint features. "
                   << "Joints will be ignored." << std::endl;
            informed = true;
          }

          // Break Each call since no joints can be processed
          return false;
        }

        sdf::Joint joint;
        joint.SetName(_name->Data());
        joint.SetType(_jointType->Data());
        joint.SetRawPose(_pose->Data());
        joint.SetThreadPitch(_threadPitch->Data());

        joint.SetParentLinkName(_parentLinkName->Data());
        joint.SetChildLinkName(_childLinkName->Data());

        auto jointAxis = _ecm.Component<components::JointAxis>(_entity);
        auto jointAxis2 = _ecm.Component<components::JointAxis2>(_entity);

        // Since we're making copies of the joint axes that were created using
        // `Model::Load`, frame semantics should work for resolving their xyz
        // axis
        if (jointAxis)
          joint.SetAxis(0, jointAxis->Data());
        if (jointAxis2)
          joint.SetAxis(1, jointAxis2->Data());

        // Use the parent link's parent model as the model of this joint
        auto jointPtrPhys = modelJointFeature->ConstructJoint(joint);

        if (jointPtrPhys.Valid())
        {
          // Some joints may not be supported, so only add them to the map if
          // the physics entity is valid
          this->entityJointMap.insert(std::make_pair(_entity, jointPtrPhys));
        }
        return true;
      });

  _ecm.EachNew<components::BatterySoC>(
      [&](const Entity & _entity, const components::BatterySoC *)->bool
      {
        // Parent entity of battery is model entity
        this->entityOffMap.insert(std::make_pair(
          _ecm.ParentEntity(_entity), false));
        return true;
      });

  // Detachable joints
  _ecm.EachNew<components::DetachableJoint>(
      [&](const Entity &_entity,
          const components::DetachableJoint *_jointInfo) -> bool
      {
        if (_jointInfo->Data().jointType != "fixed")
        {
          ignerr << "Detachable joint type [" << _jointInfo->Data().jointType
                 << "] is currently not supported" << std::endl;
          return true;
        }
        // Check if joint already exists
        if (this->entityJointMap.find(_entity) != this->entityJointMap.end())
        {
          ignwarn << "Joint entity [" << _entity
                  << "] marked as new, but it's already on the map."
                  << std::endl;
          return true;
        }

        // Check if the link entities exist in the physics engine
        auto parentLinkPhysIt =
            this->entityLinkMap.find(_jointInfo->Data().parentLink);
        if (parentLinkPhysIt == this->entityLinkMap.end())
        {
          ignwarn << "DetachableJoint's parent link entity ["
                  << _jointInfo->Data().parentLink << "] not found in link map."
                  << std::endl;
          return true;
        }

        auto childLinkEntity = _jointInfo->Data().childLink;

        // Get child link
        auto childLinkIt = this->entityLinkMap.find(childLinkEntity);
        if (childLinkIt == this->entityLinkMap.end())
        {
          ignwarn << "Failed to find joint's child link [" << childLinkEntity
                  << "]." << std::endl;
          return true;
        }

        auto childLinkDetachableJointFeature = entityCast(childLinkEntity,
            childLinkIt->second, this->entityLinkDetachableJointMap);
        if (!childLinkDetachableJointFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to create a detachable joint, but the physics"
                   << " engine doesn't support feature "
                   << "[AttachFixedJointFeature]. Detachable joints will be "
                   << "ignored." << std::endl;
            informed = true;
          }

          // Break Each call since no DetachableJoints can be processed
          return false;
        }

        const auto poseParent =
            parentLinkPhysIt->second->FrameDataRelativeToWorld().pose;
        const auto poseChild =
            childLinkDetachableJointFeature->FrameDataRelativeToWorld().pose;

        // Pose of child relative to parent
        auto poseParentChild = poseParent.inverse() * poseChild;
        auto jointPtrPhys = childLinkDetachableJointFeature->AttachFixedJoint(
            parentLinkPhysIt->second);
        if (jointPtrPhys.Valid())
        {
          // We let the joint be at the origin of the child link.
          jointPtrPhys->SetTransformFromParent(poseParentChild);

          igndbg << "Creating detachable joint [" << _entity << "]"
                 << std::endl;
          this->entityJointMap.insert(std::make_pair(_entity, jointPtrPhys));
        }
        else
        {
          ignwarn << "DetachableJoint could not be created." << std::endl;
        }
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
            // First erase the entry associated with this link from the
            // linkEntityMap which is the reverse of entityLinkMap
            auto linkPhysIt = this->entityLinkMap.find(childLink);
            if (linkPhysIt != this->entityLinkMap.end())
            {
              this->linkEntityMap.erase(linkPhysIt->second);
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

  _ecm.EachRemoved<components::DetachableJoint>(
      [&](const Entity &_entity, const components::DetachableJoint *) -> bool
      {
        auto jointIt = this->entityJointMap.find(_entity);
        if (jointIt == this->entityJointMap.end())
        {
          ignwarn << "Failed to find joint [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto castEntity = entityCast(_entity, jointIt->second,
            this->entityJointDetachableJointMap);
        if (!castEntity)
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to detach a joint, but the physics "
                   << "engine doesn't support feature "
                   << "[DetachJointFeature]. Joint won't be detached."
                   << std::endl;
            informed = true;
          }

          // Break Each call since no DetachableJoints can be processed
          return false;
        }

        igndbg << "Detaching joint [" << _entity << "]" << std::endl;
        castEntity->Detach();
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdatePhysics(EntityComponentManager &_ecm)
{
  IGN_PROFILE("PhysicsPrivate::UpdatePhysics");
  // Battery state
  _ecm.Each<components::BatterySoC>(
      [&](const Entity & _entity, const components::BatterySoC *_bat)
      {
        if (_bat->Data() <= 0)
          entityOffMap[_ecm.ParentEntity(_entity)] = true;
        else
          entityOffMap[_ecm.ParentEntity(_entity)] = false;
        return true;
      });

  // Handle joint state
  _ecm.Each<components::Joint, components::Name>(
      [&](const Entity &_entity, const components::Joint *,
          const components::Name *_name)
      {
        auto jointIt = this->entityJointMap.find(_entity);
        if (jointIt == this->entityJointMap.end())
          return true;

        // Model is out of battery
        if (this->entityOffMap[_ecm.ParentEntity(_entity)])
        {
          std::size_t nDofs = jointIt->second->GetDegreesOfFreedom();
          for (std::size_t i = 0; i < nDofs; ++i)
          {
            jointIt->second->SetForce(i, 0);
          }
          return true;
        }

        auto posReset = _ecm.Component<components::JointPositionReset>(
            _entity);
        auto velReset = _ecm.Component<components::JointVelocityReset>(
            _entity);

        // Reset the velocity
        if (velReset)
        {
          auto& jointVelocity = velReset->Data();

          if (jointVelocity.size() != jointIt->second->GetDegreesOfFreedom())
          {
            ignwarn << "There is a mismatch in the degrees of freedom "
                    << "between Joint [" << _name->Data() << "(Entity="
                    << _entity << ")] and its JointVelocityReset "
                    << "component. The joint has "
                    << jointIt->second->GetDegreesOfFreedom()
                    << " while the component has "
                    << jointVelocity.size() << ".\n";
            }

            std::size_t nDofs = std::min(
                jointVelocity.size(), jointIt->second->GetDegreesOfFreedom());

            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointIt->second->SetVelocity(i, jointVelocity[i]);
            }
        }

        // Reset the position
        if (posReset)
        {
          auto &jointPosition = posReset->Data();

          if (jointPosition.size() != jointIt->second->GetDegreesOfFreedom())
          {
            ignwarn << "There is a mismatch in the degrees of freedom "
                    << "between Joint [" << _name->Data() << "(Entity="
                    << _entity << ")] and its JointPositionyReset "
                    << "component. The joint has "
                    << jointIt->second->GetDegreesOfFreedom()
                    << " while the component has "
                    << jointPosition.size() << ".\n";
            }
            std::size_t nDofs = std::min(
                jointPosition.size(), jointIt->second->GetDegreesOfFreedom());
            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointIt->second->SetPosition(i, jointPosition[i]);
            }
        }

        auto force = _ecm.Component<components::JointForceCmd>(_entity);
        auto velCmd = _ecm.Component<components::JointVelocityCmd>(_entity);

        if (force)
        {
          if (force->Data().size() != jointIt->second->GetDegreesOfFreedom())
          {
            ignwarn << "There is a mismatch in the degrees of freedom between "
                    << "Joint [" << _name->Data() << "(Entity=" << _entity
                    << ")] and its JointForceCmd component. The joint has "
                    << jointIt->second->GetDegreesOfFreedom() << " while the "
                    << " component has " << force->Data().size() << ".\n";
          }
          std::size_t nDofs = std::min(force->Data().size(),
                                       jointIt->second->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < nDofs; ++i)
          {
            jointIt->second->SetForce(i, force->Data()[i]);
          }
        }
        // Only set joint velocity if joint force is not set.
        // If both the cmd and reset components are found, cmd is ignored.
        else if (velCmd)
        {
          auto velocityCmd = velCmd->Data();

          if (velReset)
          {
            ignwarn << "Found both JointVelocityReset and "
                    << "JointVelocityCmd components for Joint ["
                    << _name->Data() << "(Entity=" << _entity
                    << "]). Ignoring JointVelocityCmd component."
                    << std::endl;
            return true;
          }

          if (velocityCmd.size() != jointIt->second->GetDegreesOfFreedom())
          {
            ignwarn << "There is a mismatch in the degrees of freedom"
                    << " between Joint [" << _name->Data()
                    << "(Entity=" << _entity<< ")] and its "
                    << "JointVelocityCmd component. The joint has "
                    << jointIt->second->GetDegreesOfFreedom()
                    << " while the component has "
                    << velocityCmd.size() << ".\n";
          }

          auto jointVelFeature = entityCast(_entity, jointIt->second,
              this->entityJointVelocityCommandMap);
          if (!jointVelFeature)
          {
            return true;
          }

          std::size_t nDofs = std::min(
            velocityCmd.size(),
            jointIt->second->GetDegreesOfFreedom());

          for (std::size_t i = 0; i < nDofs; ++i)
          {
            jointVelFeature->SetVelocityCommand(i, velocityCmd[i]);
          }
        }

        return true;
      });

  // Link wrenches
  _ecm.Each<components::ExternalWorldWrenchCmd>(
      [&](const Entity &_entity,
          const components::ExternalWorldWrenchCmd *_wrenchComp)
      {
        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt == this->entityLinkMap.end())
        {
          ignwarn << "Failed to find link [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto linkForceFeature = entityCast(_entity, linkIt->second,
            this->entityLinkForceMap);
        if (!linkForceFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to apply a wrench, but the physics "
                   << "engine doesn't support feature "
                   << "[AddLinkExternalForceTorque]. Wrench will be ignored."
                   << std::endl;
            informed = true;
          }

          // Break Each call since no ExternalWorldWrenchCmd's can be processed
          return false;
        }

        math::Vector3 force = msgs::Convert(_wrenchComp->Data().force());
        math::Vector3 torque = msgs::Convert(_wrenchComp->Data().torque());
        linkForceFeature->AddExternalForce(math::eigen3::convert(force));
        linkForceFeature->AddExternalTorque(math::eigen3::convert(torque));

        return true;
      });

  // Update model pose
  _ecm.Each<components::Model, components::WorldPoseCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::WorldPoseCmd *_poseCmd)
      {
        auto modelIt = this->entityModelMap.find(_entity);
        if (modelIt == this->entityModelMap.end())
          return true;

        // The canonical link as specified by sdformat is different from the
        // canonical link of the FreeGroup object

        // TODO(addisu) Store the free group instead of searching for it at
        // every iteration
        auto freeGroup = modelIt->second->FindFreeGroup();
        if (!freeGroup)
          return true;

        // Get canonical link offset
        auto linkEntityIt =
            this->linkEntityMap.find(freeGroup->CanonicalLink());
        if (linkEntityIt == this->linkEntityMap.end())
          return true;

        // set world pose of canonical link in freegroup
        math::Pose3d linkPose =
            this->RelativePose(_entity, linkEntityIt->second, _ecm);

        freeGroup->SetWorldPose(math::eigen3::convert(_poseCmd->Data() *
                                linkPose));

        // Process pose commands for static models here, as one-time changes
        const components::Static *staticComp =
          _ecm.Component<components::Static>(_entity);
        if (staticComp && staticComp->Data())
        {
          auto worldPoseComp = _ecm.Component<components::Pose>(_entity);
          if (worldPoseComp)
          {
            auto state = worldPoseComp->SetData(_poseCmd->Data(),
                this->pose3Eql) ?
                ComponentState::OneTimeChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::Pose::typeId, state);
          }
        }

        return true;
      });

  // Update model angular velocity
  _ecm.Each<components::Model, components::AngularVelocityCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::AngularVelocityCmd *_angularVelocityCmd)
      {
        auto modelIt = this->entityModelMap.find(_entity);
        if (modelIt == this->entityModelMap.end())
          return true;

        auto freeGroup = modelIt->second->FindFreeGroup();
        if (!freeGroup)
          return true;

        const components::Pose *poseComp =
            _ecm.Component<components::Pose>(_entity);
        math::Vector3d worldAngularVel = poseComp->Data().Rot() *
            _angularVelocityCmd->Data();

        auto worldAngularVelFeature = entityCast(_entity, freeGroup,
            this->entityWorldVelocityCommandMap);
        if (!worldAngularVelFeature)
        {
          return true;
        }

        worldAngularVelFeature->SetWorldAngularVelocity(
            math::eigen3::convert(worldAngularVel));

        return true;
      });

  // Update model linear velocity
  _ecm.Each<components::Model, components::LinearVelocityCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::LinearVelocityCmd *_linearVelocityCmd)
      {
        auto modelIt = this->entityModelMap.find(_entity);
        if (modelIt == this->entityModelMap.end())
          return true;

        auto freeGroup = modelIt->second->FindFreeGroup();
        if (!freeGroup)
          return true;

        const components::Pose *poseComp =
            _ecm.Component<components::Pose>(_entity);
        math::Vector3d worldLinearVel = poseComp->Data().Rot() *
            _linearVelocityCmd->Data();

        auto worldLinearVelFeature = entityCast(_entity, freeGroup,
              this->entityWorldVelocityCommandMap);
        if (!worldLinearVelFeature)
        {
          return true;
        }

        worldLinearVelFeature->SetWorldLinearVelocity(
            math::eigen3::convert(worldLinearVel));

        return true;
      });

  // Clear pending commands
  // Note: Removing components from inside an Each call can be dangerous.
  // Instead, we collect all the entities that have the desired components and
  // remove the component from them afterward.
  std::vector<Entity> entitiesWorldCmd;
  _ecm.Each<components::WorldPoseCmd>(
      [&](const Entity &_entity, components::WorldPoseCmd*) -> bool
      {
        entitiesWorldCmd.push_back(_entity);
        return true;
      });

  for (const Entity &entity : entitiesWorldCmd)
  {
    _ecm.RemoveComponent<components::WorldPoseCmd>(entity);
  }

  // Populate bounding box info
  // Only compute bounding box if component exists to avoid unnecessary
  // computations
  _ecm.Each<components::Model, components::AxisAlignedBox>(
      [&](const Entity &_entity, const components::Model *,
          components::AxisAlignedBox *_bbox)
      {
        auto modelIt = this->entityModelMap.find(_entity);
        if (modelIt == this->entityModelMap.end())
        {
          ignwarn << "Failed to find model [" << _entity << "]." << std::endl;
          return true;
        }

        auto bbModel = entityCast(_entity, modelIt->second,
            this->entityModelBoundingBoxMap);
        if (!bbModel)
        {
          static bool informed{false};
          if (!informed)
          {
            igndbg << "Attempting to get a bounding box, but the physics "
                   << "engine doesn't support feature "
                   << "[GetModelBoundingBox]. Bounding box won't be populated."
                   << std::endl;
            informed = true;
          }

          // Break Each call since no AxisAlignedBox'es can be processed
          return false;
        }

        math::AxisAlignedBox bbox =
            math::eigen3::convert(bbModel->GetAxisAlignedBoundingBox());
        auto state = _bbox->SetData(bbox, this->axisAlignedBoxEql) ?
            ComponentState::OneTimeChange :
            ComponentState::NoChange;
        _ecm.SetChanged(_entity, components::AxisAlignedBox::typeId, state);

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
ignition::gazebo::Entity PhysicsPrivate::TopLevelModel(const Entity &_entity,
    const EntityComponentManager &_ecm) const
{
  auto entity = _entity;

  // check if parent is a model
  auto parentComp = _ecm.Component<components::ParentEntity>(entity);
  while (parentComp)
  {
    // check if parent is a model
    auto parentEntity = parentComp->Data();
    auto modelComp = _ecm.Component<components::Model>(
        parentEntity);
    if (!modelComp)
      break;

    // set current model entity
    entity = parentEntity;
    parentComp = _ecm.Component<components::ParentEntity>(entity);
  }
  return entity;
}

//////////////////////////////////////////////////
ignition::math::Pose3d PhysicsPrivate::RelativePose(const Entity &_from,
  const Entity &_to, const EntityComponentManager &_ecm) const
{
  math::Pose3d transform;

  if (_from == _to)
    return transform;

  auto currentEntity = _to;
  auto parentComp = _ecm.Component<components::ParentEntity>(_to);
  while (parentComp)
  {
    auto parentEntity = parentComp->Data();

    // get the entity pose
    auto entityPoseComp =
      _ecm.Component<components::Pose>(currentEntity);

    // update transform
    transform = entityPoseComp->Data() * transform;

    if (parentEntity == _from)
      break;

    // set current entity to parent
    currentEntity = parentEntity;

    // get entity's parent
    parentComp = _ecm.Component<components::ParentEntity>(
      parentEntity);
  }

  return transform;
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateSim(EntityComponentManager &_ecm)
{
  IGN_PROFILE("PhysicsPrivate::UpdateSim");

  // local pose
  _ecm.Each<components::Link, components::Pose, components::ParentEntity>(
      [&](const Entity &_entity, components::Link * /*_link*/,
          components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        // get top level model of this link
        auto topLevelModel = this->TopLevelModel(_parent->Data(), _ecm);

        // If model is static, don't process pose changes as periodic
        const auto *staticComp =
          _ecm.Component<components::Static>(topLevelModel);

        if (staticComp && staticComp->Data())
          return true;

        auto linkIt = this->entityLinkMap.find(_entity);
        if (linkIt != this->entityLinkMap.end())
        {
          auto canonicalLink =
              _ecm.Component<components::CanonicalLink>(_entity);

          auto frameData = linkIt->second->FrameDataRelativeToWorld();
          const auto &worldPose = frameData.pose;

          if (canonicalLink)
          {
            // This is the canonical link, update the top level model.
            // The pose of this link w.r.t its top level model never changes
            // because it's "fixed" to the model. Instead, we change
            // the top level model's pose here. The physics engine gives us the
            // pose of this link relative to world so to set the top level
            // model's pose, we have to post-multiply it by the inverse of the
            // transform of the link w.r.t to its top level model.
            math::Pose3d linkPoseFromTopLevelModel;
            linkPoseFromTopLevelModel =
                this->RelativePose(topLevelModel, _entity, _ecm);

            // update top level model's pose
            auto mutableModelPose =
               _ecm.Component<components::Pose>(topLevelModel);
            *(mutableModelPose) = components::Pose(
                linkPoseFromTopLevelModel.Inverse() +
                math::eigen3::convert(worldPose));

            _ecm.SetChanged(_parent->Data(), components::Pose::typeId,
                ComponentState::PeriodicChange);
          }
          else
          {
            // Compute the relative pose of this link from the model
            // first get the world pose of the parent model
            auto worldComp =
                _ecm.Component<components::ParentEntity>(topLevelModel);
            // if the worldComp is a nullptr, something is wrong with ECS
            if (!worldComp)
            {
              ignerr << "The parent component of " << topLevelModel
                     << " could not be found. This should never happen!\n";
              return true;
            }
            math::Pose3d parentWorldPose =
                this->RelativePose(worldComp->Data(), _parent->Data(), _ecm);

            *_pose = components::Pose(math::eigen3::convert(worldPose) +
                                      parentWorldPose.Inverse());
            _ecm.SetChanged(_entity, components::Pose::typeId,
                ComponentState::PeriodicChange);
          }

          // Populate world poses, velocities and accelerations of the link. For
          // now these components are updated only if another system has created
          // the corresponding component on the entity.
          auto worldPoseComp = _ecm.Component<components::WorldPose>(_entity);
          if (worldPoseComp)
          {
            auto state =
                worldPoseComp->SetData(math::eigen3::convert(frameData.pose),
                this->pose3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::WorldPose::typeId, state);
          }

          // Velocity in world coordinates
          auto worldLinVelComp =
              _ecm.Component<components::WorldLinearVelocity>(_entity);
          if (worldLinVelComp)
          {
            auto state = worldLinVelComp->SetData(
                  math::eigen3::convert(frameData.linearVelocity),
                  this->vec3Eql) ?
                  ComponentState::PeriodicChange :
                  ComponentState::NoChange;
            _ecm.SetChanged(_entity,
                components::WorldLinearVelocity::typeId, state);
          }

          // Angular velocity in world frame coordinates
          auto worldAngVelComp =
              _ecm.Component<components::WorldAngularVelocity>(_entity);
          if (worldAngVelComp)
          {
            auto state = worldAngVelComp->SetData(
                math::eigen3::convert(frameData.angularVelocity),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity,
                components::WorldAngularVelocity::typeId, state);
          }

          // Acceleration in world frame coordinates
          auto worldLinAccelComp =
              _ecm.Component<components::WorldLinearAcceleration>(_entity);
          if (worldLinAccelComp)
          {
            auto state = worldLinAccelComp->SetData(
                math::eigen3::convert(frameData.linearAcceleration),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity,
                components::WorldLinearAcceleration::typeId, state);
          }

          // Angular acceleration in world frame coordinates
          auto worldAngAccelComp =
              _ecm.Component<components::WorldAngularAcceleration>(_entity);

          if (worldAngAccelComp)
          {
            auto state = worldAngAccelComp->SetData(
                math::eigen3::convert(frameData.angularAcceleration),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity,
                components::WorldAngularAcceleration::typeId, state);
          }

          const Eigen::Matrix3d R_bs = worldPose.linear().transpose(); // NOLINT

          // Velocity in body-fixed frame coordinates
          auto bodyLinVelComp =
              _ecm.Component<components::LinearVelocity>(_entity);
          if (bodyLinVelComp)
          {
            Eigen::Vector3d bodyLinVel = R_bs * frameData.linearVelocity;
            auto state =
                bodyLinVelComp->SetData(math::eigen3::convert(bodyLinVel),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::LinearVelocity::typeId, state);
          }

          // Angular velocity in body-fixed frame coordinates
          auto bodyAngVelComp =
              _ecm.Component<components::AngularVelocity>(_entity);
          if (bodyAngVelComp)
          {
            Eigen::Vector3d bodyAngVel = R_bs * frameData.angularVelocity;
            auto state =
                bodyAngVelComp->SetData(math::eigen3::convert(bodyAngVel),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::AngularVelocity::typeId,
                state);
          }

          // Acceleration in body-fixed frame coordinates
          auto bodyLinAccelComp =
              _ecm.Component<components::LinearAcceleration>(_entity);
          if (bodyLinAccelComp)
          {
            Eigen::Vector3d bodyLinAccel = R_bs * frameData.linearAcceleration;
            auto state =
                bodyLinAccelComp->SetData(math::eigen3::convert(bodyLinAccel),
                this->vec3Eql)?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::LinearAcceleration::typeId,
                state);
          }

          // Angular acceleration in world frame coordinates
          auto bodyAngAccelComp =
              _ecm.Component<components::AngularAcceleration>(_entity);
          if (bodyAngAccelComp)
          {
            Eigen::Vector3d bodyAngAccel = R_bs * frameData.angularAcceleration;
            auto state =
                bodyAngAccelComp->SetData(math::eigen3::convert(bodyAngAccel),
                this->vec3Eql) ?
                ComponentState::PeriodicChange :
                ComponentState::NoChange;
            _ecm.SetChanged(_entity, components::AngularAcceleration::typeId,
                state);
          }
        }
        return true;
      });

  // pose/velocity/acceleration of non-link entities such as sensors /
  // collisions. These get updated only if another system has created a
  // components::WorldPose component for the entity.
  // Populated components:
  // * WorldPose
  // * WorldLinearVelocity
  // * AngularVelocity
  // * LinearAcceleration

  // world pose
  _ecm.Each<components::Pose, components::WorldPose,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose, components::WorldPose *_worldPose,
          const components::ParentEntity *_parent)->bool
      {
        // check if parent entity is a link, e.g. entity is sensor / collision
        auto linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkIt->second, _pose->Data());

          *_worldPose = components::WorldPose(
              math::eigen3::convert(entityFrameData.pose));
        }

        return true;
      });

  // world linear velocity
  _ecm.Each<components::Pose, components::WorldLinearVelocity,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::WorldLinearVelocity *_worldLinearVel,
          const components::ParentEntity *_parent)->bool
      {
        // check if parent entity is a link, e.g. entity is sensor / collision
        auto linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkIt->second, _pose->Data());

          // set entity world linear velocity
          *_worldLinearVel = components::WorldLinearVelocity(
              math::eigen3::convert(entityFrameData.linearVelocity));
        }

        return true;
      });

  // body angular velocity
  _ecm.Each<components::Pose, components::AngularVelocity,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::AngularVelocity *_angularVel,
          const components::ParentEntity *_parent)->bool
      {
        // check if parent entity is a link, e.g. entity is sensor / collision
        auto linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkIt->second, _pose->Data());

          auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
          ignition::math::Vector3d entityWorldAngularVel =
              math::eigen3::convert(entityFrameData.angularVelocity);

          auto entityBodyAngularVel =
              entityWorldPose.Rot().RotateVectorReverse(entityWorldAngularVel);
          *_angularVel = components::AngularVelocity(entityBodyAngularVel);
        }

        return true;
      });

  // body linear acceleration
  _ecm.Each<components::Pose, components::LinearAcceleration,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::LinearAcceleration *_linearAcc,
          const components::ParentEntity *_parent)->bool
      {
        auto linkIt = this->entityLinkMap.find(_parent->Data());
        if (linkIt != this->entityLinkMap.end())
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkIt->second, _pose->Data());

          auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
          ignition::math::Vector3d entityWorldLinearAcc =
              math::eigen3::convert(entityFrameData.linearAcceleration);

          auto entityBodyLinearAcc =
              entityWorldPose.Rot().RotateVectorReverse(entityWorldLinearAcc);
          *_linearAcc = components::LinearAcceleration(entityBodyLinearAcc);
        }

        return true;
      });

  // Clear reset components
  std::vector<Entity> entitiesPositionReset;
  _ecm.Each<components::JointPositionReset>(
      [&](const Entity &_entity, components::JointPositionReset *) -> bool
      {
        entitiesPositionReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesPositionReset)
  {
    _ecm.RemoveComponent<components::JointPositionReset>(entity);
  }

  std::vector<Entity> entitiesVelocityReset;
  _ecm.Each<components::JointVelocityReset>(
      [&](const Entity &_entity, components::JointVelocityReset *) -> bool
      {
        entitiesVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesVelocityReset)
  {
    _ecm.RemoveComponent<components::JointVelocityReset>(entity);
  }

  // Clear pending commands
  _ecm.Each<components::JointForceCmd>(
      [&](const Entity &, components::JointForceCmd *_force) -> bool
      {
        std::fill(_force->Data().begin(), _force->Data().end(), 0.0);
        return true;
      });

  _ecm.Each<components::ExternalWorldWrenchCmd >(
      [&](const Entity &, components::ExternalWorldWrenchCmd *_wrench) -> bool
      {
        _wrench->Data().Clear();
        return true;
      });

  _ecm.Each<components::JointVelocityCmd>(
      [&](const Entity &, components::JointVelocityCmd *_vel) -> bool
      {
        std::fill(_vel->Data().begin(), _vel->Data().end(), 0.0);
        return true;
      });

  // Update joint positions
  _ecm.Each<components::Joint, components::JointPosition>(
      [&](const Entity &_entity, components::Joint *,
          components::JointPosition *_jointPos) -> bool
      {
        auto jointIt = this->entityJointMap.find(_entity);
        if (jointIt != this->entityJointMap.end())
        {
          _jointPos->Data().resize(jointIt->second->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < jointIt->second->GetDegreesOfFreedom();
               ++i)
          {
            _jointPos->Data()[i] = jointIt->second->GetPosition(i);
          }
        }
        return true;
      });

  // Update joint Velocities
  _ecm.Each<components::Joint, components::JointVelocity>(
      [&](const Entity &_entity, components::Joint *,
          components::JointVelocity *_jointVel) -> bool
      {
        auto jointIt = this->entityJointMap.find(_entity);
        if (jointIt != this->entityJointMap.end())
        {
          _jointVel->Data().resize(jointIt->second->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < jointIt->second->GetDegreesOfFreedom();
               ++i)
          {
            _jointVel->Data()[i] = jointIt->second->GetVelocity(i);
          }
        }
        return true;
      });

  // TODO(louise) Skip this if there are no collision features
  this->UpdateCollisions(_ecm);
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateCollisions(EntityComponentManager &_ecm)
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

  auto worldIt = this->entityWorldMap.find(worldEntity);
  if (worldIt == this->entityWorldMap.end())
  {
    ignwarn << "Failed to find world [" << worldEntity << "]." << std::endl;
    return;
  }

  auto worldCollisionFeature = entityCast(worldEntity, worldIt->second,
      this->entityWorldCollisionMap);
  if (!worldCollisionFeature)
  {
    static bool informed{false};
    if (!informed)
    {
      igndbg << "Attempting process contacts, but the physics "
             << "engine doesn't support collision features. "
             << "Contacts won't be computed."
             << std::endl;
      informed = true;
    }
    return;
  }

  // Each contact object we get from ign-physics contains the EntityPtrs of the
  // two colliding entities and other data about the contact such as the
  // position. This map groups contacts so that it is easy to query all the
  // contacts of one entity.
  using EntityContactMap = std::unordered_map<Entity,
      std::deque<const WorldShapeType::ContactPoint *>>;

  // This data structure is essentially a mapping between a pair of entities and
  // a list of pointers to their contact object. We use a map inside a map to
  // create msgs::Contact objects conveniently later on.
  std::unordered_map<Entity, EntityContactMap> entityContactMap;

  // Note that we are temporarily storing pointers to elements in this
  // ("allContacts") container. Thus, we must make sure it doesn't get destroyed
  // until the end of this function.
  auto allContacts = worldCollisionFeature->GetContactsFromLastStep();
  for (const auto &contactComposite : allContacts)
  {
    const auto &contact = contactComposite.Get<WorldShapeType::ContactPoint>();
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
        *_contacts = components::ContactSensorData(contactsComp);

        return true;
      });
}

physics::FrameData3d PhysicsPrivate::LinkFrameDataAtOffset(
      const LinkPtrType &_link, const math::Pose3d &_pose) const
{
  physics::FrameData3d parent;
  parent.pose = math::eigen3::convert(_pose);
  physics::RelativeFrameData3d relFrameData(_link->GetFrameID(), parent);
  return this->engine->Resolve(relFrameData, physics::FrameID::World());
}

IGNITION_ADD_PLUGIN(Physics,
                    ignition::gazebo::System,
                    Physics::ISystemConfigure,
                    Physics::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Physics, "ignition::gazebo::systems::Physics")
