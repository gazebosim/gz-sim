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

#ifndef GZ_SIM_SYSTEMS_PHYSICS_PHYSICSPRIVATE_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_PHYSICSPRIVATE_HH_

#include "Physics.hh"

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/Utility.hh>

#include <algorithm>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/HeightmapUtil.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Uuid.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/config.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>
#include <gz/physics/heightmap/HeightmapShape.hh>
#include <gz/physics/InstallationDirectories.hh>
#include <gz/physics/RelativeQuantity.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/BoxShape.hh>
#include <gz/physics/ConeShape.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/physics/CylinderShape.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/GetBatchRayIntersection.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/GetRayIntersection.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/SphereShape.hh>
#include <gz/physics/World.hh>
#include <gz/physics/mesh/MeshShape.hh>
#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/Gravity.hh>
#include <gz/physics/Model.hh>
#include <gz/plugin/Loader.hh>
#include <gz/plugin/PluginPtr.hh>

// SDF
#include <sdf/Collision.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Polyline.hh>
#include <sdf/Surface.hh>
#include <sdf/World.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"

// Components
#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/CollisionBitmask.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/RaycastData.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/PhysicsEnginePlugin.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/components/SelfCollide.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/ThreadPitch.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/HaltMotion.hh"

#include "CanonicalLinkModelTracker.hh"
// Events
#include "gz/sim/physics/Events.hh"

#include "EntityFeatureMap.hh"

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;
using namespace gz::sim::systems::physics_system;
namespace components = gz::sim::components;


// Private data class.
class gz::sim::systems::PhysicsPrivate
{
  /// \brief This is the minimum set of features that any physics engine must
  /// implement to be supported by this system.
  /// New features can't be added to this list in minor / patch releases, in
  /// order to maintain backwards compatibility with downstream physics plugins.
  public: struct MinimumFeatureList : physics::FeatureList<
          physics::FindFreeGroupFeature,
          physics::SetFreeGroupWorldPose,
          physics::FreeGroupFrameSemantics,
          physics::LinkFrameSemantics,
          physics::ForwardStep,
          physics::RemoveModelFromWorld,
          physics::sdf::ConstructSdfModel,
          physics::sdf::ConstructSdfWorld,
          physics::GetLinkFromModel,
          physics::GetShapeFromLink
          >{};

  /// \brief Engine type with just the minimum features.
  public: using EnginePtrType = physics::EnginePtr<
            physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief World type with just the minimum features.
  public: using WorldPtrType = physics::WorldPtr<
            physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Model type with just the minimum features.
  public: using ModelPtrType = physics::ModelPtr<
            physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Link type with just the minimum features.
  public: using LinkPtrType = physics::LinkPtr<
            physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Free group type with just the minimum features.
  public: using FreeGroupPtrType = physics::FreeGroupPtr<
            physics::FeaturePolicy3d, MinimumFeatureList>;

  /// \brief Create physics entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreatePhysicsEntities(const EntityComponentManager &_ecm,
                                     bool _warnIfEntityExists = true);

  /// \brief Create world entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreateWorldEntities(const EntityComponentManager &_ecm,
                                   bool _warnIfEntityExists = true);
  /// \brief Create model entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreateModelEntities(const EntityComponentManager &_ecm,
                                   bool _warnIfEntityExists = true);

  /// \brief Create link entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreateLinkEntities(const EntityComponentManager &_ecm,
                                  bool _warnIfEntityExists = true);

  /// \brief Create collision entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreateCollisionEntities(const EntityComponentManager &_ecm,
                                       bool _warnIfEntityExists = true);

  /// \brief Create joint entities
  /// \param[in] _ecm Constant reference to ECM.
  /// \param[in] _warnIfEntityExists True to emit warnings if the same entity
  /// already exists in the physics system
  public: void CreateJointEntities(const EntityComponentManager &_ecm,
                                   bool _warnIfEntityExists = true);

  /// \brief Create Battery entities
  /// \param[in] _ecm Constant reference to ECM.
  public: void CreateBatteryEntities(const EntityComponentManager &_ecm);

  /// \brief Remove physics entities if they are removed from the ECM
  /// \param[in] _ecm Constant reference to ECM.
  public: void RemovePhysicsEntities(const EntityComponentManager &_ecm);

  /// \brief Update physics from components
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdatePhysics(EntityComponentManager &_ecm);

  /// \brief Reset physics from components
  /// \param[in] _ecm Constant reference to ECM.
  public: void ResetPhysics(EntityComponentManager &_ecm);

  /// \brief Step the simulation for each world
  /// \param[in] _dt Duration
  /// \returns Output data from the physics engine (this currently contains
  /// data for links that experienced a pose change in the physics step)
  public: gz::physics::ForwardStep::Output Step(
              const std::chrono::steady_clock::duration &_dt);

  /// \brief Get data of links that were updated in the latest physics step.
  /// \param[in] _ecm Mutable reference to ECM.
  /// \param[in] _updatedLinks Updated link poses from the latest physics step
  /// that were written to by the physics engine (some physics engines may
  /// not write this data to ForwardStep::Output. If not, _ecm is used to get
  /// this updated link pose data).
  /// \return A map of gazebo link entities to their updated pose data.
  /// std::map is used because canonical links must be in topological order
  /// to ensure that nested models with multiple canonical links are updated
  /// properly (models must be updated in topological order).
  public: std::map<Entity, physics::FrameData3d> ChangedLinks(
              EntityComponentManager &_ecm,
              const gz::physics::ForwardStep::Output &_updatedLinks);

  /// \brief Check if a model contains any plane collision geometry.
  /// \param[in] modelEntity The entity of the model to check.
  /// \param[in] _ecm The entity component manager.
  /// \return True if any collision geometry is a plane.
  public: bool ModelContainsPlaneCollision(const Entity &_modelEntity,
              EntityComponentManager &_ecm) const;

  /// \brief Helper function to update the pose of a model.
  /// \param[in] _model The model to update.
  /// \param[in] _canonicalLink The canonical link of _model.
  /// \param[in] _ecm The entity component manager.
  /// \param[in, out] _linkFrameData Links that experienced a pose change in the
  /// most recent physics step. The key is the entity of the link, and the
  /// value is the updated frame data corresponding to that entity. The
  /// canonical links of _model's nested models are added to _linkFrameData to
  /// ensure that all of _model's nested models are marked as models to be
  /// updated (if a parent model's pose changes, all nested model poses must be
  /// updated since nested model poses are saved w.r.t. the parent model).
  public: void UpdateModelPose(const Entity _model,
              const Entity _canonicalLink, EntityComponentManager &_ecm,
              std::map<Entity, physics::FrameData3d> &_linkFrameData);

  /// \brief Get an entity's frame data relative to world from physics.
  /// \param[in] _entity The entity.
  /// \param[in, out] _data The frame data to populate.
  /// \return True if _data was populated with frame data for _entity, false
  /// otherwise.
  public: bool GetFrameDataRelativeToWorld(const Entity _entity,
              physics::FrameData3d &_data);

  /// \brief Update components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  /// \param[in, out] _linkFrameData Links that experienced a pose change in the
  /// most recent physics step. The key is the entity of the link, and the
  /// value is the updated frame data corresponding to that entity.
  public: void UpdateSim(EntityComponentManager &_ecm,
              std::map<Entity, physics::FrameData3d> &_linkFrameData);

  /// \brief Update collision components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateCollisions(EntityComponentManager &_ecm);

  /// \brief Update ray intersection components from physics simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void UpdateRayIntersections(EntityComponentManager &_ecm);

  /// \brief FrameData relative to world at a given offset pose
  /// \param[in] _link gz-physics link
  /// \param[in] _pose Offset pose in which to compute the frame data
  /// \returns FrameData at the given offset pose
  public: physics::FrameData3d LinkFrameDataAtOffset(
      const LinkPtrType &_link, const math::Pose3d &_pose) const;

  /// \brief Get transform from one ancestor entity to a descendant entity
  /// that are in the same model.
  /// \param[in] _from An ancestor of the _to entity.
  /// \param[in] _to A descendant of the _from entity.
  /// \return Pose transform between the two entities
  public: math::Pose3d RelativePose(const Entity &_from,
      const Entity &_to, const EntityComponentManager &_ecm) const;

  /// \brief Enable contact surface customization for the given world.
  /// \param[in] _world The world to enable it for.
  public: void EnableContactSurfaceCustomization(const Entity &_world);

  /// \brief Disable contact surface customization for the given world.
  /// \param[in] _world The world to disable it for.
  public: void DisableContactSurfaceCustomization(const Entity &_world);

  /// \brief Update the AxisAlignedBox for link entities.
  /// \param[in] _ecm The entity component manager.
  public: void UpdateLinksBoundingBoxes(EntityComponentManager &_ecm);

  /// \brief Update the AxisAlignedBox for model entities.
  /// \param[in] _ecm The entity component manager.
  public: void UpdateModelsBoundingBoxes(EntityComponentManager &_ecm);

  /// \brief Cache the top-level model for each entity.
  /// The key is an entity and the value is its top level model.
  public: std::unordered_map<Entity, Entity> topLevelModelMap;

  /// \brief Keep track of what entities are static (models and links).
  public: std::unordered_set<Entity> staticEntities;

  /// \brief Keep track of poses for links attached to non-static models.
  /// This allows for skipping pose updates if a link's pose didn't change
  /// after a physics step.
  public: std::unordered_map<Entity, gz::math::Pose3d> linkWorldPoses;

  /// \brief Keep a mapping of canonical links to models that have this
  /// canonical link. Useful for updating model poses efficiently after a
  /// physics step
  public: CanonicalLinkModelTracker canonicalLinkModelTracker;

  /// \brief Keep track of non-static model world poses. Since non-static
  /// models may not move on a given iteration, we want to keep track of the
  /// most recent model world pose change that took place.
  public: std::unordered_map<Entity, math::Pose3d> modelWorldPoses;

  /// \brief A map between model entity ids in the ECM to whether its battery
  /// has drained.
  public: std::unordered_map<Entity, bool> entityOffMap;

  /// \brief Entities whose pose commands have been processed and should be
  /// deleted the following iteration.
  public: std::unordered_set<Entity> worldPoseCmdsToRemove;

  /// \brief Entities whose static commands have been processed and should
  /// be deleted the following iteration.
  public: std::unordered_set<Entity> staticCmdsToRemove;

  /// \brief Entities whose collide bitmask commands have been processed
  /// and should be deleted the following iteration.
  public: std::unordered_set<Entity> collideBitmaskCmdsToRemove;

  /// \brief Entities whose category bitmask commands have been processed
  /// and should be deleted the following iteration.
  public: std::unordered_set<Entity> categoryBitmaskCmdsToRemove;

  /// \brief Entities whose gravity enabled commands have been processed and
  /// should be deleted the following iteration.
  public: std::unordered_set<Entity> gravityEnabledCmdsToRemove;

  /// \brief Entities whose collision enabled commands have been processed and
  /// should be deleted the following iteration.
  public: std::unordered_set<Entity> collisionEnabledCmdsToRemove;

  /// \brief IDs of the ContactSurfaceHandler callbacks registered for worlds
  public: std::unordered_map<Entity, std::string> worldContactCallbackIDs;

  /// \brief used to store whether physics objects have been created.
  public: bool initialized = false;

  /// \brief Pointer to the underlying gz-physics Engine entity.
  public: EnginePtrType engine = nullptr;

  /// \brief Set whether to enforce fixed constraints. Applicable only if
  /// the underlying physics engine supports SetWeldChildToParent feature, e.g.
  /// gz-physics bullet-featherstone-plugin
  public: bool enforceFixedConstraint = false;

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
                         _a.Rot().Equal(_b.Rot(), 1e-6);
                     }};

  /// \brief AxisAlignedBox equality comparison function.
  public: std::function<bool(const math::AxisAlignedBox &,
          const math::AxisAlignedBox&)>
          axisAlignedBoxEql { [](const math::AxisAlignedBox &_a,
                                 const math::AxisAlignedBox &_b)
                     {
                       return _a == _b;
                     }};

  /// \brief msgs::Wrench equality comparison function.
  public: std::function<bool(const msgs::Wrench &, const msgs::Wrench &)>
          wrenchEql{
          [](const msgs::Wrench &_a, const msgs::Wrench &_b)
          {
            return math::equal(_a.torque().x(), _b.torque().x(), 1e-6) &&
                   math::equal(_a.torque().y(), _b.torque().y(), 1e-6) &&
                   math::equal(_a.torque().z(), _b.torque().z(), 1e-6) &&

                   math::equal(_a.force().x(), _b.force().x(), 1e-6) &&
                   math::equal(_a.force().y(), _b.force().y(), 1e-6) &&
                   math::equal(_a.force().z(), _b.force().z(), 1e-6);
          }};

  /// \brief Environment variable which holds paths to look for engine plugins
  public: std::string pluginPathEnv = "GZ_SIM_PHYSICS_ENGINE_PATH";

  //////////////////////////////////////////////////
  ////////////// Optional Features /////////////////
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////

  // Gravity
  public: struct GravityFeatureList
    : physics::FeatureList<
          MinimumFeatureList,
          physics::Gravity>{};

  // Slip Compliance

  /// \brief Feature list to process `FrictionPyramidSlipCompliance` components.
  public: struct FrictionPyramidSlipComplianceFeatureList
      : physics::FeatureList<
            MinimumFeatureList,
            physics::GetShapeFrictionPyramidSlipCompliance,
            physics::SetShapeFrictionPyramidSlipCompliance>{};
  //////////////////////////////////////////////////
  // Joints

  /// \brief Feature list to handle joints.
  public: struct JointFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetJointFromModel,
            physics::GetBasicJointProperties,
            physics::GetBasicJointState,
            physics::SetBasicJointState>{};

  /// \brief Feature list to construct joints
  public: struct ConstructSdfJointFeatureList : gz::physics::FeatureList<
            JointFeatureList,
            gz::physics::sdf::ConstructSdfJoint>{};

  /// \brief Feature list for mimic constraints
  public: struct MimicConstraintJointFeatureList : gz::physics::FeatureList<
            physics::SetMimicConstraintFeature>{};

  //////////////////////////////////////////////////
  // Detachable joints

  /// \brief Feature list to process `DetachableJoint` components.
  public: struct DetachableJointFeatureList : physics::FeatureList<
            JointFeatureList,
            physics::AttachFixedJointFeature,
            physics::DetachJointFeature,
            physics::SetJointTransformFromParentFeature>{};

  /// \brief Feature list for setting fixed joint to weld child to parent entity
  public: struct SetFixedJointWeldChildToParentFeatureList
            : physics::FeatureList<
            DetachableJointFeatureList,
            physics::SetFixedJointWeldChildToParentFeature>{};

  //////////////////////////////////////////////////
  // Joint transmitted wrench
  /// \brief Feature list for getting joint transmitted wrenches.
  public: struct JointGetTransmittedWrenchFeatureList : physics::FeatureList<
            physics::GetJointTransmittedWrench>{};

  //////////////////////////////////////////////////
  // Collisions

  /// \brief Feature list to handle collisions.
  public: struct CollisionFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::sdf::ConstructSdfCollision>{};

  /// \brief Feature list to handle contacts information.
  public: struct ContactFeatureList : physics::FeatureList<
            CollisionFeatureList,
            physics::GetContactsFromLastStepFeature>{};

  /// \brief Feature list to handle ray intersection information.
  public: struct RayIntersectionFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetRayIntersectionFromLastStepFeature>{};

  /// \brief Feature list for batched ray intersection queries.
  public: struct BatchRayIntersectionFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetBatchRayIntersectionFromLastStepFeature>{};

  /// \brief Per-entity cached batch ray query buffers to avoid per-step
  /// heap allocation. Input is cleared and refilled each step; output retains
  /// its allocated capacity so the physics engine can reuse it.
  public: struct BatchRayCacheEntry {
    using BatchWorld = physics::World3d<BatchRayIntersectionFeatureList>;
    std::vector<BatchWorld::RayQuery> input;
    BatchWorld::BatchedRayIntersectionData output;
  };
  public: std::unordered_map<Entity, BatchRayCacheEntry> batchRayCache;

  /// \brief Feature list to change contacts before they are applied to physics.
  public: struct SetContactPropertiesCallbackFeatureList :
            physics::FeatureList<
              ContactFeatureList,
              physics::SetContactPropertiesCallbackFeature>{};

  /// \brief Collision type with collision features.
  public: using ShapePtrType = physics::ShapePtr<
            physics::FeaturePolicy3d, CollisionFeatureList>;

  /// \brief World type with just the minimum features. Non-pointer.
  public: using WorldShapeType = physics::World<
            physics::FeaturePolicy3d, ContactFeatureList>;

  /// \brief Using ExtraContactData to expose contact Norm, Force & Depth
  public: using Policy = physics::FeaturePolicy3d;
  public: using GCFeature = physics::GetContactsFromLastStepFeature;
  public: using ExtraContactData = GCFeature::ExtraContactDataT<Policy>;

  /// \brief A contact is described by a contactPoint and the corresponding
  /// extraContactData which we bundle in a pair data structure
  public: using ContactData = std::pair<const WorldShapeType::ContactPoint *,
                                const ExtraContactData *>;
  /// \brief Each contact object we get from gz-physics contains the EntityPtrs
  /// of the two colliding entities and other data about the contact such as the
  /// position and extra contact date (wrench, normal and penetration depth).
  /// This map groups contacts so that it is easy to query all the
  /// contacts of one entity.
  public: using EntityContactMap = std::unordered_map<
            Entity, std::deque<ContactData>>;

  /// \brief msgs::Contacts equality comparison function.
  public: bool contactsEql(const msgs::Contacts &_msg,
                           const EntityContactMap &_map)
  {
    if (_msg.contact_size() != static_cast<int>(_map.size()))
    {
      return false;
    }

    auto mapIt = _map.begin();
    for (int i = 0; i < _msg.contact_size(); ++i, ++mapIt)
    {
      const auto &contactData = mapIt->second;
      if (_msg.contact(i).position_size() !=
          static_cast<int>(contactData.size()))
      {
        return false;
      }

      for (int j = 0; j < _msg.contact(i).position_size(); ++j)
      {
        const auto &contact = contactData[j];
        auto pos1 = _msg.contact(i).position(j);
        auto pos2 = contact.first->point;

        if (!math::equal(pos1.x(), pos2.x(), 1e-6) ||
            !math::equal(pos1.y(), pos2.y(), 1e-6) ||
            !math::equal(pos1.z(), pos2.z(), 1e-6))
        {
          return false;
        }

        if(contact.second != nullptr)
        {
          // Compare normals
          auto normal1 = _msg.contact(i).normal(j);
          auto normal2 = contact.second->normal;
          if (!math::equal(normal1.x(), normal2.x(), 1e-6) ||
              !math::equal(normal1.y(), normal2.y(), 1e-6) ||
              !math::equal(normal1.z(), normal2.z(), 1e-6))
          {
            return false;
          }
          // Compare body1 and body2 forces
          auto body1force1 = _msg.contact(i).wrench(j).body_1_wrench().force();
          auto body1force2 = contact.second->force;
          if (!math::equal(body1force1.x(), body1force2.x(), 1e-6) ||
              !math::equal(body1force1.y(), body1force2.y(), 1e-6) ||
              !math::equal(body1force1.z(), body1force2.z(), 1e-6))
          {
            return false;
          }

          auto body2force1 = _msg.contact(i).wrench(j).body_2_wrench().force();
          auto body2force2 = -contact.second->force;
          if (!math::equal(body2force1.x(), body2force2.x(), 1e-6) ||
              !math::equal(body2force1.y(), body2force2.y(), 1e-6) ||
              !math::equal(body2force1.z(), body2force2.z(), 1e-6))
          {
            return false;
          }
        }
      }
    }
    return true;
  }

  //////////////////////////////////////////////////
  // Collision filtering with bitmasks

  /// \brief Feature list to filter collisions with bitmasks.
  public: struct CollisionMaskFeatureList : physics::FeatureList<
          CollisionFeatureList,
          physics::CollisionFilterMaskFeature,
          physics::CategoryFilterMaskFeature>{};

  //////////////////////////////////////////////////
  // Link force
  /// \brief Feature list for applying forces to links.
  public: struct LinkForceFeatureList : physics::FeatureList<
            physics::AddLinkExternalForceTorque>{};


  //////////////////////////////////////////////////
  // Model Bounding box
  /// \brief Feature list for model bounding box.
  public: struct ModelBoundingBoxFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetModelBoundingBox>{};

  //////////////////////////////////////////////////
  // Static State
  /// \brief Feature list for model static state.
  public: struct StaticStateFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::ModelStaticState>{};

  //////////////////////////////////////////////////
  // Gravity Enabled
  /// \brief Feature list for gravity enabled.
  public: struct GravityEnabledFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GravityEnabled>{};

  //////////////////////////////////////////////////
  // Collision Enabled
  /// \brief Feature list for enabling and disabling model collisions.
  public: struct ModelCollisionEnabledFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::ModelCollisionEnabled>{};

  //////////////////////////////////////////////////
  // Link Bounding box
  /// \brief Feature list for model bounding box.
  public: struct LinkBoundingBoxFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetLinkBoundingBox>{};

  //////////////////////////////////////////////////
  // Joint velocity command
  /// \brief Feature list for set joint velocity command.
  public: struct JointVelocityCommandFeatureList : physics::FeatureList<
            physics::SetJointVelocityCommandFeature>{};


  //////////////////////////////////////////////////
  // Joint position limits command
  /// \brief Feature list for setting joint position limits.
  public: struct JointPositionLimitsCommandFeatureList : physics::FeatureList<
            physics::SetJointPositionLimitsFeature>{};


  //////////////////////////////////////////////////
  // Joint velocity limits command
  /// \brief Feature list for setting joint velocity limits.
  public: struct JointVelocityLimitsCommandFeatureList : physics::FeatureList<
            physics::SetJointVelocityLimitsFeature>{};


  //////////////////////////////////////////////////
  // Joint effort limits command
  /// \brief Feature list for setting joint effort limits.
  public: struct JointEffortLimitsCommandFeatureList : physics::FeatureList<
            physics::SetJointEffortLimitsFeature>{};


  //////////////////////////////////////////////////
  // World velocity command
  public: struct WorldVelocityCommandFeatureList :
            physics::FeatureList<
              physics::SetFreeGroupWorldVelocity>{};


  //////////////////////////////////////////////////
  // Meshes
  /// \brief Feature list for meshes.
  /// Include MinimumFeatureList so created collision can be automatically
  /// up-cast.
  public: struct MeshFeatureList : physics::FeatureList<
            CollisionFeatureList,
            physics::mesh::AttachMeshShapeFeature>{};

  //////////////////////////////////////////////////
  // Construct Links
  /// \brief Feature list for constructing links
  public: struct ConstructSdfLinkFeatureList : gz::physics::FeatureList<
            MinimumFeatureList,
            gz::physics::sdf::ConstructSdfLink>{};

  //////////////////////////////////////////////////
  // Heightmap
  /// \brief Feature list for heightmaps.
  /// Include MinimumFeatureList so created collision can be automatically
  /// up-cast.
  public: struct HeightmapFeatureList : gz::physics::FeatureList<
            CollisionFeatureList,
            physics::heightmap::AttachHeightmapShapeFeature>{};

  //////////////////////////////////////////////////
  // Collision detector
  /// \brief Feature list for setting and getting the collision detector
  public: struct CollisionDetectorFeatureList : gz::physics::FeatureList<
            gz::physics::CollisionDetector>{};

  //////////////////////////////////////////////////
  // Solver
  /// \brief Feature list for setting and getting the solver
  public: struct SolverFeatureList : gz::physics::FeatureList<
            gz::physics::Solver>{};

  //////////////////////////////////////////////////
  // CollisionPairMaxContacts
  /// \brief Feature list for setting and getting the max total contacts for
  /// collision pairs
  public: struct CollisionPairMaxContactsFeatureList :
            gz::physics::FeatureList<
            gz::physics::CollisionPairMaxContacts>{};

  //////////////////////////////////////////////////
  // Nested Models
  /// \brief Feature list to construct nested models
  public: struct NestedModelFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::sdf::ConstructSdfNestedModel>{};

  //////////////////////////////////////////////////
  // World models (used for world joints)
  public: struct WorldModelFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::WorldModelFeature>{};

  //////////////////////////////////////////////////
  /// \brief World EntityFeatureMap
  public: using WorldEntityMap = EntityFeatureMap3d<
          physics::World,
          MinimumFeatureList,
          CollisionFeatureList,
          ContactFeatureList,
          GravityFeatureList,
          RayIntersectionFeatureList,
          BatchRayIntersectionFeatureList,
          SetContactPropertiesCallbackFeatureList,
          NestedModelFeatureList,
          CollisionDetectorFeatureList,
          SolverFeatureList,
          WorldModelFeatureList,
          CollisionPairMaxContactsFeatureList
          >;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// gz-physics.
  public: WorldEntityMap entityWorldMap;

  /// \brief Model EntityFeatureMap
  public: using ModelEntityMap = EntityFeatureMap3d<
            physics::Model,
            MinimumFeatureList,
            JointFeatureList,
            ModelBoundingBoxFeatureList,
            NestedModelFeatureList,
            ConstructSdfLinkFeatureList,
            ConstructSdfJointFeatureList,
            StaticStateFeatureList,
            GravityEnabledFeatureList,
            ModelCollisionEnabledFeatureList>;

  /// \brief A map between model entity ids in the ECM to Model Entities in
  /// gz-physics.
  public: ModelEntityMap entityModelMap;

  /// \brief Link EntityFeatureMap
  public: using EntityLinkMap = EntityFeatureMap3d<
            physics::Link,
            MinimumFeatureList,
            DetachableJointFeatureList,
            CollisionFeatureList,
            HeightmapFeatureList,
            LinkForceFeatureList,
            MeshFeatureList,
            LinkBoundingBoxFeatureList,
            GravityEnabledFeatureList>;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// gz-physics.
  public: EntityLinkMap entityLinkMap;

  /// \brief Joint EntityFeatureMap
  public: using EntityJointMap = EntityFeatureMap3d<
            physics::Joint,
            JointFeatureList,
            DetachableJointFeatureList,
            SetFixedJointWeldChildToParentFeatureList,
            MimicConstraintJointFeatureList,
            JointVelocityCommandFeatureList,
            JointGetTransmittedWrenchFeatureList,
            JointPositionLimitsCommandFeatureList,
            JointVelocityLimitsCommandFeatureList,
            JointEffortLimitsCommandFeatureList
            >;

  /// \brief A map between joint entity ids in the ECM to Joint Entities in
  /// gz-physics
  public: EntityJointMap entityJointMap;

  /// \brief Collision EntityFeatureMap
  public: using EntityCollisionMap = EntityFeatureMap3d<
            physics::Shape,
            CollisionFeatureList,
            ContactFeatureList,
            CollisionMaskFeatureList,
            FrictionPyramidSlipComplianceFeatureList
            >;

  /// \brief A map between collision entity ids in the ECM to Shape Entities in
  /// gz-physics.
  public: EntityCollisionMap entityCollisionMap;

  /// \brief FreeGroup EntityFeatureMap
  public: using EntityFreeGroupMap = EntityFeatureMap3d<
            physics::FreeGroup,
            MinimumFeatureList,
            WorldVelocityCommandFeatureList
            >;

  /// \brief A map between collision entity ids in the ECM to FreeGroup Entities
  /// in gz-physics.
  public: EntityFreeGroupMap entityFreeGroupMap;

  /// \brief Event manager from simulation runner.
  public: EventManager *eventManager = nullptr;

  /// \brief Keep track of what entities use customized contact surfaces.
  /// Map keys are expected to be world entities so that we keep a set of
  /// entities with customizations per world.
  public: std::unordered_map<Entity, std::unordered_set<Entity>>
    customContactSurfaceEntities;

  /// \brief Set of links that were added to an existing model. This set
  /// is used to track links that were added to an existing model, such as
  /// through the GUI model editor, so that we can avoid premature creation
  /// of links and collision elements. This also lets us suppress some
  /// invalid error messages.
  public: std::set<Entity> linkAddedToModel;

  /// \brief Set of joints that were added to an existing model. This set
  /// is used to track joints that were added to an existing model, such as
  /// through the GUI model editor, so that we can avoid premature creation
  /// of joints. This also lets us suppress some invalid error messages.
  public: std::set<Entity> jointAddedToModel;

  /// \brief Flag to store whether the names of colliding entities should
  /// be populated in the contact points.
  public: bool contactsEntityNames = true;

  /// \brief Cached physics output, to reduce allocations / deallocations
  physics::ForwardStep::Output stepOutput;
};

#endif  // GZ_SIM_SYSTEMS_PHYSICS_PHYSICSPRIVATE_HH_
