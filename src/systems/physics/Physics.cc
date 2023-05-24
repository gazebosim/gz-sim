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

#include "Physics.hh"

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/Utility.hh>

#include <algorithm>
#include <iostream>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
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
#include <gz/physics/RelativeQuantity.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/BoxShape.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/physics/CylinderShape.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/GetEntities.hh>
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
#include <gz/plugin/Loader.hh>
#include <gz/plugin/PluginPtr.hh>
#include <gz/plugin/Register.hh>

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
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

// Components
#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
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
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
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

  /// \brief IDs of the ContactSurfaceHandler callbacks registered for worlds
  public: std::unordered_map<Entity, std::string> worldContactCallbackIDs;

  /// \brief used to store whether physics objects have been created.
  public: bool initialized = false;

  /// \brief Pointer to the underlying gz-physics Engine entity.
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

  /// \brief msgs::Contacts equality comparison function.
  public: std::function<bool(const msgs::Contacts &,
          const msgs::Contacts &)>
          contactsEql { [](const msgs::Contacts &_a,
                          const msgs::Contacts &_b)
                    {
                      if (_a.contact_size() != _b.contact_size())
                      {
                        return false;
                      }

                      for (int i = 0; i < _a.contact_size(); ++i)
                      {
                        if (_a.contact(i).position_size() !=
                            _b.contact(i).position_size())
                        {
                          return false;
                        }

                        for (int j = 0; j < _a.contact(i).position_size();
                          ++j)
                        {
                          auto pos1 = _a.contact(i).position(j);
                          auto pos2 = _b.contact(i).position(j);

                          if (!math::equal(pos1.x(), pos2.x(), 1e-6) ||
                              !math::equal(pos1.y(), pos2.y(), 1e-6) ||
                              !math::equal(pos1.z(), pos2.z(), 1e-6))
                          {
                            return false;
                          }
                        }
                      }
                      return true;
                    }};
  /// \brief msgs::Contacts equality comparison function.
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
  public: std::string pluginPathEnvDeprecated = \
    "IGN_GAZEBO_PHYSICS_ENGINE_PATH";

  //////////////////////////////////////////////////
  ////////////// Optional Features /////////////////
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
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

  //////////////////////////////////////////////////
  // Detachable joints

  /// \brief Feature list to process `DetachableJoint` components.
  public: struct DetachableJointFeatureList : physics::FeatureList<
            JointFeatureList,
            physics::AttachFixedJointFeature,
            physics::DetachJointFeature,
            physics::SetJointTransformFromParentFeature>{};

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

  //////////////////////////////////////////////////
  // Collision filtering with bitmasks

  /// \brief Feature list to filter collisions with bitmasks.
  public: struct CollisionMaskFeatureList : physics::FeatureList<
          CollisionFeatureList,
          physics::CollisionFilterMaskFeature>{};

  //////////////////////////////////////////////////
  // Link force
  /// \brief Feature list for applying forces to links.
  public: struct LinkForceFeatureList : physics::FeatureList<
            physics::AddLinkExternalForceTorque>{};


  //////////////////////////////////////////////////
  // Bounding box
  /// \brief Feature list for model bounding box.
  public: struct BoundingBoxFeatureList : physics::FeatureList<
            MinimumFeatureList,
            physics::GetModelBoundingBox>{};


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
          SetContactPropertiesCallbackFeatureList,
          NestedModelFeatureList,
          CollisionDetectorFeatureList,
          SolverFeatureList,
          WorldModelFeatureList
          >;

  /// \brief A map between world entity ids in the ECM to World Entities in
  /// gz-physics.
  public: WorldEntityMap entityWorldMap;

  /// \brief Model EntityFeatureMap
  public: using ModelEntityMap = EntityFeatureMap3d<
            physics::Model,
            MinimumFeatureList,
            JointFeatureList,
            BoundingBoxFeatureList,
            NestedModelFeatureList,
            ConstructSdfLinkFeatureList,
            ConstructSdfJointFeatureList>;

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
            MeshFeatureList>;

  /// \brief A map between link entity ids in the ECM to Link Entities in
  /// gz-physics.
  public: EntityLinkMap entityLinkMap;

  /// \brief Joint EntityFeatureMap
  public: using EntityJointMap = EntityFeatureMap3d<
            physics::Joint,
            JointFeatureList,
            DetachableJointFeatureList,
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
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
}

//////////////////////////////////////////////////
void Physics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
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
    pluginLib = "gz-physics-dartsim-plugin";
  }

  // Deprecated: accept ignition-prefixed engines
  std::string deprecatedPrefix{"ignition"};
  auto pos = pluginLib.find(deprecatedPrefix);
  if (pos != std::string::npos)
  {
    auto msg = "Trying to load deprecated plugin [" + pluginLib + "]. Use [";
    pluginLib.replace(pos, deprecatedPrefix.size(), "gz");
    gzwarn << msg << pluginLib << "] instead." << std::endl;
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

  // Check if entity names should be populated in contact points.
  auto contactsElement = _sdf->FindElement("contacts");
  if (contactsElement)
  {
    this->dataPtr->contactsEntityNames = contactsElement->Get<bool>(
      "include_entity_names", true).first;
  }

  // Find engine shared library
  // Look in:
  // * Paths from environment variable
  // * Engines installed with gz-physics
  common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv(this->dataPtr->pluginPathEnv);
  systemPaths.AddPluginPaths({GZ_PHYSICS_ENGINE_INSTALL_DIR});

  auto pathToLib = systemPaths.FindSharedLibrary(pluginLib);
  if (pathToLib.empty())
  {
    // Try deprecated environment variable
    // TODO(CH3): Deprecated. Remove on tock.
    common::SystemPaths systemPathsDep;
    systemPathsDep.SetPluginPathEnv(this->dataPtr->pluginPathEnvDeprecated);
    pathToLib = systemPathsDep.FindSharedLibrary(pluginLib);

    if (pathToLib.empty())
    {
      gzerr << "Failed to find plugin [" << pluginLib
      << "]. Have you checked the " << this->dataPtr->pluginPathEnv
      << " environment variable?" << std::endl;

      return;
    }
    else
    {
      gzwarn << "Found plugin [" << pluginLib
             << "] using deprecated environment variable ["
             << this->dataPtr->pluginPathEnvDeprecated << "]. Please use ["
             << this->dataPtr->pluginPathEnv << "] instead." << std::endl;
    }
  }

  // Load engine plugin
  plugin::Loader pluginLoader;
  auto plugins = pluginLoader.LoadLib(pathToLib);
  if (plugins.empty())
  {
    gzerr << "Unable to load the [" << pathToLib << "] library.\n";
    return;
  }

  auto classNames = pluginLoader.PluginsImplementing<
      physics::ForwardStep::Implementation<
      physics::FeaturePolicy3d>>();
  if (classNames.empty())
  {
    gzerr << "No physics plugins found in library [" << pathToLib << "]."
           << std::endl;
    return;
  }

  // Get the first plugin that works
  for (auto className : classNames)
  {
    auto plugin = pluginLoader.Instantiate(className);

    if (!plugin)
    {
      gzwarn << "Failed to instantiate [" << className << "] from ["
              << pathToLib << "]" << std::endl;
      continue;
    }

    this->dataPtr->engine = physics::RequestEngine<
      physics::FeaturePolicy3d,
      PhysicsPrivate::MinimumFeatureList>::From(plugin);

    if (nullptr != this->dataPtr->engine)
    {
      gzdbg << "Loaded [" << className << "] from library ["
             << pathToLib << "]" << std::endl;
      break;
    }

    auto missingFeatures = physics::RequestEngine<
        physics::FeaturePolicy3d,
        PhysicsPrivate::MinimumFeatureList>::MissingFeatureNames(plugin);

    std::stringstream msg;
    msg << "Plugin [" << className << "] misses required features:"
        << std::endl;
    for (auto feature : missingFeatures)
    {
      msg << "- " << feature << std::endl;
    }
    gzwarn << msg.str();
  }

  if (nullptr == this->dataPtr->engine)
  {
    gzerr << "Failed to load a valid physics engine from [" << pathToLib
           << "]."
           << std::endl;
    return;
  }

  this->dataPtr->eventManager = &_eventMgr;
}

//////////////////////////////////////////////////
Physics::~Physics() = default;

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("Physics::Update");

  if (this->dataPtr->engine)
  {
    this->dataPtr->CreatePhysicsEntities(_ecm);
    this->dataPtr->UpdatePhysics(_ecm);
    gz::physics::ForwardStep::Output stepOutput;
    // Only step if not paused.
    if (!_info.paused)
    {
      stepOutput = this->dataPtr->Step(_info.dt);
    }
    auto changedLinks = this->dataPtr->ChangedLinks(_ecm, stepOutput);
    this->dataPtr->UpdateSim(_ecm, changedLinks);

    // Entities scheduled to be removed should be removed from physics after the
    // simulation step. Otherwise, since the to-be-removed entity still shows up
    // in the ECM::Each the UpdatePhysics and UpdateSim calls will have an error
    this->dataPtr->RemovePhysicsEntities(_ecm);
  }
}

//////////////////////////////////////////////////
void Physics::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("Physics::Reset");

  if (this->dataPtr->engine)
  {
    gzdbg << "Resetting Physics\n";
    this->dataPtr->ResetPhysics(_ecm);
  }
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreatePhysicsEntities(const EntityComponentManager &_ecm,
                                           bool _warnIfEntityExists)
{
  // Clear the set of links that were added to a model.
  this->linkAddedToModel.clear();
  this->jointAddedToModel.clear();

  this->CreateWorldEntities(_ecm, _warnIfEntityExists);
  this->CreateModelEntities(_ecm, _warnIfEntityExists);
  this->CreateLinkEntities(_ecm, _warnIfEntityExists);
  // We don't need to add visuals to the physics engine.
  this->CreateCollisionEntities(_ecm, _warnIfEntityExists);
  this->CreateJointEntities(_ecm, _warnIfEntityExists);
  this->CreateBatteryEntities(_ecm);
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateWorldEntities(const EntityComponentManager &_ecm,
                                         bool _warnIfEntityExists)
{
  // Get all the new worlds
  _ecm.EachNew<components::World, components::Name, components::Gravity>(
      [&](const Entity &_entity,
        const components::World * /* _world */,
        const components::Name *_name,
        const components::Gravity *_gravity)->bool
      {
        // Check if world already exists
        if (this->entityWorldMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "World entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }

        sdf::World world;
        world.SetName(_name->Data());
        world.SetGravity(_gravity->Data());
        auto worldPtrPhys = this->engine->ConstructWorld(world);
        this->entityWorldMap.AddEntity(_entity, worldPtrPhys);

        // Optional world features
        auto collisionDetectorComp =
            _ecm.Component<components::PhysicsCollisionDetector>(_entity);
        if (collisionDetectorComp)
        {
          auto collisionDetectorFeature =
              this->entityWorldMap.EntityCast<CollisionDetectorFeatureList>(
              _entity);
          if (!collisionDetectorFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to set physics options, but the "
                     << "phyiscs engine doesn't support feature "
                     << "[CollisionDetectorFeature]. Options will be ignored."
                     << std::endl;
              informed = true;
            }
          }
          else
          {
            collisionDetectorFeature->SetCollisionDetector(
                collisionDetectorComp->Data());
          }
        }

        auto solverComp =
            _ecm.Component<components::PhysicsSolver>(_entity);
        if (solverComp)
        {
          auto solverFeature =
              this->entityWorldMap.EntityCast<SolverFeatureList>(
              _entity);
          if (!solverFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to set physics options, but the "
                     << "phyiscs engine doesn't support feature "
                     << "[SolverFeature]. Options will be ignored."
                     << std::endl;
              informed = true;
            }
          }
          else
          {
            solverFeature->SetSolver(solverComp->Data());
          }
        }

        // World Model proxy (used for joints directly under <world> in SDF)
        auto worldModelFeature =
            this->entityWorldMap.EntityCast<WorldModelFeatureList>(_entity);
        if (worldModelFeature)
        {
          auto modelPtrPhys = worldModelFeature->GetWorldModel();
          this->entityModelMap.AddEntity(_entity, modelPtrPhys);
        }

        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateModelEntities(const EntityComponentManager &_ecm,
                                         bool _warnIfEntityExists)
{
  _ecm.EachNew<components::Model, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        if (_ecm.EntityHasComponentType(_entity, components::Recreate::typeId))
          return true;

        // Check if model already exists
        if (this->entityModelMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "Model entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }
        // TODO(anyone) Don't load models unless they have collisions

        // Check if parent world / model exists
        sdf::Model model;
        if (const auto *modelSdfComp =
            _ecm.Component<components::ModelSdf>(_entity))
        {
          model = modelSdfComp->Data();
        }

        // Component values should override whatever values were put into the
        // ModelSdf component.
        model.SetName(_name->Data());
        model.SetRawPose(_pose->Data());
        model.SetPoseRelativeTo("");

        sdf::Root root;
        root.SetModel(model);
        root.UpdateGraphs();

        auto staticComp = _ecm.Component<components::Static>(_entity);
        if (staticComp && staticComp->Data())
        {
          model.SetStatic(staticComp->Data());
          this->staticEntities.insert(_entity);
        }
        auto selfCollideComp = _ecm.Component<components::SelfCollide>(_entity);
        if (selfCollideComp && selfCollideComp ->Data())
        {
          model.SetSelfCollide(selfCollideComp->Data());
        }

        // check if parent is a world
        if (auto worldPtrPhys =
                this->entityWorldMap.Get(_parent->Data()))
        {
          // Use the ConstructNestedModel feature for nested models
          if (model.ModelCount() > 0)
          {
            auto nestedModelFeature =
                this->entityWorldMap.EntityCast<NestedModelFeatureList>(
                    _parent->Data());
            if (!nestedModelFeature)
            {
              static bool informed{false};
              if (!informed)
              {
                gzdbg << "Attempting to construct nested models, but the "
                       << "phyiscs engine doesn't support feature "
                       << "[ConstructSdfNestedModelFeature]. "
                       << "Nested model will be ignored."
                       << std::endl;
                informed = true;
              }
              return true;
            }
            auto modelPtrPhys =
              nestedModelFeature->ConstructNestedModel(*root.Model());
            if (modelPtrPhys)
            {
              this->entityModelMap.AddEntity(_entity, modelPtrPhys);
              this->topLevelModelMap.insert(std::make_pair(_entity,
                  topLevelModel(_entity, _ecm)));
            }
          }
          else
          {
            auto modelPtrPhys = worldPtrPhys->ConstructModel(*root.Model());
            if (modelPtrPhys)
            {
              this->entityModelMap.AddEntity(_entity, modelPtrPhys);
              this->topLevelModelMap.insert(std::make_pair(_entity,
                  topLevelModel(_entity, _ecm)));
            }
          }
        }
        // check if parent is a model (nested model)
        else
        {
          if (auto parentPtrPhys = this->entityModelMap.Get(_parent->Data()))
          {
            auto nestedModelFeature =
                this->entityModelMap.EntityCast<NestedModelFeatureList>(
                    _parent->Data());
            if (!nestedModelFeature)
            {
              static bool informed{false};
              if (!informed)
              {
                gzdbg << "Attempting to construct nested models, but the "
                       << "physics engine doesn't support feature "
                       << "[ConstructSdfNestedModelFeature]. "
                       << "Nested model will be ignored."
                       << std::endl;
                informed = true;
              }
              return true;
            }

            // override static property only if parent is static.
            auto parentStaticComp =
              _ecm.Component<components::Static>(_parent->Data());
            if (parentStaticComp && parentStaticComp->Data())
            {
              model.SetStatic(true);
              this->staticEntities.insert(_entity);
            }

            auto modelPtrPhys = nestedModelFeature->ConstructNestedModel(model);
            if (modelPtrPhys)
            {
              this->entityModelMap.AddEntity(_entity, modelPtrPhys);
              this->topLevelModelMap.insert(std::make_pair(_entity,
                  topLevelModel(_entity, _ecm)));
            }
            else
            {
              gzerr << "Model: '" << _name->Data() << "' not loaded. "
                     << "Failed to create nested model."
                     << std::endl;
            }
          }
          else
          {
            gzwarn << "Model's parent entity [" << _parent->Data()
                    << "] not found on world / model map." << std::endl;
            return true;
          }
        }

        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateLinkEntities(const EntityComponentManager &_ecm,
                                        bool _warnIfEntityExists)
{
  _ecm.EachNew<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Link * /* _link */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        // If the parent model is scheduled for recreation, then do not
        // try to create a new link. This situation can occur when a link
        // is added to a model from the GUI model editor.
        if (_ecm.EntityHasComponentType(_parent->Data(),
              components::Recreate::typeId))
        {
          // Add this entity to the set of newly added links to existing
          // models.
          this->linkAddedToModel.insert(_entity);
          return true;
        }

        // Check if link already exists
        if (this->entityLinkMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "Link entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }

        // TODO(anyone) Don't load links unless they have collisions

        // Check if parent model exists
        if (!this->entityModelMap.HasEntity(_parent->Data()))
        {
          gzwarn << "Link's parent entity [" << _parent->Data()
                  << "] not found on model map." << std::endl;
          return true;
        }
        auto basicModelPtrPhys = this->entityModelMap.Get(_parent->Data());

        if (const auto existingLink = basicModelPtrPhys->GetLink(_name->Data()))
        {
          // No need to create this link because it was already created when
          // parsing the model (links in models are required to have unique
          // names). Instead we will register its existence and move along.
          this->entityLinkMap.AddEntity(_entity, existingLink);
          this->topLevelModelMap.insert(
            std::make_pair(_entity, topLevelModel(_entity, _ecm)));
          return true;
        }

        auto modelPtrPhys =
            this->entityModelMap
              .EntityCast<ConstructSdfLinkFeatureList>(_parent->Data());

        if (!modelPtrPhys)
        {
          gzwarn << "Cannot create a new link [" << _name->Data() << "] "
                 << "because the physics engine plugin does not support "
                 << "link construction during runtime" << std::endl;
          return true;
        }

        sdf::Link link;
        link.SetName(_name->Data());
        link.SetRawPose(_pose->Data());
        link.SetPoseRelativeTo("");

        if (this->staticEntities.find(_parent->Data()) !=
            this->staticEntities.end())
        {
          this->staticEntities.insert(_entity);
        }

        // get link inertial
        auto inertial = _ecm.Component<components::Inertial>(_entity);
        if (inertial)
        {
          link.SetInertial(inertial->Data());
        }

        auto constructLinkFeature =
          this->entityModelMap.EntityCast<ConstructSdfLinkFeatureList>(
            _parent->Data());

        if (!constructLinkFeature)
        {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to construct sdf link, but the "
                     << "physics engine doesn't support feature "
                     << "[ConstructSdfLinkFeature]." << std::endl;
              informed = true;
            }
            return true;
        }

        auto linkPtrPhys = constructLinkFeature->ConstructLink(link);
        this->entityLinkMap.AddEntity(_entity, linkPtrPhys);
        this->topLevelModelMap.insert(std::make_pair(_entity,
            topLevelModel(_entity, _ecm)));

        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateCollisionEntities(const EntityComponentManager &_ecm,
                                             bool _warnIfEntityExists)
{
  _ecm.EachNew<components::Collision, components::Name, components::Pose,
            components::Geometry, components::CollisionElement,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Collision *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::Geometry *_geom,
          const components::CollisionElement *_collElement,
          const components::ParentEntity *_parent) -> bool
      {
        // Check to see if this collision's parent is a link that was
        // not created because the parent model is marked for recreation.
        if (this->linkAddedToModel.find(_parent->Data()) !=
            this->linkAddedToModel.end())
        {
          return true;
        }

        if (this->entityCollisionMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "Collision entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }

        // Check if parent link exists
        if (!this->entityLinkMap.HasEntity(_parent->Data()))
        {
          gzwarn << "Collision's parent entity [" << _parent->Data()
                  << "] not found on link map." << std::endl;
          return true;
        }
        auto linkPtrPhys = this->entityLinkMap.Get(_parent->Data());

        if (const auto existingShape = linkPtrPhys->GetShape(_name->Data()))
        {
          // No need to create this collision shape because it was already
          // created when parsing the model.
          auto linkCollisionFeature =
              this->entityLinkMap.EntityCast<CollisionFeatureList>(
                  _parent->Data());
          this->entityCollisionMap.AddEntity(
            _entity, linkCollisionFeature->GetShape(_name->Data()));
          this->topLevelModelMap.insert(
            std::make_pair(_entity, topLevelModel(_entity, _ecm)));
          return true;
        }

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
            gzwarn << "Mesh geometry for collision [" << _name->Data()
                    << "] missing mesh shape." << std::endl;
            return true;
          }

          auto &meshManager = *common::MeshManager::Instance();
          auto fullPath = asFullPath(meshSdf->Uri(), meshSdf->FilePath());
          auto *mesh = meshManager.Load(fullPath);
          if (nullptr == mesh)
          {
            gzwarn << "Failed to load mesh from [" << fullPath
                    << "]." << std::endl;
            return true;
          }

          auto linkMeshFeature =
              this->entityLinkMap.EntityCast<MeshFeatureList>(_parent->Data());
          if (!linkMeshFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to process mesh geometries, but the physics"
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
        else if (_geom->Data().Type() == sdf::GeometryType::HEIGHTMAP)
        {
          auto linkHeightmapFeature =
              this->entityLinkMap.EntityCast<HeightmapFeatureList>(
                  _parent->Data());
          if (!linkHeightmapFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to process heightmap geometries, but the "
                     << "physics engine doesn't support feature "
                     << "[AttachHeightmapShapeFeature]. Heightmaps will be "
                     << "ignored." << std::endl;
              informed = true;
            }
            return true;
          }

          auto heightmapSdf = _geom->Data().HeightmapShape();
          if (nullptr == heightmapSdf)
          {
            gzwarn << "Heightmap geometry for collision [" << _name->Data()
                    << "] missing heightmap shape." << std::endl;
            return true;
          }

          auto fullPath = common::findFile(asFullPath(heightmapSdf->Uri(),
              heightmapSdf->FilePath()));
          if (fullPath.empty())
          {
            gzerr << "Heightmap geometry missing URI" << std::endl;
            return true;
          }

          std::shared_ptr<common::HeightmapData> data;
          std::string lowerFullPath = common::lowercase(fullPath);
          // check if heightmap is an image
          if (common::EndsWith(lowerFullPath, ".png")
              || common::EndsWith(lowerFullPath, ".jpg")
              || common::EndsWith(lowerFullPath, ".jpeg"))
          {
            auto img = std::make_shared<common::ImageHeightmap>();
            if (img->Load(fullPath) < 0)
            {
              gzerr << "Failed to load heightmap image data from ["
                     << fullPath << "]" << std::endl;
              return true;
            }
            data = img;
          }
          // DEM
          else
          {
            auto worldEntity = _ecm.EntityByComponents(components::World());
            auto sphericalCoordinatesComponent =
              _ecm.Component<components::SphericalCoordinates>(
                worldEntity);

            auto dem = std::make_shared<common::Dem>();
            if (sphericalCoordinatesComponent)
            {
              dem->SetSphericalCoordinates(
                  sphericalCoordinatesComponent->Data());
            }
            if (dem->Load(fullPath) < 0)
            {
              gzerr << "Failed to load heightmap dem data from ["
                     << fullPath << "]" << std::endl;
              return true;
            }
            data = dem;
          }

          collisionPtrPhys = linkHeightmapFeature->AttachHeightmapShape(
              _name->Data(),
              *data,
              math::eigen3::convert(_pose->Data()),
              math::eigen3::convert(heightmapSdf->Size()),
              heightmapSdf->Sampling());
        }
        else if (_geom->Data().Type() == sdf::GeometryType::POLYLINE)
        {
          auto polylineSdf = _geom->Data().PolylineShape();
          if (polylineSdf.empty())
          {
            gzwarn << "Polyline geometry for collision [" << _name->Data()
                    << "] missing polylines." << std::endl;
            return true;
          }

          std::vector<std::vector<math::Vector2d>> vertices;
          for (const auto &polyline : _geom->Data().PolylineShape())
          {
            vertices.push_back(polyline.Points());
          }

          std::string name("POLYLINE_" + common::Uuid().String());
          auto meshManager = common::MeshManager::Instance();
          meshManager->CreateExtrudedPolyline(name, vertices,
              _geom->Data().PolylineShape()[0].Height());

          auto polyline = meshManager->MeshByName(name);
          if (nullptr == polyline)
          {
            gzwarn << "Failed to create polyline for collision ["
                    << _name->Data() << "]." << std::endl;
            return true;
          }

          auto linkMeshFeature =
              this->entityLinkMap.EntityCast<MeshFeatureList>(_parent->Data());
          if (!linkMeshFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to process polyline geometries, but the"
                     << " physics engine doesn't support feature "
                     << "[AttachMeshShapeFeature]. Polylines will be ignored."
                     << std::endl;
              informed = true;
            }
            return true;
          }

          collisionPtrPhys = linkMeshFeature->AttachMeshShape(_name->Data(),
              *polyline,
              math::eigen3::convert(_pose->Data()));
        }
        else
        {
          auto linkCollisionFeature =
              this->entityLinkMap.EntityCast<CollisionFeatureList>(
                  _parent->Data());
          if (!linkCollisionFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to process collisions, but the physics "
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

        if (nullptr == collisionPtrPhys)
        {
          gzdbg << "Failed to create collision [" << _name->Data()
                 << "]. Does the physics engine support geometries of type ["
                 << static_cast<int>(_geom->Data().Type()) << "]?" << std::endl;
          return true;
        }

        this->entityCollisionMap.AddEntity(_entity, collisionPtrPhys);

        // Check that the physics engine has a filter mask feature
        // Set the collide_bitmask if it does
        auto filterMaskFeature =
            this->entityCollisionMap.EntityCast<CollisionMaskFeatureList>(
                _entity);
        if (filterMaskFeature)
        {
          filterMaskFeature->SetCollisionFilterMask(collideBitmask);
        }
        else
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set collision bitmasks, but the physics "
                   << "engine doesn't support feature [CollisionFilterMask]. "
                   << "Collision bitmasks will be ignored." << std::endl;
            informed = true;
          }
        }

        this->topLevelModelMap.insert(std::make_pair(_entity,
            topLevelModel(_entity, _ecm)));
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateJointEntities(const EntityComponentManager &_ecm,
                                         bool _warnIfEntityExists)
{
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
        // If the parent model is scheduled for recreation, then do not
        // try to create a new joint. This situation can occur when a joint
        // is added to a model from the GUI model editor.
        if (_ecm.EntityHasComponentType(_parentModel->Data(),
              components::Recreate::typeId))
        {
          // Add this entity to the set of newly added joints to existing
          // models.
          this->jointAddedToModel.insert(_entity);
          return true;
        }

        // Check if joint already exists
        if (this->entityJointMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "Joint entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }

        // Check if parent model exists
        if (!this->entityModelMap.HasEntity(_parentModel->Data()))
        {
          gzerr << "Joint's parent model entity [" << _parentModel->Data()
                  << "] not found on model map." << std::endl;
          return true;
        }

        auto basicModelPtrPhys = this->entityModelMap
            .EntityCast<JointFeatureList>(_parentModel->Data());
        if (!basicModelPtrPhys)
        {
          static bool informed{false};
          if (!informed)
          {
            gzerr << "Attempting to create a new joint [" <<_name->Data()
                  << "] but the chosen physics engine does not support the "
                  << "minimal joint features, so no joints will be created."
                  << std::endl;
            informed = true;
          }

          // Skip all other attempts to create joints
          return false;
        }

        if (const auto existingJoint =
            basicModelPtrPhys->GetJoint(_name->Data()))
        {
          // No need to create this joint because it was already created when
          // parsing the model.
          this->entityJointMap.AddEntity(_entity, existingJoint);
          this->topLevelModelMap.insert(
            std::make_pair(_entity, topLevelModel(_entity, _ecm)));
          return true;
        }

        auto modelPtrPhys =
            this->entityModelMap.EntityCast<ConstructSdfJointFeatureList>(
                _parentModel->Data());
        if (!modelPtrPhys)
        {
          gzerr << "Attempting to create a new joint [" << _name->Data()
                << "], but the physics engine doesn't support constructing "
                << "joints at runtime." << std::endl;
          return true;
        }

        sdf::Joint joint;
        joint.SetName(_name->Data());
        joint.SetType(_jointType->Data());
        joint.SetRawPose(_pose->Data());
        joint.SetThreadPitch(_threadPitch->Data());

        joint.SetParentName(_parentLinkName->Data());
        joint.SetChildName(_childLinkName->Data());

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
        auto jointPtrPhys = modelPtrPhys->ConstructJoint(joint);

        if (jointPtrPhys.Valid())
        {
          // Some joints may not be supported, so only add them to the map if
          // the physics entity is valid
          this->entityJointMap.AddEntity(_entity, jointPtrPhys);
          this->topLevelModelMap.insert(std::make_pair(_entity,
              topLevelModel(_entity, _ecm)));
        }
        return true;
      });

  // Detachable joints
  _ecm.EachNew<components::DetachableJoint>(
      [&](const Entity &_entity,
          const components::DetachableJoint *_jointInfo) -> bool
      {
        if (_jointInfo->Data().jointType != "fixed")
        {
          gzerr << "Detachable joint type [" << _jointInfo->Data().jointType
                 << "] is currently not supported" << std::endl;
          return true;
        }
        // Check if joint already exists
        if (this->entityJointMap.HasEntity(_entity))
        {
          if (_warnIfEntityExists)
          {
            gzwarn << "Joint entity [" << _entity
                    << "] marked as new, but it's already on the map."
                    << std::endl;
          }
          return true;
        }

        // Check if the link entities exist in the physics engine
        auto parentLinkPhys =
            this->entityLinkMap.Get(_jointInfo->Data().parentLink);
        if (!parentLinkPhys)
        {
          gzerr << "DetachableJoint's parent link entity ["
                << _jointInfo->Data().parentLink << "] not found in link map."
                << std::endl;
          return true;
        }

        auto childLinkEntity = _jointInfo->Data().childLink;

        // Get child link
        auto childLinkPhys = this->entityLinkMap.Get(childLinkEntity);
        if (!childLinkPhys)
        {
          gzerr << "Failed to find joint's child link [" << childLinkEntity
                << "]." << std::endl;
          return true;
        }

        auto childLinkDetachableJointFeature =
            this->entityLinkMap.EntityCast<DetachableJointFeatureList>(
                childLinkEntity);
        if (!childLinkDetachableJointFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzerr << "Attempting to create a detachable joint, but the physics"
                   << " engine doesn't support feature "
                   << "[AttachFixedJointFeature]. Detachable joints will be "
                   << "ignored." << std::endl;
            informed = true;
          }

          // Break Each call since no DetachableJoints can be processed
          return false;
        }

        const auto poseParent =
            parentLinkPhys->FrameDataRelativeToWorld().pose;
        const auto poseChild =
            childLinkDetachableJointFeature->FrameDataRelativeToWorld().pose;

        // Pose of child relative to parent
        auto poseParentChild = poseParent.inverse() * poseChild;
        auto jointPtrPhys =
            childLinkDetachableJointFeature->AttachFixedJoint(parentLinkPhys);
        if (jointPtrPhys.Valid())
        {
          // We let the joint be at the origin of the child link.
          jointPtrPhys->SetTransformFromParent(poseParentChild);

          gzdbg << "Creating detachable joint [" << _entity << "]"
                 << std::endl;
          this->entityJointMap.AddEntity(_entity, jointPtrPhys);
          this->topLevelModelMap.insert(std::make_pair(_entity,
              topLevelModel(_entity, _ecm)));
        }
        else
        {
          gzerr << "DetachableJoint could not be created." << std::endl;
        }
        return true;
      });

  // The components are removed after each update, so we want to process all
  // components in every update.
  _ecm.Each<components::EnableContactSurfaceCustomization,
            components::Collision, components::Name>(
      [&](const Entity & _entity,
          const components::EnableContactSurfaceCustomization *_enable,
          const components::Collision */*_collision*/,
          const components::Name *_name) -> bool
      {
        const auto world = worldEntity(_entity, _ecm);
        if (_enable->Data())
        {
          if (this->customContactSurfaceEntities[world].empty())
          {
            this->EnableContactSurfaceCustomization(world);
          }
          this->customContactSurfaceEntities[world].insert(_entity);
          gzmsg << "Enabling contact surface customization for collision ["
                 << _name->Data() << "]" << std::endl;
        }
        else
        {
          if (this->customContactSurfaceEntities[world].erase(_entity) > 0)
          {
            gzmsg << "Disabling contact surface customization for collision ["
                   << _name->Data() << "]" << std::endl;
            if (this->customContactSurfaceEntities[world].empty())
            {
              this->DisableContactSurfaceCustomization(world);
            }
          }
        }
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::CreateBatteryEntities(const EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::BatterySoC>(
      [&](const Entity & _entity, const components::BatterySoC *)->bool
      {
        // Parent entity of battery is model entity
        this->entityOffMap.insert(std::make_pair(
          _ecm.ParentEntity(_entity), false));
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::RemovePhysicsEntities(const EntityComponentManager &_ecm)
{
  // Assume the world will not be erased
  // Only removing models is supported by gz-physics right now so we only
  // remove links, joints and collisions if they are children of the removed
  // model.
  // We assume the links, joints and collisions will be removed from the
  // physics engine when the containing model gets removed so, here, we only
  // remove the entities from the gazebo entity->physics entity map.
  _ecm.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *
          /* _model */) -> bool
      {
        const auto world = worldEntity(_ecm);
        // Remove model if found
        if (auto modelPtrPhys = this->entityModelMap.Get(_entity))
        {
          // Remove child links, collisions and joints first
          for (const auto &childLink :
               _ecm.ChildrenByComponents(_entity, components::Link()))
          {
            for (const auto &childCollision :
                 _ecm.ChildrenByComponents(childLink, components::Collision()))
            {
              this->entityCollisionMap.Remove(childCollision);
              this->topLevelModelMap.erase(childCollision);
              if (this->customContactSurfaceEntities[world].erase(
                childCollision))
              {
                // if this was the last collision with contact customization,
                // disable the whole feature in the physics engine
                if (this->customContactSurfaceEntities[world].empty())
                {
                  this->DisableContactSurfaceCustomization(world);
                }
              }
            }
            this->entityLinkMap.Remove(childLink);
            this->entityFreeGroupMap.Remove(childLink);
            this->topLevelModelMap.erase(childLink);
            this->staticEntities.erase(childLink);
            this->linkWorldPoses.erase(childLink);
            this->canonicalLinkModelTracker.RemoveLink(childLink);
          }

          for (const auto &childJoint :
               _ecm.ChildrenByComponents(_entity, components::Joint()))
          {
            this->entityJointMap.Remove(childJoint);
            this->topLevelModelMap.erase(childJoint);
          }

          this->entityFreeGroupMap.Remove(_entity);
          // Remove the model from the physics engine
          modelPtrPhys->Remove();
          this->entityModelMap.Remove(_entity);
          this->topLevelModelMap.erase(_entity);
          this->staticEntities.erase(_entity);
          this->modelWorldPoses.erase(_entity);
        }
        return true;
      });

  _ecm.EachRemoved<components::DetachableJoint>(
      [&](const Entity &_entity, const components::DetachableJoint *) -> bool
      {
        if (!this->entityJointMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find joint [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto castEntity =
            this->entityJointMap.EntityCast<DetachableJointFeatureList>(
                _entity);
        if (!castEntity)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to detach a joint, but the physics "
                   << "engine doesn't support feature "
                   << "[DetachJointFeature]. Joint won't be detached."
                   << std::endl;
            informed = true;
          }

          // Break Each call since no DetachableJoints can be processed
          return false;
        }

        gzdbg << "Detaching joint [" << _entity << "]" << std::endl;
        castEntity->Detach();
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdatePhysics(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::UpdatePhysics");
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
        auto jointPhys = this->entityJointMap.Get(_entity);
        if (nullptr == jointPhys)
          return true;

        auto jointVelFeature =
          this->entityJointMap.EntityCast<JointVelocityCommandFeatureList>(
              _entity);

        auto jointPosLimitsFeature =
          this->entityJointMap.EntityCast<JointPositionLimitsCommandFeatureList>
              (_entity);

        auto jointVelLimitsFeature =
          this->entityJointMap.EntityCast<JointVelocityLimitsCommandFeatureList>
              (_entity);

        auto jointEffLimitsFeature =
          this->entityJointMap.EntityCast<JointEffortLimitsCommandFeatureList>(
              _entity);

        auto haltMotionComp = _ecm.Component<components::HaltMotion>(
            _ecm.ParentEntity(_entity));
        bool haltMotion = false;
        if (haltMotionComp)
        {
          haltMotion = haltMotionComp->Data();
        }

        // Model is out of battery or halt motion has been triggered.
        if (this->entityOffMap[_ecm.ParentEntity(_entity)] || haltMotion)
        {
          std::size_t nDofs = jointPhys->GetDegreesOfFreedom();
          for (std::size_t i = 0; i < nDofs; ++i)
          {
            jointPhys->SetForce(i, 0);

            // Halt motion requires the vehicle to come to a full stop,
            // while running out of battery can leave existing joint velocity
            // in place.
            if (haltMotion && jointVelFeature)
              jointVelFeature->SetVelocityCommand(i, 0);
          }
          return true;
        }

        auto posLimits = _ecm.Component<components::JointPositionLimitsCmd>(
            _entity);
        if (posLimits && !posLimits->Data().empty())
        {
          const auto& limits = posLimits->Data();

          if (limits.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom "
            << "between Joint [" << _name->Data() << "(Entity="
            << _entity << ")] and its JointPositionLimitsCmd "
            << "component. The joint has "
            << jointPhys->GetDegreesOfFreedom()
            << " while the component has "
            << limits.size() << ".\n";
          }

          if (jointPosLimitsFeature)
          {
            std::size_t nDofs = std::min(
              limits.size(),
              jointPhys->GetDegreesOfFreedom());

            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointPosLimitsFeature->SetMinPosition(i, limits[i].X());
              jointPosLimitsFeature->SetMaxPosition(i, limits[i].Y());
            }
          }
        }

        auto velLimits = _ecm.Component<components::JointVelocityLimitsCmd>(
            _entity);
        if (velLimits && !velLimits->Data().empty())
        {
          const auto& limits = velLimits->Data();

          if (limits.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom "
            << "between Joint [" << _name->Data() << "(Entity="
            << _entity << ")] and its JointVelocityLimitsCmd "
            << "component. The joint has "
            << jointPhys->GetDegreesOfFreedom()
            << " while the component has "
            << limits.size() << ".\n";
          }

          if (jointVelLimitsFeature)
          {
            std::size_t nDofs = std::min(
              limits.size(),
              jointPhys->GetDegreesOfFreedom());

            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointVelLimitsFeature->SetMinVelocity(i, limits[i].X());
              jointVelLimitsFeature->SetMaxVelocity(i, limits[i].Y());
            }
          }
        }

        auto effLimits = _ecm.Component<components::JointEffortLimitsCmd>(
            _entity);
        if (effLimits && !effLimits->Data().empty())
        {
          const auto& limits = effLimits->Data();

          if (limits.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom "
            << "between Joint [" << _name->Data() << "(Entity="
            << _entity << ")] and its JointEffortLimitsCmd "
            << "component. The joint has "
            << jointPhys->GetDegreesOfFreedom()
            << " while the component has "
            << limits.size() << ".\n";
          }

          if (jointEffLimitsFeature)
          {
            std::size_t nDofs = std::min(
              limits.size(),
              jointPhys->GetDegreesOfFreedom());

            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointEffLimitsFeature->SetMinEffort(i, limits[i].X());
              jointEffLimitsFeature->SetMaxEffort(i, limits[i].Y());
            }
          }
        }

        auto posReset = _ecm.Component<components::JointPositionReset>(
            _entity);
        auto velReset = _ecm.Component<components::JointVelocityReset>(
            _entity);

        // Reset the velocity
        if (velReset)
        {
          auto& jointVelocity = velReset->Data();

          if (jointVelocity.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom "
                    << "between Joint [" << _name->Data() << "(Entity="
                    << _entity << ")] and its JointVelocityReset "
                    << "component. The joint has "
                    << jointPhys->GetDegreesOfFreedom()
                    << " while the component has "
                    << jointVelocity.size() << ".\n";
            }

            std::size_t nDofs = std::min(
                jointVelocity.size(), jointPhys->GetDegreesOfFreedom());

            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointPhys->SetVelocity(i, jointVelocity[i]);
            }
        }

        // Reset the position
        if (posReset)
        {
          auto &jointPosition = posReset->Data();

          if (jointPosition.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom "
                    << "between Joint [" << _name->Data() << "(Entity="
                    << _entity << ")] and its JointPositionyReset "
                    << "component. The joint has "
                    << jointPhys->GetDegreesOfFreedom()
                    << " while the component has "
                    << jointPosition.size() << ".\n";
            }
            std::size_t nDofs = std::min(
                jointPosition.size(), jointPhys->GetDegreesOfFreedom());
            for (std::size_t i = 0; i < nDofs; ++i)
            {
              jointPhys->SetPosition(i, jointPosition[i]);
            }
        }

        auto force = _ecm.Component<components::JointForceCmd>(_entity);
        auto velCmd = _ecm.Component<components::JointVelocityCmd>(_entity);

        if (force)
        {
          if (force->Data().size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom between "
                    << "Joint [" << _name->Data() << "(Entity=" << _entity
                    << ")] and its JointForceCmd component. The joint has "
                    << jointPhys->GetDegreesOfFreedom() << " while the "
                    << " component has " << force->Data().size() << ".\n";
          }
          std::size_t nDofs = std::min(force->Data().size(),
                                       jointPhys->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < nDofs; ++i)
          {
            jointPhys->SetForce(i, force->Data()[i]);
          }
        }
        // Only set joint velocity if joint force is not set.
        // If both the cmd and reset components are found, cmd is ignored.
        else if (velCmd)
        {
          auto velocityCmd = velCmd->Data();

          if (velReset)
          {
            gzwarn << "Found both JointVelocityReset and "
                    << "JointVelocityCmd components for Joint ["
                    << _name->Data() << "(Entity=" << _entity
                    << "]). Ignoring JointVelocityCmd component."
                    << std::endl;
            return true;
          }

          if (velocityCmd.size() != jointPhys->GetDegreesOfFreedom())
          {
            gzwarn << "There is a mismatch in the degrees of freedom"
                    << " between Joint [" << _name->Data()
                    << "(Entity=" << _entity<< ")] and its "
                    << "JointVelocityCmd component. The joint has "
                    << jointPhys->GetDegreesOfFreedom()
                    << " while the component has "
                    << velocityCmd.size() << ".\n";
          }

          if (!jointVelFeature)
          {
            return true;
          }

          std::size_t nDofs = std::min(
            velocityCmd.size(),
            jointPhys->GetDegreesOfFreedom());

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
        if (!this->entityLinkMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find link [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto linkForceFeature =
            this->entityLinkMap.EntityCast<LinkForceFeatureList>(_entity);
        if (!linkForceFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to apply a wrench, but the physics "
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
  auto olderWorldPoseCmdsToRemove = std::move(this->worldPoseCmdsToRemove);
  this->worldPoseCmdsToRemove.clear();

  _ecm.Each<components::Model, components::WorldPoseCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::WorldPoseCmd *_poseCmd)
      {
        this->worldPoseCmdsToRemove.insert(_entity);

        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        // world pose cmd currently not supported for nested models
        if (_entity != this->topLevelModelMap[_entity])
        {
          gzerr << "Unable to set world pose for nested models."
                 << std::endl;
          return true;
        }

        // TODO(addisu) Store the free group instead of searching for it at
        // every iteration
        auto freeGroup = modelPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;

        // Get root link offset
        const auto linkEntity =
            this->entityLinkMap.Get(freeGroup->RootLink());
        if (linkEntity == kNullEntity)
          return true;

        // set world pose of root link in freegroup
        // root link might be in a nested model so use RelativePose to get
        // its pose relative to this model
        math::Pose3d linkPose =
            this->RelativePose(_entity, linkEntity, _ecm);

        freeGroup->SetWorldPose(math::eigen3::convert(_poseCmd->Data() *
                                linkPose));

        // Process pose commands for static models here, as one-time changes
        if (this->staticEntities.find(_entity) != this->staticEntities.end())
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

  // Remove world commands from previous iteration. We let them rotate one
  // iteration so other systems have a chance to react to them too.
  for (const Entity &entity : olderWorldPoseCmdsToRemove)
  {
    _ecm.RemoveComponent<components::WorldPoseCmd>(entity);
  }

  // Slip compliance on Collisions
  _ecm.Each<components::SlipComplianceCmd>(
      [&](const Entity &_entity,
          const components::SlipComplianceCmd *_slipCmdComp)
      {
        if (!this->entityCollisionMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find shape [" << _entity << "]." << std::endl;
          return true;
        }

        auto slipComplianceShape =
            this->entityCollisionMap
                .EntityCast<FrictionPyramidSlipComplianceFeatureList>(_entity);

        if (!slipComplianceShape)
        {
          gzwarn << "Can't process Wheel Slip component, physics engine "
                  << "missing SetShapeFrictionPyramidSlipCompliance"
                  << std::endl;

          // Break Each call since no SlipCompliances can be processed
          return false;
        }

        if (_slipCmdComp->Data().size() == 2)
        {
          slipComplianceShape->SetPrimarySlipCompliance(
              _slipCmdComp->Data()[0]);
          slipComplianceShape->SetSecondarySlipCompliance(
              _slipCmdComp->Data()[1]);
        }

        return true;
      });

  // Update model angular velocity
  _ecm.Each<components::Model, components::AngularVelocityCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::AngularVelocityCmd *_angularVelocityCmd)
      {
        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        // angular vel cmd currently not supported for nested models
        if (_entity != this->topLevelModelMap[_entity])
        {
          gzerr << "Unable to set angular velocity for nested models."
                 << std::endl;
          return true;
        }

        auto freeGroup = modelPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;
        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        const components::Pose *poseComp =
            _ecm.Component<components::Pose>(_entity);
        math::Vector3d worldAngularVel = poseComp->Data().Rot() *
            _angularVelocityCmd->Data();

        auto worldAngularVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);
        if (!worldAngularVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set model angular velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be set."
                   << std::endl;
            informed = true;
          }
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
        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        // linear vel cmd currently not supported for nested models
        if (_entity != this->topLevelModelMap[_entity])
        {
          gzerr << "Unable to set linear velocity for nested models."
                 << std::endl;
          return true;
        }

        auto freeGroup = modelPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;

        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        const components::Pose *poseComp =
            _ecm.Component<components::Pose>(_entity);
        math::Vector3d worldLinearVel = poseComp->Data().Rot() *
            _linearVelocityCmd->Data();

        auto worldLinearVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);
        if (!worldLinearVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set model linear velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be set."
                   << std::endl;
            informed = true;
          }
          return true;
        }

        worldLinearVelFeature->SetWorldLinearVelocity(
            math::eigen3::convert(worldLinearVel));

        return true;
      });

  // Update link angular velocity
  _ecm.Each<components::Link, components::AngularVelocityCmd>(
      [&](const Entity &_entity, const components::Link *,
          const components::AngularVelocityCmd *_angularVelocityCmd)
      {
        if (!this->entityLinkMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find link [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto linkPtrPhys = this->entityLinkMap.Get(_entity);
        if (nullptr == linkPtrPhys)
          return true;

        auto freeGroup = linkPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;
        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        auto worldAngularVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);

        if (!worldAngularVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set link angular velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be set."
                   << std::endl;
            informed = true;
          }
          return true;
        }
        // velocity in world frame = world_to_model_tf * model_to_link_tf * vel
        Entity modelEntity = topLevelModel(_entity, _ecm);
        const components::Pose *modelEntityPoseComp =
            _ecm.Component<components::Pose>(modelEntity);
        math::Pose3d modelToLinkTransform = this->RelativePose(
            modelEntity, _entity, _ecm);
        math::Vector3d worldAngularVel = modelEntityPoseComp->Data().Rot()
            * modelToLinkTransform.Rot() * _angularVelocityCmd->Data();
        worldAngularVelFeature->SetWorldAngularVelocity(
            math::eigen3::convert(worldAngularVel));

        return true;
      });

  // Update link linear velocity
  _ecm.Each<components::Link, components::LinearVelocityCmd>(
      [&](const Entity &_entity, const components::Link *,
          const components::LinearVelocityCmd *_linearVelocityCmd)
      {
        if (!this->entityLinkMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find link [" << _entity
                  << "]." << std::endl;
          return true;
        }

        auto linkPtrPhys = this->entityLinkMap.Get(_entity);
        if (nullptr == linkPtrPhys)
          return true;

        auto freeGroup = linkPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;
        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        auto worldLinearVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);
        if (!worldLinearVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set link linear velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be set."
                   << std::endl;
            informed = true;
          }
          return true;
        }

        // velocity in world frame = world_to_model_tf * model_to_link_tf * vel
        Entity modelEntity = topLevelModel(_entity, _ecm);
        const components::Pose *modelEntityPoseComp =
            _ecm.Component<components::Pose>(modelEntity);
        math::Pose3d modelToLinkTransform = this->RelativePose(
            modelEntity, _entity, _ecm);
        math::Vector3d worldLinearVel = modelEntityPoseComp->Data().Rot()
            * modelToLinkTransform.Rot() * _linearVelocityCmd->Data();
        worldLinearVelFeature->SetWorldLinearVelocity(
            math::eigen3::convert(worldLinearVel));

        return true;
      });


  // Populate bounding box info
  // Only compute bounding box if component exists to avoid unnecessary
  // computations
  _ecm.Each<components::Model, components::AxisAlignedBox>(
      [&](const Entity &_entity, const components::Model *,
          components::AxisAlignedBox *_bbox)
      {
        if (!this->entityModelMap.HasEntity(_entity))
        {
          gzwarn << "Failed to find model [" << _entity << "]." << std::endl;
          return true;
        }

        auto bbModel =
            this->entityModelMap.EntityCast<BoundingBoxFeatureList>(_entity);

        if (!bbModel)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to get a bounding box, but the physics "
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
            ComponentState::PeriodicChange :
            ComponentState::NoChange;
        _ecm.SetChanged(_entity, components::AxisAlignedBox::typeId, state);

        return true;
      });
}  // NOLINT readability/fn_size
// TODO (azeey) Reduce size of function and remove the NOLINT above

//////////////////////////////////////////////////
void PhysicsPrivate::ResetPhysics(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::ResetPhysics");
  // Clear worldPoseCmdsToRemove because pose commands that were issued before
  // the reset will be ignored.
  this->linkWorldPoses.clear();
  this->canonicalLinkModelTracker = CanonicalLinkModelTracker();
  this->modelWorldPoses.clear();
  this->worldPoseCmdsToRemove.clear();

  this->RemovePhysicsEntities(_ecm);
  this->CreatePhysicsEntities(_ecm, false);
  this->canonicalLinkModelTracker.AddAllModels(_ecm);

  // Update link pose, linear velocity, and angular velocity
  _ecm.Each<components::Link>(
      [&](const Entity &_entity, const components::Link *)
      {
        auto linkPtrPhys = this->entityLinkMap.Get(_entity);
        if (nullptr == linkPtrPhys)
        {
          gzwarn << "Failed to find link [" << _entity << "]." << std::endl;
          return true;
        }

        auto freeGroup = linkPtrPhys->FindFreeGroup();
        if (!freeGroup)
          return true;

        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        if (freeGroup->RootLink() == linkPtrPhys)
        {
          auto linkWorldPose = worldPose(_entity, _ecm);
          freeGroup->SetWorldPose(math::eigen3::convert(linkWorldPose));
        }

        auto worldAngularVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);

        if (!worldAngularVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to reset link angular velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be reset."
                   << std::endl;
            informed = true;
          }
          return true;
        }
        else
        {
          worldAngularVelFeature->SetWorldAngularVelocity(
              Eigen::Vector3d::Zero());
        }

        auto worldLinearVelFeature =
            this->entityFreeGroupMap
                .EntityCast<WorldVelocityCommandFeatureList>(_entity);
        if (!worldLinearVelFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set link linear velocity, but the "
                   << "physics engine doesn't support velocity commands. "
                   << "Velocity won't be set."
                   << std::endl;
            informed = true;
          }
          return true;
        }
        else
        {
          worldLinearVelFeature->SetWorldLinearVelocity(
              Eigen::Vector3d::Zero());
        }

        return true;
      });

  // Handle joint state
  _ecm.Each<components::Joint>(
      [&](const Entity &_entity, const components::Joint *)
      {
        auto jointPhys = this->entityJointMap.Get(_entity);
        if (nullptr == jointPhys)
        {
          gzwarn << "Failed to find joint [" << _entity << "]." << std::endl;
          return true;
        }

        // Assume initial joint position and velocities are zero
        // Reset the velocity
        for (std::size_t i = 0; i < jointPhys->GetDegreesOfFreedom(); ++i)
        {
          jointPhys->SetVelocity(i, 0.0);
          jointPhys->SetPosition(i, 0.0);
        }

        return true;
      });

  // Also update modelWorldPoses. This is a workaround to the problem that we
  // don't have a way to reset the physics engine and clear its internal cache
  // of link poses. In the event that a model's canonical link's pose hasn't
  // changed after reset, the parent model's world pose won't be recorded in
  // the modelWorldPoses map. If any of the model's other links have changed,
  // however, we try to look for the parent model's world pose in
  // modelWorldPoses and fail. So the workaround here is to update the world
  // poses of all models.
  _ecm.Each<components::Model>(
      [&](const Entity &_entity, const components::Model *)
      {
        this->modelWorldPoses[_entity] = sim::worldPose(_entity, _ecm);
        return true;
      });
}

//////////////////////////////////////////////////
gz::physics::ForwardStep::Output PhysicsPrivate::Step(
    const std::chrono::steady_clock::duration &_dt)
{
  GZ_PROFILE("PhysicsPrivate::Step");
  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;

  input.Get<std::chrono::steady_clock::duration>() = _dt;

  for (const auto &world : this->entityWorldMap.Map())
  {
    world.second->Step(output, state, input);
  }

  return output;
}

//////////////////////////////////////////////////
math::Pose3d PhysicsPrivate::RelativePose(const Entity &_from,
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
std::map<Entity, physics::FrameData3d> PhysicsPrivate::ChangedLinks(
    EntityComponentManager &_ecm,
    const gz::physics::ForwardStep::Output &_updatedLinks)
{
  GZ_PROFILE("Links Frame Data");

  std::map<Entity, physics::FrameData3d> linkFrameData;

  // Check to see if the physics engine gave a list of changed poses. If not, we
  // will iterate through all of the links via the ECM to see which ones changed
  if (_updatedLinks.Has<gz::physics::ChangedWorldPoses>())
  {
    for (const auto &link :
        _updatedLinks.Query<gz::physics::ChangedWorldPoses>()->entries)
    {
      // get the gazebo entity that matches the updated physics link entity
      const auto linkPhys = this->entityLinkMap.GetPhysicsEntityPtr(link.body);
      if (nullptr == linkPhys)
      {
        gzerr << "Internal error: a physics entity ptr with an ID of ["
          << link.body << "] does not exist." << std::endl;
        continue;
      }
      auto entity = this->entityLinkMap.Get(linkPhys);
      if (entity == kNullEntity)
      {
        gzerr << "Internal error: no gazebo entity matches the physics entity "
          << "with ID [" << link.body << "]." << std::endl;
        continue;
      }

      auto frameData = linkPhys->FrameDataRelativeToWorld();
      linkFrameData[entity] = frameData;
    }
  }
  else
  {
    _ecm.Each<components::Link>(
      [&](const Entity &_entity, components::Link *) -> bool
      {
        if (this->staticEntities.find(_entity) != this->staticEntities.end() ||
            _ecm.EntityHasComponentType(_entity, components::Recreate::typeId))
        {
          return true;
        }

        // This `once` variable is here to aid in debugging, make sure to
        // remove it.
        auto linkPhys = this->entityLinkMap.Get(_entity);
        if (nullptr == linkPhys)
        {
          if (this->linkAddedToModel.find(_entity) ==
              this->linkAddedToModel.end())
          {
            // ignore links from actors for now
            auto parentId =
                _ecm.Component<components::ParentEntity>(_entity)->Data();
            if (!_ecm.Component<components::Actor>(parentId))
            {
              gzerr << "Internal error: link [" << _entity
                    << "] not in entity map" << std::endl;
            }
          }
          return true;
        }

        auto frameData = linkPhys->FrameDataRelativeToWorld();

        // update the link pose if this is the first update,
        // or if the link pose has changed since the last update
        // (if the link pose hasn't changed, there's no need for a pose update)
        const auto worldPoseMath3d = gz::math::eigen3::convert(
            frameData.pose);
        if ((this->linkWorldPoses.find(_entity) == this->linkWorldPoses.end())
            || !this->pose3Eql(this->linkWorldPoses[_entity], worldPoseMath3d))
        {
          // cache the updated link pose to check if the link pose has changed
          // during the next iteration
          this->linkWorldPoses[_entity] = worldPoseMath3d;

          linkFrameData[_entity] = frameData;
        }

        return true;
      });
  }

  return linkFrameData;
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateModelPose(const Entity _model,
    const Entity _canonicalLink, EntityComponentManager &_ecm,
    std::map<Entity, physics::FrameData3d> &_linkFrameData)
{
  std::optional<math::Pose3d> parentWorldPose;

  // If this model is nested, the pose of the parent model has already
  // been updated since we iterate through the modified links in
  // topological order. We expect to find the updated pose in
  // this->modelWorldPoses. If not found, this must not be nested, so this
  // model's pose component would reflect it's absolute pose.
  auto parentModelPoseIt =
    this->modelWorldPoses.find(
        _ecm.Component<components::ParentEntity>(_model)->Data());
  if (parentModelPoseIt != this->modelWorldPoses.end())
  {
    parentWorldPose = parentModelPoseIt->second;
  }

  // Given the following frame names:
  // W: World/inertial frame
  // P: Parent frame (this could be a parent model or the World frame)
  // M: This model's frame
  // L: The frame of this model's canonical link
  //
  // And the following quantities:
  // (See http://sdformat.org/tutorials?tut=specify_pose for pose
  // convention)
  // parentWorldPose (X_WP): Pose of the parent frame w.r.t the world
  // linkPoseFromModel (X_ML): Pose of the canonical link frame w.r.t the
  // model frame
  // linkWorldPose (X_WL): Pose of the canonical link w.r.t the world
  // modelWorldPose (X_WM): Pose of this model w.r.t the world
  //
  // The Pose component of this model entity stores the pose of M w.r.t P
  // (X_PM) and is calculated as
  //   X_PM = (X_WP)^-1 * X_WM
  //
  // And X_WM is calculated from X_WL, which is obtained from physics as:
  //   X_WM = X_WL * (X_ML)^-1
  auto linkPoseFromModel = this->RelativePose(_model, _canonicalLink, _ecm);
  const auto &linkWorldPose = _linkFrameData[_canonicalLink].pose;
  const auto &modelWorldPose =
      math::eigen3::convert(linkWorldPose) * linkPoseFromModel.Inverse();

  this->modelWorldPoses[_model] = modelWorldPose;

  // update model's pose
  auto modelPose = _ecm.Component<components::Pose>(_model);
  if (parentWorldPose)
  {
    *modelPose =
        components::Pose(parentWorldPose->Inverse() * modelWorldPose);
  }
  else
  {
    // This is a non-nested model and parentWorldPose would be identity
    // because it would be the pose of the parent (world) w.r.t the world.
    *modelPose = components::Pose(modelWorldPose);
  }

  _ecm.SetChanged(_model, components::Pose::typeId,
                  ComponentState::PeriodicChange);

  // once the model pose has been updated, all descendant link poses of this
  // model must be updated (whether the link actually changed pose or not)
  // since link poses are saved w.r.t. their parent model
  auto model = sim::Model(_model);
  for (const auto &childLink : model.Links(_ecm))
  {
    // skip links that are already marked as a link to be updated
    if (_linkFrameData.find(childLink) != _linkFrameData.end())
      continue;

    physics::FrameData3d childLinkFrameData;
    if (!this->GetFrameDataRelativeToWorld(childLink, childLinkFrameData))
      continue;

    _linkFrameData[childLink] = childLinkFrameData;
  }

  // since nested model poses are saved w.r.t. the nested model's parent
  // pose, we must also update any nested models that have a different
  // canonical link
  for (const auto &nestedModel : model.Models(_ecm))
  {
    auto nestedModelCanonicalLinkComp =
      _ecm.Component<components::ModelCanonicalLink>(nestedModel);
    if (!nestedModelCanonicalLinkComp)
    {
      auto staticComp = _ecm.Component<components::Static>(nestedModel);
      if (!staticComp || !staticComp->Data())
        gzerr << "Model [" << nestedModel << "] has no canonical link\n";
      continue;
    }

    auto nestedCanonicalLink = nestedModelCanonicalLinkComp->Data();

    // skip links that are already marked as a link to be updated
    if (nestedCanonicalLink == _canonicalLink ||
        _linkFrameData.find(nestedCanonicalLink) != _linkFrameData.end())
      continue;

    // mark this canonical link as one that needs to be updated so that all of
    // the models that have this canonical link are updated
    physics::FrameData3d canonicalLinkFrameData;
    if (!this->GetFrameDataRelativeToWorld(nestedCanonicalLink,
          canonicalLinkFrameData))
      continue;

    _linkFrameData[nestedCanonicalLink] = canonicalLinkFrameData;
  }
}

//////////////////////////////////////////////////
bool PhysicsPrivate::GetFrameDataRelativeToWorld(const Entity _entity,
    physics::FrameData3d &_data)
{
  auto entityPhys = this->entityLinkMap.Get(_entity);
  if (nullptr == entityPhys)
  {
    // Suppress error message if the link has just been added to the model.
    if (this->linkAddedToModel.find(_entity) == this->linkAddedToModel.end())
    {
      gzerr << "Internal error: entity [" << _entity
        << "] not in entity map" << std::endl;
    }
    return false;
  }

  _data = entityPhys->FrameDataRelativeToWorld();
  return true;
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateSim(EntityComponentManager &_ecm,
    std::map<Entity, physics::FrameData3d> &_linkFrameData)
{
  GZ_PROFILE("PhysicsPrivate::UpdateSim");

  // Populate world components with default values
  _ecm.EachNew<components::World>(
      [&](const Entity &_entity,
        const components::World *)->bool
      {
        // If not provided by ECM, create component with values from physics if
        // those features are available
        auto collisionDetectorComp =
            _ecm.Component<components::PhysicsCollisionDetector>(_entity);
        if (!collisionDetectorComp)
        {
          auto collisionDetectorFeature =
              this->entityWorldMap.EntityCast<CollisionDetectorFeatureList>(
              _entity);
          if (collisionDetectorFeature)
          {
            _ecm.CreateComponent(_entity, components::PhysicsCollisionDetector(
                collisionDetectorFeature->GetCollisionDetector()));
          }
        }

        auto solverComp = _ecm.Component<components::PhysicsSolver>(_entity);
        if (!solverComp)
        {
          auto solverFeature =
              this->entityWorldMap.EntityCast<SolverFeatureList>(_entity);
          if (solverFeature)
          {
            _ecm.CreateComponent(_entity,
                components::PhysicsSolver(solverFeature->GetSolver()));
          }
        }

        return true;
      });

  GZ_PROFILE_BEGIN("Models");

  // make sure we have an up-to-date mapping of canonical links to their models
  this->canonicalLinkModelTracker.AddNewModels(_ecm);

  for (const auto &[linkEntity, frameData] : _linkFrameData)
  {
    // get a topological ordering of the models that have linkEntity as the
    // model's canonical link. If linkEntity isn't a canonical link for any
    // models, canonicalLinkModels will be empty
    auto canonicalLinkModels =
      this->canonicalLinkModelTracker.CanonicalLinkModels(linkEntity);

    // Update poses for all of the models that have this changed canonical link
    // (linkEntity). Since we have the models in topological order and
    // _linkFrameData stores links in topological order thanks to the ordering
    // of std::map (entity IDs are created in ascending order), this should
    // properly handle pose updates for nested models that share the same
    // canonical link.
    //
    // Nested models that don't share the same canonical link will also need to
    // be updated since these nested models have their pose saved w.r.t. their
    // parent model, which just experienced a pose update. The UpdateModelPose
    // method also handles this case.
    for (auto &modelEnt : canonicalLinkModels)
      this->UpdateModelPose(modelEnt, linkEntity, _ecm, _linkFrameData);
  }
  GZ_PROFILE_END();

  // Link poses, velocities...
  GZ_PROFILE_BEGIN("Links");
  for (const auto &[entity, frameData] : _linkFrameData)
  {
    GZ_PROFILE_BEGIN("Local pose");
    auto canonicalLink =
        _ecm.Component<components::CanonicalLink>(entity);

    const auto &worldPose = frameData.pose;
    const auto parentEntity = _ecm.ParentEntity(entity);

    if (!canonicalLink)
    {
      // Compute the relative pose of this link from the parent model
      auto parentModelPoseIt = this->modelWorldPoses.find(parentEntity);
      if (parentModelPoseIt == this->modelWorldPoses.end())
      {
        gzerr << "Internal error: parent model [" << parentEntity
              << "] does not have a world pose available for child entity["
              << entity << "]" << std::endl;
        continue;
      }
      const math::Pose3d &parentWorldPose = parentModelPoseIt->second;

      // Unlike canonical links, pose of regular links can move relative.
      // to the parent. Same for links inside nested models.
      auto pose = _ecm.Component<components::Pose>(entity);
      *pose = components::Pose(parentWorldPose.Inverse() *
                                math::eigen3::convert(worldPose));
      _ecm.SetChanged(entity, components::Pose::typeId,
          ComponentState::PeriodicChange);
    }
    GZ_PROFILE_END();

    // Populate world poses, velocities and accelerations of the link. For
    // now these components are updated only if another system has created
    // the corresponding component on the entity.
    auto worldPoseComp = _ecm.Component<components::WorldPose>(entity);
    if (worldPoseComp)
    {
      auto state =
          worldPoseComp->SetData(math::eigen3::convert(frameData.pose),
          this->pose3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity, components::WorldPose::typeId, state);
    }

    // Velocity in world coordinates
    auto worldLinVelComp =
        _ecm.Component<components::WorldLinearVelocity>(entity);
    if (worldLinVelComp)
    {
      auto state = worldLinVelComp->SetData(
            math::eigen3::convert(frameData.linearVelocity),
            this->vec3Eql) ?
            ComponentState::PeriodicChange :
            ComponentState::NoChange;
      _ecm.SetChanged(entity,
          components::WorldLinearVelocity::typeId, state);
    }

    // Angular velocity in world frame coordinates
    auto worldAngVelComp =
        _ecm.Component<components::WorldAngularVelocity>(entity);
    if (worldAngVelComp)
    {
      auto state = worldAngVelComp->SetData(
          math::eigen3::convert(frameData.angularVelocity),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity,
          components::WorldAngularVelocity::typeId, state);
    }

    // Acceleration in world frame coordinates
    auto worldLinAccelComp =
        _ecm.Component<components::WorldLinearAcceleration>(entity);
    if (worldLinAccelComp)
    {
      auto state = worldLinAccelComp->SetData(
          math::eigen3::convert(frameData.linearAcceleration),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity,
          components::WorldLinearAcceleration::typeId, state);
    }

    // Angular acceleration in world frame coordinates
    auto worldAngAccelComp =
        _ecm.Component<components::WorldAngularAcceleration>(entity);

    if (worldAngAccelComp)
    {
      auto state = worldAngAccelComp->SetData(
          math::eigen3::convert(frameData.angularAcceleration),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity,
          components::WorldAngularAcceleration::typeId, state);
    }

    const Eigen::Matrix3d R_bs = worldPose.linear().transpose(); // NOLINT

    // Velocity in body-fixed frame coordinates
    auto bodyLinVelComp =
        _ecm.Component<components::LinearVelocity>(entity);
    if (bodyLinVelComp)
    {
      Eigen::Vector3d bodyLinVel = R_bs * frameData.linearVelocity;
      auto state =
          bodyLinVelComp->SetData(math::eigen3::convert(bodyLinVel),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity, components::LinearVelocity::typeId, state);
    }

    // Angular velocity in body-fixed frame coordinates
    auto bodyAngVelComp =
        _ecm.Component<components::AngularVelocity>(entity);
    if (bodyAngVelComp)
    {
      Eigen::Vector3d bodyAngVel = R_bs * frameData.angularVelocity;
      auto state =
          bodyAngVelComp->SetData(math::eigen3::convert(bodyAngVel),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity, components::AngularVelocity::typeId,
          state);
    }

    // Acceleration in body-fixed frame coordinates
    auto bodyLinAccelComp =
        _ecm.Component<components::LinearAcceleration>(entity);
    if (bodyLinAccelComp)
    {
      Eigen::Vector3d bodyLinAccel = R_bs * frameData.linearAcceleration;
      auto state =
          bodyLinAccelComp->SetData(math::eigen3::convert(bodyLinAccel),
          this->vec3Eql)?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity, components::LinearAcceleration::typeId,
          state);
    }

    // Angular acceleration in world frame coordinates
    auto bodyAngAccelComp =
        _ecm.Component<components::AngularAcceleration>(entity);
    if (bodyAngAccelComp)
    {
      Eigen::Vector3d bodyAngAccel = R_bs * frameData.angularAcceleration;
      auto state =
          bodyAngAccelComp->SetData(math::eigen3::convert(bodyAngAccel),
          this->vec3Eql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(entity, components::AngularAcceleration::typeId,
          state);
    }
  }
  GZ_PROFILE_END();

  // pose/velocity/acceleration of non-link entities such as sensors /
  // collisions. These get updated only if another system has created
  // the component for the entity.
  // Populated components:
  // * WorldPose
  // * WorldLinearVelocity
  // * AngularVelocity
  // * LinearAcceleration

  GZ_PROFILE_BEGIN("Sensors / collisions");
  // world pose
  _ecm.Each<components::Pose, components::WorldPose,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose, components::WorldPose *_worldPose,
          const components::ParentEntity *_parent)->bool
      {
        // check if parent entity is a link, e.g. entity is sensor / collision
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

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
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          // set entity world linear velocity
          *_worldLinearVel = components::WorldLinearVelocity(
              math::eigen3::convert(entityFrameData.linearVelocity));
        }

        return true;
      });

  // body linear velocity
  _ecm.Each<components::Pose, components::LinearVelocity,
            components::ParentEntity>(
    [&](const Entity&, const components::Pose *_pose,
        components::LinearVelocity *_linearVel,
        const components::ParentEntity *_parent)->bool
    {
      // check if parent entity is a link, e.g. entity is sensor / collision
      if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
      {
        const auto entityFrameData =
            this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

        auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
        math::Vector3d entityWorldLinearVel =
            math::eigen3::convert(entityFrameData.linearVelocity);

        auto entityBodyLinearVel =
            entityWorldPose.Rot().RotateVectorReverse(entityWorldLinearVel);
        *_linearVel = components::LinearVelocity(entityBodyLinearVel);
      }

      return true;
    });

  // world angular velocity
  _ecm.Each<components::Pose, components::WorldAngularVelocity,
            components::ParentEntity>(
      [&](const Entity&,
          const components::Pose *_pose,
          components::WorldAngularVelocity *_worldAngularVel,
          const components::ParentEntity *_parent)->bool
      {
        // check if parent entity is a link, e.g. entity is sensor / collision
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          // set entity world angular velocity
          *_worldAngularVel = components::WorldAngularVelocity(
              math::eigen3::convert(entityFrameData.angularVelocity));
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
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
          math::Vector3d entityWorldAngularVel =
              math::eigen3::convert(entityFrameData.angularVelocity);

          auto entityBodyAngularVel =
              entityWorldPose.Rot().RotateVectorReverse(entityWorldAngularVel);
          *_angularVel = components::AngularVelocity(entityBodyAngularVel);
        }

        return true;
      });

  // world linear acceleration
  _ecm.Each<components::Pose, components::WorldLinearAcceleration,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::WorldLinearAcceleration *_worldLinearAcc,
          const components::ParentEntity *_parent)->bool
      {
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          // set entity world linear acceleration
          *_worldLinearAcc = components::WorldLinearAcceleration(
              math::eigen3::convert(entityFrameData.linearAcceleration));
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
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
          math::Vector3d entityWorldLinearAcc =
              math::eigen3::convert(entityFrameData.linearAcceleration);

          auto entityBodyLinearAcc =
              entityWorldPose.Rot().RotateVectorReverse(entityWorldLinearAcc);
          *_linearAcc = components::LinearAcceleration(entityBodyLinearAcc);
        }

        return true;
      });

  // world angular acceleration
  _ecm.Each<components::Pose, components::WorldAngularAcceleration,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::WorldAngularAcceleration *_worldAngularAcc,
          const components::ParentEntity *_parent)->bool
      {
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          // set entity world angular acceleration
          *_worldAngularAcc = components::WorldAngularAcceleration(
              math::eigen3::convert(entityFrameData.angularAcceleration));
        }

        return true;
      });

  // body angular acceleration
  _ecm.Each<components::Pose, components::AngularAcceleration,
            components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose,
          components::AngularAcceleration *_angularAcc,
          const components::ParentEntity *_parent)->bool
      {
        if (auto linkPhys = this->entityLinkMap.Get(_parent->Data()))
        {
          const auto entityFrameData =
              this->LinkFrameDataAtOffset(linkPhys, _pose->Data());

          auto entityWorldPose = math::eigen3::convert(entityFrameData.pose);
          math::Vector3d entityWorldAngularAcc =
              math::eigen3::convert(entityFrameData.angularAcceleration);

          auto entityBodyAngularAcc =
              entityWorldPose.Rot().RotateVectorReverse(entityWorldAngularAcc);
          *_angularAcc = components::AngularAcceleration(entityBodyAngularAcc);
        }

        return true;
      });
  GZ_PROFILE_END();

  // Clear reset components
  GZ_PROFILE_BEGIN("Clear / reset components");
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

  std::vector<Entity> entitiesCustomContactSurface;
  _ecm.Each<components::EnableContactSurfaceCustomization>(
      [&](const Entity &_entity,
      components::EnableContactSurfaceCustomization *) -> bool
      {
        entitiesCustomContactSurface.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesCustomContactSurface)
  {
    _ecm.RemoveComponent<components::EnableContactSurfaceCustomization>(entity);
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

  _ecm.Each<components::JointPositionLimitsCmd>(
      [&](const Entity &, components::JointPositionLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  _ecm.Each<components::JointVelocityLimitsCmd>(
      [&](const Entity &, components::JointVelocityLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  _ecm.Each<components::JointEffortLimitsCmd>(
      [&](const Entity &, components::JointEffortLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  _ecm.Each<components::JointVelocityCmd>(
      [&](const Entity &, components::JointVelocityCmd *_vel) -> bool
      {
        std::fill(_vel->Data().begin(), _vel->Data().end(), 0.0);
        return true;
      });

  _ecm.Each<components::SlipComplianceCmd>(
      [&](const Entity &, components::SlipComplianceCmd *_slip) -> bool
      {
        std::fill(_slip->Data().begin(), _slip->Data().end(), 0.0);
        return true;
      });
  GZ_PROFILE_END();

  _ecm.Each<components::AngularVelocityCmd>(
      [&](const Entity &, components::AngularVelocityCmd *_vel) -> bool
      {
        _vel->Data() = math::Vector3d::Zero;
        return true;
      });

  _ecm.Each<components::LinearVelocityCmd>(
      [&](const Entity &, components::LinearVelocityCmd *_vel) -> bool
      {
        _vel->Data() = math::Vector3d::Zero;
        return true;
      });

  // Update joint positions
  GZ_PROFILE_BEGIN("Joints");
  _ecm.Each<components::Joint, components::JointPosition>(
      [&](const Entity &_entity, components::Joint *,
          components::JointPosition *_jointPos) -> bool
      {
        if (auto jointPhys = this->entityJointMap.Get(_entity))
        {
          _jointPos->Data().resize(jointPhys->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < jointPhys->GetDegreesOfFreedom(); ++i)
          {
            _jointPos->Data()[i] = jointPhys->GetPosition(i);
          }
          _ecm.SetChanged(_entity, components::JointPosition::typeId,
              ComponentState::PeriodicChange);
        }
        return true;
      });

  // Update joint Velocities
  _ecm.Each<components::Joint, components::JointVelocity>(
      [&](const Entity &_entity, components::Joint *,
          components::JointVelocity *_jointVel) -> bool
      {
        if (auto jointPhys = this->entityJointMap.Get(_entity))
        {
          _jointVel->Data().resize(jointPhys->GetDegreesOfFreedom());
          for (std::size_t i = 0; i < jointPhys->GetDegreesOfFreedom();
               ++i)
          {
            _jointVel->Data()[i] = jointPhys->GetVelocity(i);
          }
        }
        return true;
      });
  GZ_PROFILE_END();

  // Update joint transmitteds
  _ecm.Each<components::Joint, components::JointTransmittedWrench>(
      [&](const Entity &_entity, components::Joint *,
          components::JointTransmittedWrench *_wrench) -> bool
      {
        auto jointPhys =
            this->entityJointMap
                .EntityCast<JointGetTransmittedWrenchFeatureList>(_entity);
        if (jointPhys)
        {
          const auto &jointWrench = jointPhys->GetTransmittedWrench();

          msgs::Wrench wrenchData;
          msgs::Set(wrenchData.mutable_torque(),
                    math::eigen3::convert(jointWrench.torque));
          msgs::Set(wrenchData.mutable_force(),
                    math::eigen3::convert(jointWrench.force));
          const auto state =
              _wrench->SetData(wrenchData, this->wrenchEql)
                  ? ComponentState::PeriodicChange
                  : ComponentState::NoChange;
          _ecm.SetChanged(_entity, components::JointTransmittedWrench::typeId,
                          state);
        }
        else
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg
                << "Attempting to get joint transmitted wrenches, but the "
                   "physics engine doesn't support this feature. Values in the "
                   "JointTransmittedWrench component will not be meaningful."
                << std::endl;
            informed = true;
          }
        }
        return true;
      });

  // TODO(louise) Skip this if there are no collision features
  this->UpdateCollisions(_ecm);
}  // NOLINT readability/fn_size
// TODO (azeey) Reduce size of function and remove the NOLINT above

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateCollisions(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::UpdateCollisions");
  // Quit early if the ContactData component hasn't been created. This means
  // there are no systems that need contact information
  if (!_ecm.HasComponentType(components::ContactSensorData::typeId))
    return;

  // TODO(addisu) If systems are assumed to only have one world, we should
  // capture the world Entity in a Configure call
  Entity worldEntity = _ecm.EntityByComponents(components::World());

  if (worldEntity == kNullEntity)
  {
    gzerr << "Missing world entity.\n";
    return;
  }

  if (!this->entityWorldMap.HasEntity(worldEntity))
  {
    gzwarn << "Failed to find world [" << worldEntity << "]." << std::endl;
    return;
  }

  auto worldCollisionFeature =
      this->entityWorldMap.EntityCast<ContactFeatureList>(worldEntity);
  if (!worldCollisionFeature)
  {
    static bool informed{false};
    if (!informed)
    {
      gzdbg << "Attempting process contacts, but the physics "
             << "engine doesn't support contact features. "
             << "Contacts won't be computed."
             << std::endl;
      informed = true;
    }
    return;
  }

  // Each contact object we get from gz-physics contains the EntityPtrs of the
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
  auto allContacts =
      std::move(worldCollisionFeature->GetContactsFromLastStep());

  for (const auto &contactComposite : allContacts)
  {
    const auto &contact = contactComposite.Get<WorldShapeType::ContactPoint>();
    auto coll1Entity =
      this->entityCollisionMap.GetByPhysicsId(contact.collision1->EntityID());
    auto coll2Entity =
      this->entityCollisionMap.GetByPhysicsId(contact.collision2->EntityID());

    if (coll1Entity != kNullEntity && coll2Entity != kNullEntity)
    {
      entityContactMap[coll1Entity][coll2Entity].push_back(&contact);
      entityContactMap[coll2Entity][coll1Entity].push_back(&contact);
    }
  }

  // Go through each collision entity that has a ContactData component and
  // set the component value to the list of contacts that correspond to
  // the collision entity
  _ecm.Each<components::Collision, components::ContactSensorData>(
      [&](const Entity &_collEntity1, components::Collision *,
          components::ContactSensorData *_contacts) -> bool
      {
        msgs::Contacts contactsComp;
        if (entityContactMap.find(_collEntity1) == entityContactMap.end())
        {
          // Clear the last contact data
          auto state = _contacts->SetData(contactsComp,
            this->contactsEql) ?
            ComponentState::PeriodicChange :
            ComponentState::NoChange;
          _ecm.SetChanged(
            _collEntity1, components::ContactSensorData::typeId, state);
          return true;
        }

        const auto &contactMap = entityContactMap[_collEntity1];

        for (const auto &[collEntity2, contactData] : contactMap)
        {
          msgs::Contact *contactMsg = contactsComp.add_contact();
          contactMsg->mutable_collision1()->set_id(_collEntity1);
          contactMsg->mutable_collision2()->set_id(collEntity2);
          if (this->contactsEntityNames)
          {
            contactMsg->mutable_collision1()->set_name(
              removeParentScope(scopedName(_collEntity1, _ecm, "::", 0), "::"));
            contactMsg->mutable_collision2()->set_name(
              removeParentScope(scopedName(collEntity2, _ecm, "::", 0), "::"));
          }
          for (const auto &contact : contactData)
          {
            auto *position = contactMsg->add_position();
            position->set_x(contact->point.x());
            position->set_y(contact->point.y());
            position->set_z(contact->point.z());
          }
        }

        auto state = _contacts->SetData(contactsComp,
          this->contactsEql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
        _ecm.SetChanged(
          _collEntity1, components::ContactSensorData::typeId, state);

        return true;
      });
}

//////////////////////////////////////////////////
physics::FrameData3d PhysicsPrivate::LinkFrameDataAtOffset(
      const LinkPtrType &_link, const math::Pose3d &_pose) const
{
  physics::FrameData3d parent;
  parent.pose = math::eigen3::convert(_pose);
  physics::RelativeFrameData3d relFrameData(_link->GetFrameID(), parent);
  return this->engine->Resolve(relFrameData, physics::FrameID::World());
}

//////////////////////////////////////////////////
void PhysicsPrivate::EnableContactSurfaceCustomization(const Entity &_world)
{
  // allow customization of contact joint surface parameters
  auto setContactPropertiesCallbackFeature =
    this->entityWorldMap.EntityCast<
      SetContactPropertiesCallbackFeatureList>(_world);
  if (!setContactPropertiesCallbackFeature)
    return;

  using Policy = physics::FeaturePolicy3d;
  using Feature = physics::SetContactPropertiesCallbackFeature;
  using FeatureList = SetContactPropertiesCallbackFeatureList;
  using GCFeature = physics::GetContactsFromLastStepFeature;
  using GCFeatureWorld = GCFeature::World<Policy, FeatureList>;
  using ContactPoint = GCFeatureWorld::ContactPoint;
  using ExtraContactData = GCFeature::ExtraContactDataT<Policy>;

  const auto callbackID = "gz::sim::systems::Physics";
  setContactPropertiesCallbackFeature->AddContactPropertiesCallback(
    callbackID,
    [this, _world](const GCFeatureWorld::Contact &_contact,
      const size_t _numContactsOnCollision,
      Feature::ContactSurfaceParams<Policy> &_params)
      {
        const auto &contact = _contact.Get<ContactPoint>();
        auto coll1Entity = this->entityCollisionMap.GetByPhysicsId(
          contact.collision1->EntityID());
        auto coll2Entity = this->entityCollisionMap.GetByPhysicsId(
          contact.collision2->EntityID());

        // check if at least one of the entities wants contact surface
        // customization
        if (this->customContactSurfaceEntities[_world].find(coll1Entity) ==
          this->customContactSurfaceEntities[_world].end() &&
          this->customContactSurfaceEntities[_world].find(coll2Entity) ==
          this->customContactSurfaceEntities[_world].end())
        {
          return;
        }

        std::optional<math::Vector3d> force;
        std::optional<math::Vector3d> normal;
        std::optional<double> depth;
        const auto* extraData = _contact.Query<ExtraContactData>();
        if (extraData != nullptr)
        {
          force = math::eigen3::convert(extraData->force);
          normal = math::eigen3::convert(extraData->normal);
          depth = extraData->depth;
        }

        // broadcast the event that we want to collect the customized
        // contact surface properties; each connected client should
        // filter in the callback to treat just the entities it knows
        this->eventManager->
          Emit<events::CollectContactSurfaceProperties>(
            coll1Entity, coll2Entity, math::eigen3::convert(contact.point),
            force, normal, depth, _numContactsOnCollision, _params);
      }
  );

  this->worldContactCallbackIDs[_world] = callbackID;

  gzmsg << "Enabled contact surface customization for world entity [" << _world
         << "]" << std::endl;
}


//////////////////////////////////////////////////
void PhysicsPrivate::DisableContactSurfaceCustomization(const Entity &_world)
{
  if (this->worldContactCallbackIDs.find(_world) ==
      this->worldContactCallbackIDs.end())
  {
    return;
  }

  auto setContactPropertiesCallbackFeature =
    this->entityWorldMap.EntityCast<
      SetContactPropertiesCallbackFeatureList>(_world);
  if (!setContactPropertiesCallbackFeature)
    return;

  setContactPropertiesCallbackFeature->
   RemoveContactPropertiesCallback(this->worldContactCallbackIDs[_world]);

  gzmsg << "Disabled contact surface customization for world entity ["
         << _world << "]" << std::endl;
}

GZ_ADD_PLUGIN(Physics,
                    System,
                    Physics::ISystemConfigure,
                    Physics::ISystemReset,
                    Physics::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(Physics, "gz::sim::systems::Physics")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(Physics, "ignition::gazebo::systems::Physics")
