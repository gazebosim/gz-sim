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

#include "PhysicsPrivate.hh"

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

  std::vector<Entity> entitiesLinearVelocityReset;
  _ecm.Each<components::WorldLinearVelocityReset>(
      [&](const Entity &_entity,
      components::WorldLinearVelocityReset *) -> bool
      {
        entitiesLinearVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesLinearVelocityReset)
  {
    _ecm.RemoveComponent<components::WorldLinearVelocityReset>(entity);
  }

  std::vector<Entity> entitiesAngularVelocityReset;
  _ecm.Each<components::WorldAngularVelocityReset>(
      [&](const Entity &_entity,
      components::WorldAngularVelocityReset *) -> bool
      {
        entitiesAngularVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesAngularVelocityReset)
  {
    _ecm.RemoveComponent<components::WorldAngularVelocityReset>(entity);
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

  {
    std::vector<Entity> entitiesJointVelocityCmd;
    _ecm.Each<components::JointVelocityCmd>(
        [&](const Entity &_entity, components::JointVelocityCmd *) -> bool
        {
          entitiesJointVelocityCmd.push_back(_entity);
          return true;
        });

    for (const auto entity : entitiesJointVelocityCmd)
    {
      _ecm.RemoveComponent<components::JointVelocityCmd>(entity);
    }
  }

  _ecm.Each<components::SlipComplianceCmd>(
      [&](const Entity &, components::SlipComplianceCmd *_slip) -> bool
      {
        std::fill(_slip->Data().begin(), _slip->Data().end(), 0.0);
        return true;
      });

  {
    std::vector<Entity> entitiesAngularVelocityCmd;
    _ecm.Each<components::AngularVelocityCmd>(
        [&](const Entity &_entity, components::AngularVelocityCmd *) -> bool
        {
          entitiesAngularVelocityCmd.push_back(_entity);
          return true;
        });

    for (const auto entity : entitiesAngularVelocityCmd)
    {
      _ecm.RemoveComponent<components::AngularVelocityCmd>(entity);
    }
  }

  {
    std::vector<Entity> entitiesLinearVelocityCmd;
    _ecm.Each<components::LinearVelocityCmd>(
        [&](const Entity &_entity, components::LinearVelocityCmd *) -> bool
        {
          entitiesLinearVelocityCmd.push_back(_entity);
          return true;
        });

    for (const auto entity : entitiesLinearVelocityCmd)
    {
      _ecm.RemoveComponent<components::LinearVelocityCmd>(entity);
    }
  }
  GZ_PROFILE_END();

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

  this->UpdateRayIntersections(_ecm);
}  // NOLINT readability/fn_size
// TODO (azeey) Reduce size of function and remove the NOLINT above

//////////////////////////////////////////////////
