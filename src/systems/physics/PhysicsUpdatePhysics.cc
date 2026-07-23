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

void PhysicsPrivate::UpdatePhysics(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::UpdatePhysics");
  // Gravity state
  _ecm.Each<components::Gravity>(
      [&](const Entity & _entity, const components::Gravity *_gravity)
      {
        auto gravityFeature =
          this->entityWorldMap.EntityCast<GravityFeatureList>(_entity);
        if (!gravityFeature)
        {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to set physics options, but the "
                     << "physics engine doesn't support feature "
                     << "[GravityFeature]. Options will be ignored."
                     << std::endl;
              informed = true;
            }
            return false;
        }
        auto new_grav = _gravity->Data();
        gravityFeature->SetGravity({new_grav.X(), new_grav.Y(), new_grav.Z()});
        return true;
  });

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

  // Update Collision Bitmasks
  auto olderCollideBitmaskCmdsToRemove =
      std::move(this->collideBitmaskCmdsToRemove);
  this->collideBitmaskCmdsToRemove.clear();

  _ecm.Each<components::Collision, components::CollideBitmaskCmd>(
      [&](const Entity &_entity, const components::Collision *,
          const components::CollideBitmaskCmd *_bitmaskCmd) -> bool
      {
        this->collideBitmaskCmdsToRemove.insert(_entity);
        auto filterMaskFeature =
            this->entityCollisionMap.EntityCast<CollisionMaskFeatureList>(
                _entity);
        if (filterMaskFeature)
        {
          filterMaskFeature->SetCollisionFilterMask(_bitmaskCmd->Data());
        }
        else
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set collide bitmasks, but the physics "
                   << "engine doesn't support feature [CollisionFilterMask]. "
                   << "Collision bitmasks will be ignored." << std::endl;
            informed = true;
          }
        }
        return true;
      });

  for (const Entity &entity : olderCollideBitmaskCmdsToRemove)
  {
    _ecm.RemoveComponent<components::CollideBitmaskCmd>(entity);
  }

  // Update Category Bitmasks
  auto olderCategoryBitmaskCmdsToRemove =
      std::move(this->categoryBitmaskCmdsToRemove);
  this->categoryBitmaskCmdsToRemove.clear();

  _ecm.Each<components::Collision, components::CategoryBitmaskCmd>(
      [&](const Entity &_entity, const components::Collision *,
          const components::CategoryBitmaskCmd *_bitmaskCmd) -> bool
      {
        this->categoryBitmaskCmdsToRemove.insert(_entity);
        auto filterMaskFeature =
            this->entityCollisionMap.EntityCast<CollisionMaskFeatureList>(
                _entity);
        if (filterMaskFeature)
        {
          filterMaskFeature->SetCategoryFilterMask(_bitmaskCmd->Data());
        }
        else
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set category bitmasks, but the physics "
                   << "engine doesn't support feature [CategoryFilterMask]. "
                   << "Category bitmasks will be ignored." << std::endl;
            informed = true;
          }
        }
        return true;
      });

  for (const Entity &entity : olderCategoryBitmaskCmdsToRemove)
  {
    _ecm.RemoveComponent<components::CategoryBitmaskCmd>(entity);
  }

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

  // update Static State
  auto olderStaticStatesToRemove = std::move(this->staticCmdsToRemove);
  this->staticCmdsToRemove.clear();

  _ecm.Each<components::Model,
    components::StaticCmd,
    components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::StaticCmd *_staticCmd,
          const components::Name *_name)->bool
      {
        this->staticCmdsToRemove.insert(_entity);

        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        auto modelStaticStateFeature =
          this->entityModelMap.EntityCast<StaticStateFeatureList>(_entity);

        if (!modelStaticStateFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set static state, but the physics "
                   << "engine doesn't support feature "
                   << "[ModelStaticState]. Static state won't be populated."
                   << " " << _name->Data()
                   << std::endl;
            informed = true;
          }

          return true;
        }

        // Make sure to update the set of staticEntities
        bool isStatic = this->staticEntities.find(_entity) !=
            this->staticEntities.end();
        if (isStatic != _staticCmd->Data())
        {
          sim::Model model(_entity);
          auto links = model.Links(_ecm);

          if (isStatic)
          {
            this->staticEntities.erase(_entity);
            for (const auto &linkEntity : links)
            {
              this->staticEntities.erase(linkEntity);
            }
          }
          else
          {
            this->staticEntities.insert(_entity);
            for (const auto &linkEntity : links)
            {
              this->staticEntities.insert(linkEntity);

              // Zero out velocities in ECM when making static.
              // This is needed because the physics engine (e.g., DART)
              // freezes the model but may keep stale velocities in its
              // body node cache. By zeroing them here and adding links to
              // staticEntities, we ensure they stay zero.
              auto linVelComp =
                  _ecm.Component<components::WorldLinearVelocity>(
                      linkEntity);
              if (linVelComp)
              {
                linVelComp->SetData(math::Vector3d::Zero, this->vec3Eql);
                _ecm.SetChanged(linkEntity,
                    components::WorldLinearVelocity::typeId,
                    ComponentState::OneTimeChange);
              }
              auto angVelComp =
                  _ecm.Component<components::WorldAngularVelocity>(
                      linkEntity);
              if (angVelComp)
              {
                angVelComp->SetData(math::Vector3d::Zero, this->vec3Eql);
                _ecm.SetChanged(linkEntity,
                    components::WorldAngularVelocity::typeId,
                    ComponentState::OneTimeChange);
              }
            }
          }
        }

        modelStaticStateFeature->SetStatic(_staticCmd->Data());
        return true;
      });

  // Remove static commands from previous iteration. We let them rotate one
  // iteration so other systems have a chance to react to them too.
  for (const Entity &entity : olderStaticStatesToRemove)
  {
    _ecm.RemoveComponent<components::StaticCmd>(entity);
  }

  // update Gravity enabled
  auto olderGravityEnabledCmdsToRemove =
    std::move(this->gravityEnabledCmdsToRemove);
  this->gravityEnabledCmdsToRemove.clear();

  _ecm.Each<components::Model,
    components::GravityEnabledCmd,
    components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::GravityEnabledCmd *_gravityEnabledCmd,
          const components::Name *_name)->bool
      {
        this->gravityEnabledCmdsToRemove.insert(_entity);

        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        auto modelGravityEnabledFeature =
          this->entityModelMap.EntityCast<GravityEnabledFeatureList>(
            _entity);

        if (!modelGravityEnabledFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set gravity enabled, but the physics "
                   << "engine doesn't support feature "
                   << "[GravityEnabled]. Gravity state won't be populated."
                   << " " << _name->Data()
                   << std::endl;
            informed = true;
          }

          return true;
        }
        modelGravityEnabledFeature->SetGravityEnabled(
            _gravityEnabledCmd->Data());
        return true;
      });

  // update Link Gravity enabled
  _ecm.Each<components::Link,
    components::GravityEnabledCmd,
    components::Name>(
      [&](const Entity &_entity, const components::Link *,
          const components::GravityEnabledCmd *_gravityEnabledCmd,
          const components::Name *_name)->bool
      {
        this->gravityEnabledCmdsToRemove.insert(_entity);

        auto linkPtrPhys = this->entityLinkMap.Get(_entity);
        if (nullptr == linkPtrPhys)
          return true;

        auto linkGravityEnabledFeature =
          this->entityLinkMap.EntityCast<GravityEnabledFeatureList>(
            _entity);

        if (!linkGravityEnabledFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set link gravity enabled, but the physics "
                   << "engine doesn't support feature "
                   << "[GravityEnabled]. Gravity state won't be populated."
                   << " " << _name->Data()
                   << std::endl;
            informed = true;
          }

          // Continue loop so commands are marked for removal in next iteration
          return true;
        }
        linkGravityEnabledFeature->SetGravityEnabled(
            _gravityEnabledCmd->Data());
        return true;
      });

  // Remove gravity enabled commands from previous iteration. We let them
  // rotate one iteration so other systems have a chance to react to them too.
  for (const Entity &entity : olderGravityEnabledCmdsToRemove)
  {
    _ecm.RemoveComponent<components::GravityEnabledCmd>(entity);
  }

  // update Collision enabled
  auto olderCollisionEnabledCmdsToRemove =
    std::move(this->collisionEnabledCmdsToRemove);
  this->collisionEnabledCmdsToRemove.clear();

  _ecm.Each<components::Model,
    components::CollisionEnabledCmd,
    components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::CollisionEnabledCmd *_collisionEnabledCmd,
          const components::Name *_name)->bool
      {
        this->collisionEnabledCmdsToRemove.insert(_entity);

        auto modelPtrPhys = this->entityModelMap.Get(_entity);
        if (nullptr == modelPtrPhys)
          return true;

        auto modelCollisionEnabledFeature =
          this->entityModelMap.EntityCast<ModelCollisionEnabledFeatureList>(
            _entity);

        if (!modelCollisionEnabledFeature)
        {
          static bool informed{false};
          if (!informed)
          {
            gzdbg << "Attempting to set collision enabled, but the physics "
                   << "engine doesn't support feature "
                   << "[ModelCollisionEnabled]. Collision state won't be "
                   << "populated. " << _name->Data()
                   << std::endl;
            informed = true;
          }

          return true;
        }
        modelCollisionEnabledFeature->SetCollisionEnabled(
            _collisionEnabledCmd->Data());

        // Reflect the applied state in the CollisionEnabled component so
        // queries via Model::CollisionEnabled() return the latest value.
        auto stateComp =
            _ecm.Component<components::CollisionEnabled>(_entity);
        if (stateComp == nullptr)
        {
          _ecm.CreateComponent(_entity,
              components::CollisionEnabled(_collisionEnabledCmd->Data()));
        }
        else
        {
          stateComp->SetData(_collisionEnabledCmd->Data(),
              [](const bool &, const bool &){return false;});
          _ecm.SetChanged(_entity,
              components::CollisionEnabled::typeId,
              ComponentState::OneTimeChange);
        }
        return true;
      });

  // Remove collision enabled commands from previous iteration. We let them
  // rotate one iteration so other systems have a chance to react to them too.
  for (const Entity &entity : olderCollisionEnabledCmdsToRemove)
  {
    _ecm.RemoveComponent<components::CollisionEnabledCmd>(entity);
  }

  // Update model pose
  auto olderWorldPoseCmdsToRemove = std::move(this->worldPoseCmdsToRemove);
  this->worldPoseCmdsToRemove.clear();

  _ecm.Each<components::Model, components::WorldPoseCmd>(
      [&](const Entity &_entity, const components::Model *,
          const components::WorldPoseCmd *_poseCmd)
      {
        this->worldPoseCmdsToRemove.insert(_entity);
        // Check if the model contains any plane collision geometry.
        // If so, reject set_pose to prevent physics engine crash.
        if (this->ModelContainsPlaneCollision(_entity, _ecm))
        {
          gzerr << "SetPose is not supported for models containing "
                << "plane collision geometry. Entity [" << _entity << "]. "
                << "Request ignored"
                << std::endl;
          return true;
        }
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
        math::Pose3d worldPoseCmd = _poseCmd->Data();
        if (!worldPoseCmd.Pos().IsFinite() || !worldPoseCmd.Rot().IsFinite() ||
            worldPoseCmd.Rot() == math::Quaterniond::Zero)
        {
          gzerr << "Unable to set world pose. Invalid pose value: "
                << worldPoseCmd << std::endl;
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

        freeGroup->SetWorldPose(math::eigen3::convert(worldPoseCmd *
                                linkPose));

        // Process pose commands for static models here, as one-time changes
        if (this->staticEntities.find(_entity) != this->staticEntities.end())
        {
          auto worldPoseComp = _ecm.Component<components::Pose>(_entity);
          if (worldPoseComp)
          {
            auto state = worldPoseComp->SetData(worldPoseCmd,
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

  // Reset link linear velocity in world frame
  _ecm.Each<components::Link, components::WorldLinearVelocityReset>(
      [&](const Entity &_entity, const components::Link *,
          const components::WorldLinearVelocityReset *_worldlinearvelocityreset)
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

        auto rootLinkPtr = freeGroup->RootLink();
        if (rootLinkPtr != linkPtrPhys)
        {
          gzdbg << "Attempting to set linear velocity for link [ " << _entity
                 << " ] which is not root link of the FreeGroup."
                 << "Velocity won't be set."
                 << std::endl;

          return true;
        }

        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        auto worldLinearVelFeature = this->entityFreeGroupMap
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

        // Linear velocity in world frame
        math::Vector3d worldLinearVel = _worldlinearvelocityreset->Data();

        worldLinearVelFeature->SetWorldLinearVelocity(
            math::eigen3::convert(worldLinearVel));

        return true;
      });

  // Reset link angular velocity in world frame
  _ecm.Each<components::Link, components::WorldAngularVelocityReset>(
      [&](const Entity &_entity, const components::Link *,
          const components::WorldAngularVelocityReset
          *_worldangularvelocityreset)
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

        auto rootLinkPtr = freeGroup->RootLink();
        if(rootLinkPtr != linkPtrPhys)
        {
          gzdbg << "Attempting to set angular velocity for link [ " << _entity
                 << " ] which is not root link of the FreeGroup."
                 << "Velocity won't be set."
                 << std::endl;

          return true;
        }

        this->entityFreeGroupMap.AddEntity(_entity, freeGroup);

        auto worldAngularVelFeature = this->entityFreeGroupMap
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
        // Angular velocity in world frame
        math::Vector3d worldAngularVel = _worldangularvelocityreset->Data();

        worldAngularVelFeature->SetWorldAngularVelocity(
            math::eigen3::convert(worldAngularVel));

        return true;
      });

  // Populate bounding box info
  this->UpdateLinksBoundingBoxes(_ecm);
  this->UpdateModelsBoundingBoxes(_ecm);
}  // NOLINT readability/fn_size
// TODO (azeey) Reduce size of function and remove the NOLINT above

//////////////////////////////////////////////////
