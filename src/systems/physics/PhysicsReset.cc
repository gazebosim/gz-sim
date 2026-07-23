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

void PhysicsPrivate::ResetPhysics(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::ResetPhysics");
  // Clear worldPoseCmdsToRemove because pose commands that were issued before
  // the reset will be ignored.
  this->linkWorldPoses.clear();
  this->canonicalLinkModelTracker = CanonicalLinkModelTracker();
  this->modelWorldPoses.clear();
  this->worldPoseCmdsToRemove.clear();
  this->staticCmdsToRemove.clear();
  this->collideBitmaskCmdsToRemove.clear();
  this->categoryBitmaskCmdsToRemove.clear();
  this->gravityEnabledCmdsToRemove.clear();
  this->collisionEnabledCmdsToRemove.clear();

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

  input.Get<std::chrono::steady_clock::duration>() = _dt;

  for (const auto &world : this->entityWorldMap.Map())
  {
    world.second->Step(this->stepOutput, state, input);
  }

  return this->stepOutput;
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
bool PhysicsPrivate::ModelContainsPlaneCollision(const Entity &_modelEntity,
    EntityComponentManager &_ecm) const
{
  sim::Model model(_modelEntity);
  for (const auto &linkEntity : model.Links(_ecm))
  {
    sim::Link link(linkEntity);
    for (const auto &collisionEntity : link.Collisions(_ecm))
    {
      auto geomComp = _ecm.Component<components::Geometry>(collisionEntity);
      if (geomComp && geomComp->Data().Type() == sdf::GeometryType::PLANE)
      {
        return true;
      }
    }
  }
  return false;
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
