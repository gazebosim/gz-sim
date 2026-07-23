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

void PhysicsPrivate::UpdateCollisions(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::UpdateCollisions");
  // Quit early if the ContactData component hasn't been created. This means
  // there are no systems that need contact information
  if (!_ecm.HasComponentType(components::ContactSensorData::typeId))
    return;

  // Also check if any entity currently has a ContactSensorData component.
  bool needContactSensorData = false;
  _ecm.Each<components::Collision, components::ContactSensorData>(
      [&](const Entity &/*unused*/, components::Collision *,
          components::ContactSensorData */*unused*/) -> bool
      {
        needContactSensorData = true;
        return false;
      });
  if (!needContactSensorData)
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
    const auto &contactPoint =
        contactComposite.Get<WorldShapeType::ContactPoint>();
    const auto &extraContactData = contactComposite.Query<ExtraContactData>();
    auto coll1Entity = this->entityCollisionMap.GetByPhysicsId(
        contactPoint.collision1->EntityID());
    auto coll2Entity = this->entityCollisionMap.GetByPhysicsId(
        contactPoint.collision2->EntityID());

    if (coll1Entity != kNullEntity && coll2Entity != kNullEntity)
    {
      ContactData data = std::make_pair(&contactPoint, extraContactData);
      entityContactMap[coll1Entity][coll2Entity].push_back(data);
      entityContactMap[coll2Entity][coll1Entity].push_back(data);
    }
  }

  // Go through each collision entity that has a ContactData component and
  // set the component value to the list of contacts that correspond to
  // the collision entity
  _ecm.Each<components::Collision, components::ContactSensorData>(
      [&](const Entity &_collEntity1, components::Collision *,
          components::ContactSensorData *_contacts) -> bool
      {
        const auto contactMapIt = entityContactMap.find(_collEntity1);
        if (contactMapIt == entityContactMap.end())
        {
          // Clear the last contact data
          bool changed = _contacts->Data().contact_size() > 0;
          if (changed)
          {
            _contacts->Data().Clear();
          }

          auto state = changed ?
            ComponentState::PeriodicChange :
            ComponentState::NoChange;
          _ecm.SetChanged(
            _collEntity1, components::ContactSensorData::typeId, state);
          return true;
        }

        const auto &contactMap = contactMapIt->second;

        bool changed = !this->contactsEql(_contacts->Data(), contactMap);
        auto state = changed ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
        _ecm.SetChanged(
          _collEntity1, components::ContactSensorData::typeId, state);

        // If contacts are unchanged, no need to update them again
        if (!changed)
        {
          return true;
        }

        // If contacts have changed, first clear data then add contacts
        _contacts->Data().Clear();
        for (const auto &[collEntity2, contactData] : contactMap)
        {
          msgs::Contact *contactMsg = _contacts->Data().add_contact();
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
            position->set_x(contact.first->point.x());
            position->set_y(contact.first->point.y());
            position->set_z(contact.first->point.z());

            // Check if the extra contact data exists,
            // since not all physics engines support it.
            // Then, fill the msg with extra data.
            if(contact.second != nullptr)
            {
              auto *normal = contactMsg->add_normal();
              normal->set_x(contact.second->normal.x());
              normal->set_y(contact.second->normal.y());
              normal->set_z(contact.second->normal.z());

              auto *wrench = contactMsg->add_wrench();
              auto *body1Wrench = wrench->mutable_body_1_wrench();
              auto *body1Force = body1Wrench->mutable_force();
              body1Force->set_x(contact.second->force.x());
              body1Force->set_y(contact.second->force.y());
              body1Force->set_z(contact.second->force.z());

              // The force on the second body is equal and opposite
              auto *body2Wrench = wrench->mutable_body_2_wrench();
              auto *body2Force = body2Wrench->mutable_force();
              body2Force->set_x(-contact.second->force.x());
              body2Force->set_y(-contact.second->force.y());
              body2Force->set_z(-contact.second->force.z());

              contactMsg->add_depth(contact.second->depth);
            }
          }
        }
        return true;
      });
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateRayIntersections(EntityComponentManager &_ecm)
{
  GZ_PROFILE("PhysicsPrivate::UpdateRayIntersections");
  // Quit early if the RaycastData component hasn't been created.
  // This means there are no systems that need raycasting information
  if (!_ecm.HasComponentType(components::RaycastData::typeId))
    return;

  // Assume that there is only one world entity
  Entity worldEntity = _ecm.EntityByComponents(components::World());

  if (!this->entityWorldMap.HasEntity(worldEntity))
  {
    gzwarn << "Failed to find world [" << worldEntity << "]." << std::endl;
    return;
  }

  // Prefer the batched ray intersection API when available.
  auto worldBatchRayFeature =
      this->entityWorldMap
        .EntityCast<BatchRayIntersectionFeatureList>(worldEntity);

  if (worldBatchRayFeature)
  {
    using BatchWorld = physics::World3d<BatchRayIntersectionFeatureList>;
    using RayQuery = BatchWorld::RayQuery;
    using RayIntersection = BatchWorld::RayIntersection;

    _ecm.Each<components::RaycastData,
              components::NeedsRaycast,
              components::WorldPose>(
        [&](const Entity &_entity,
            components::RaycastData *_raycastData,
            components::NeedsRaycast *_needsRaycast,
            const components::WorldPose *_worldPose) -> bool
        {
          if (!_needsRaycast->Data())
            return true;
          _needsRaycast->Data() = false;

          const auto &rays = _raycastData->Data().rays;
          auto &results = _raycastData->Data().results;
          results.clear();
          results.reserve(rays.size());

          const auto &entityWorldPose = _worldPose->Data();

          auto &cache = this->batchRayCache[_entity];
          auto &batchInput = cache.input;
          auto &batchOutput = cache.output;
          batchInput.clear();
          batchInput.reserve(rays.size());
          for (const auto &ray : rays)
          {
            RayQuery q;
            q.origin = math::eigen3::convert(
              entityWorldPose.Pos() +
              entityWorldPose.Rot().RotateVector(ray.start));
            q.target = math::eigen3::convert(
              entityWorldPose.Pos() +
              entityWorldPose.Rot().RotateVector(ray.end));
            batchInput.push_back(q);
          }

          worldBatchRayFeature->GetBatchRayIntersectionFromLastStep(
            batchInput, batchOutput);

          for (const auto &hit :
              batchOutput.Get<std::vector<RayIntersection>>())
          {
            auto &result = results.emplace_back();

            const math::Vector3d intersectionPoint =
              math::eigen3::convert(hit.point);
            result.point = entityWorldPose.Rot().RotateVectorReverse(
              intersectionPoint - entityWorldPose.Pos());

            result.fraction = hit.fraction;

            const math::Vector3d normal = math::eigen3::convert(hit.normal);
            result.normal = entityWorldPose.Rot().RotateVectorReverse(normal);
          }
          return true;
        });
    return;
  }

  // Fallback: single-ray intersection
  auto worldRayIntersectionFeature =
      this->entityWorldMap.EntityCast<RayIntersectionFeatureList>(worldEntity);

  if (!worldRayIntersectionFeature)
  {
    static bool informed{false};
    if (!informed)
    {
      gzdbg << "Attempting process ray intersections, but the physics "
             << "engine doesn't support ray intersection features. "
             << "Ray intersections won't be computed."
             << std::endl;
      informed = true;
    }
    return;
  }

  // Go through each entity that has a RaycastData component, trace the
  // rays and store the results
  _ecm.Each<components::RaycastData,
            components::NeedsRaycast,
            components::WorldPose>(
      [&](const Entity & /*_entity*/,
          components::RaycastData *_raycastData,
          components::NeedsRaycast *_needsRaycast,
          const components::WorldPose *_worldPose) -> bool
      {
        if (!_needsRaycast->Data())
          return true;
        _needsRaycast->Data() = false;

        const auto &rays = _raycastData->Data().rays;

        auto &results = _raycastData->Data().results;
        results.clear();
        results.reserve(rays.size());

        const auto &entityWorldPose = _worldPose->Data();

        for (const auto &ray : rays)
        {
          const math::Vector3d rayStart = entityWorldPose.Pos() +
            entityWorldPose.Rot().RotateVector(ray.start);
          const math::Vector3d rayEnd = entityWorldPose.Pos() +
            entityWorldPose.Rot().RotateVector(ray.end);

          auto rayIntersection =
            worldRayIntersectionFeature->GetRayIntersectionFromLastStep(
              math::eigen3::convert(rayStart),
              math::eigen3::convert(rayEnd));

          const auto rayIntersectionResult =
            rayIntersection.Get<
              physics::World3d<RayIntersectionFeatureList>::RayIntersection>();

          auto &result = results.emplace_back();

          const math::Vector3d intersectionPoint =
            math::eigen3::convert(rayIntersectionResult.point);
          result.point = entityWorldPose.Rot().RotateVectorReverse(
            intersectionPoint - entityWorldPose.Pos());

          result.fraction = rayIntersectionResult.fraction;

          const math::Vector3d normal =
            math::eigen3::convert(rayIntersectionResult.normal);
          result.normal = entityWorldPose.Rot().RotateVectorReverse(normal);
        }
        return true;
      });

  _ecm.EachRemoved<components::RaycastData>(
      [&](const Entity &_entity,
          const components::RaycastData *) -> bool
      {
        this->batchRayCache.erase(_entity);
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

  using Feature = physics::SetContactPropertiesCallbackFeature;
  using FeatureList = SetContactPropertiesCallbackFeatureList;
  using GCFeatureWorld = GCFeature::World<Policy, FeatureList>;
  using ContactPoint = GCFeatureWorld::ContactPoint;

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

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateLinksBoundingBoxes(EntityComponentManager &_ecm)
{
  // Only compute bounding box if component exists to avoid unnecessary
  // computation for links.
  _ecm.Each<components::Link, components::AxisAlignedBox>(
    [&](const Entity &_entity, const components::Link *,
        components::AxisAlignedBox *_bbox)
    {
      auto linkPhys = this->entityLinkMap.Get(_entity);
      if (!linkPhys)
      {
        gzwarn << "Failed to find link [" << _entity << "]." << std::endl;
        return true;
      }

      auto bbLink = this->entityLinkMap.EntityCast<
        LinkBoundingBoxFeatureList>(_entity);

      // Bounding box expressed in the link frame
      math::AxisAlignedBox bbox;

      if (bbLink)
      {
        bbox = math::eigen3::convert(
          bbLink->GetAxisAlignedBoundingBox(linkPhys->GetFrameID()));
      }
      else
      {
        static bool informed{false};
        if (!informed)
        {
          gzdbg << "Attempting to get a bounding box, but the physics "
                 << "engine doesn't support feature "
                 << "[GetLinkBoundingBox]. Link bounding boxes will be "
                 << "computed from their collision shapes based on their "
                 << "geometry properties in SDF." << std::endl;
          informed = true;
        }

        // Fallback to SDF API to get the link AABB from its collision shapes.
        // If the link has no collision shapes, the AABB will be invalid.
        bbox = gz::sim::Link(_entity).ComputeAxisAlignedBox(_ecm).value_or(
          math::AxisAlignedBox());
      }

      auto state = _bbox->SetData(bbox, this->axisAlignedBoxEql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(_entity, components::AxisAlignedBox::typeId, state);

      return true;
    });
}

//////////////////////////////////////////////////
void PhysicsPrivate::UpdateModelsBoundingBoxes(EntityComponentManager &_ecm)
{
  // Only compute bounding box if component exists to avoid unnecessary
  // computation for models.
  _ecm.Each<components::Model, components::AxisAlignedBox>(
    [&](const Entity &_entity, const components::Model *,
        components::AxisAlignedBox *_bbox)
    {
      if (!this->entityModelMap.HasEntity(_entity))
      {
        gzwarn << "Failed to find model [" << _entity << "]." << std::endl;
        return true;
      }

      auto bbModel = this->entityModelMap.EntityCast<
        ModelBoundingBoxFeatureList>(_entity);

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

      // Bounding box expressed in the world frame
      math::AxisAlignedBox bbox =
          math::eigen3::convert(bbModel->GetAxisAlignedBoundingBox());
      auto state = _bbox->SetData(bbox, this->axisAlignedBoxEql) ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
      _ecm.SetChanged(_entity, components::AxisAlignedBox::typeId, state);

      return true;
    });
}

