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
                     << "physics engine doesn't support feature "
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
                     << "physics engine doesn't support feature "
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
        auto solverItersComp =
            _ecm.Component<components::PhysicsSolverIterations>(_entity);
        if (solverItersComp)
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
                     << "physics engine doesn't support feature "
                     << "[SolverFeature]. Options will be ignored."
                     << std::endl;
              informed = true;
            }
          }
          else
          {
            solverFeature->SetSolverIterations(solverItersComp->Data());
          }
        }

        auto physicsComp =
            _ecm.Component<components::Physics>(_entity);
        if (physicsComp)
        {
          auto maxContactsFeature =
              this->entityWorldMap.EntityCast<
              CollisionPairMaxContactsFeatureList>(_entity);
          if (!maxContactsFeature)
          {
            static bool informed{false};
            if (!informed)
            {
              gzdbg << "Attempting to set physics options, but the "
                     << "physics engine doesn't support feature "
                     << "[CollisionPairMaxContacts]. "
                     << "Options will be ignored."
                     << std::endl;
              informed = true;
            }
          }
          else
          {
            maxContactsFeature->SetCollisionPairMaxContacts(
              physicsComp->Data().MaxContacts());
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
  std::map<Entity, std::tuple<const components::Name*, const components::Pose*,
    const components::ParentEntity*>> modelEntities;

  _ecm.EachNew<components::Model, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        if (!_ecm.EntityHasComponentType(_entity, components::Recreate::typeId))
          modelEntities.insert({_entity,
              std::make_tuple(_name, _pose, _parent)});
        return true;
      });

  for (const auto &[_entity, components] : modelEntities)
  {
    const auto [_name, _pose, _parent] = components;

    // Check if model already exists
    if (this->entityModelMap.HasEntity(_entity))
    {
      if (_warnIfEntityExists)
      {
        gzwarn << "Model entity [" << _entity
                << "] marked as new, but it's already on the map."
                << std::endl;
      }
      continue;
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
                   << "physics engine doesn't support feature "
                   << "[ConstructSdfNestedModelFeature]. "
                   << "Nested model will be ignored."
                   << std::endl;
            informed = true;
          }
          continue;
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
          continue;
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
        continue;
      }
    }
  }
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

        // get link gravity
        const components::GravityEnabled *gravityEnabled =
            _ecm.Component<components::GravityEnabled>(_entity);
        if (nullptr != gravityEnabled)
        {
          // gravityEnabled set in SdfEntityCreator::CreateEntities()
          link.SetEnableGravity(gravityEnabled->Data());
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

          const common::Mesh *mesh = loadMesh(*meshSdf);
          if (!mesh)
            return true;

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
          // check if heightmap is an image
          if (common::isSupportedImageHeightmapFileExtension(fullPath))
          {
            data = common::loadHeightmapData(fullPath);
            if (!data)
              return true;
          }
          // DEM
          else
          {
            auto worldEntity = _ecm.EntityByComponents(components::World());
            auto sphericalCoordinatesComponent =
              _ecm.Component<components::SphericalCoordinates>(
                worldEntity);
            math::SphericalCoordinates sphericalCoordinates;
            if (sphericalCoordinatesComponent)
              sphericalCoordinates = sphericalCoordinatesComponent->Data();
            data = common::loadHeightmapData(fullPath, sphericalCoordinates);
            if (!data)
              return true;
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

          // Check if mimic constraint should be applied to this joint's axes.
          using AxisIndex = std::size_t;
          std::map<AxisIndex, sdf::JointAxis> jointAxisByIndex;
          auto jointAxis = _ecm.Component<components::JointAxis>(_entity);
          auto jointAxis2 = _ecm.Component<components::JointAxis2>(_entity);

          if (jointAxis)
          {
            jointAxisByIndex[0] = jointAxis->Data();
          }

          if (jointAxis2)
          {
            jointAxisByIndex[1] = jointAxis2->Data();
          }

          for (const auto &[axisIndex, axis] : jointAxisByIndex)
          {
            if (auto mimic = axis.Mimic())
            {
              auto jointPtrMimic = this->entityJointMap
                  .EntityCast<MimicConstraintJointFeatureList>(existingJoint);
              if (jointPtrMimic)
              {
                const auto leaderJoint =
                    basicModelPtrPhys->GetJoint(mimic->Joint());
                std::size_t leaderAxis = 0;
                if (mimic->Axis() == "axis2")
                {
                  leaderAxis = 1;
                }
                jointPtrMimic->SetMimicConstraint(axisIndex,
                    leaderJoint,
                    leaderAxis,
                    mimic->Multiplier(),
                    mimic->Offset(),
                    mimic->Reference());
              }
              else
              {
                static bool informed{false};
                if (!informed)
                {
                  gzerr << "Attempting to create a mimic constraint for joint ["
                        << _name->Data()
                        << "] but the chosen physics engine does not support "
                        << "mimic constraints, so no constraint will be "
                        << "created."
                        << std::endl;
                  informed = true;
                }
              }
            }
          }

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

          bool enforce = _ecm.ComponentData<
              components::DetachableJointEnforceFixedConstraint>(
              _entity).value_or(this->enforceFixedConstraint);
          if (enforce)
          {
            auto jointPtrWeld = this->entityJointMap
                .EntityCast<SetFixedJointWeldChildToParentFeatureList>(_entity);
            if (!jointPtrWeld)
            {
              static bool informed{false};
              if (!informed)
              {
                gzerr << "Attempting to enforce fixed constraint in a "
                      << "detachable joint but the physics engine doesn't "
                      << "support feature "
                      << "[SetFixedJointWeldChildToParentFeature]. "
                      << "The fixed constraint in detachable joints will not "
                      << "be enforced." << std::endl;
                informed = true;
              }
            }
            else
            {
              jointPtrWeld->SetWeldChildToParent(true);
            }
          }
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
  // Remove detachable joints. Do this before removing models otherwise the
  // physics engine may not find the links associated with the detachable
  // joints.
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
}

//////////////////////////////////////////////////
