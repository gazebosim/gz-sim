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

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/NameSet.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/RenderState.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "LevelManager.hh"

using namespace ignition;
using namespace gazebo;


/////////////////////////////////////////////////
LevelManager::LevelManager(EntityComponentManager &_ecm,
                           const sdf::World *_world)
    : entityCompMgr(&_ecm), sdfWorld(_world)
{
  // Do nothing
}

/////////////////////////////////////////////////
void LevelManager::ReadLevelPerformerInfo()
{
  // World entity
  EntityId worldEntity = this->entityCompMgr->CreateEntity();

  // World components
  this->entityCompMgr->CreateComponent(worldEntity, components::World());
  this->entityCompMgr->CreateComponent(
      worldEntity, components::Name(this->sdfWorld->Name()));

  auto worldElem = this->sdfWorld->Element();

  for (auto performer = worldElem->GetElement("performer"); performer;
       performer = performer->GetNextElement("performer"))
  {
    std::string name = performer->Get<std::string>("name");

    EntityId performerEntity = this->entityCompMgr->CreateEntity();
    // We use the ref to create a parent entity component later on
    std::string ref = performer->GetElement("ref")->GetValue()->GetAsString();
    if (this->performerMap.find(ref) == this->performerMap.end())
    {
      this->performerMap[ref] = performerEntity;
    }
    else
    {
      std::string otherPerformer =
          this->entityCompMgr
              ->Component<components::Name>(this->performerMap[ref])
              ->Data();

      ignerr << "Found multiple performers (" << name << " and "
             << otherPerformer << ")referring to the same entity\n";
    }

    sdf::Geometry geometry;
    geometry.Load(performer->GetElement("geometry"));
    this->entityCompMgr->CreateComponent(performerEntity,
                                         components::Performer());
    this->entityCompMgr->CreateComponent(performerEntity,
                                         components::Name(name));
    this->entityCompMgr->CreateComponent(performerEntity,
                                         components::Geometry(geometry));
  }

  // This holds all entities referenced by at least one level. This will be used
  // later to create the default level.
  std::unordered_set<std::string> entityNamesInLevels;

  for (auto level = worldElem->GetElement("level"); level;
       level = level->GetNextElement("level"))
  {
    auto name = level->Get<std::string>("name");
    auto pose = level->Get<math::Pose3d>("pose");
    sdf::Geometry geometry;
    geometry.Load(level->GetElement("geometry"));
    std::unordered_set<std::string> entityNames;
    for (auto ref = level->GetElement("ref"); ref;
         ref = ref->GetNextElement("ref"))
    {
      std::string entityName = ref->GetValue()->GetAsString();
      // TODO(addisu) Make sure the names are unique
      entityNames.insert(entityName);

      entityNamesInLevels.insert(entityName);
    }

    // Entity
    EntityId levelEntity = this->entityCompMgr->CreateEntity();

    // Components
    this->entityCompMgr->CreateComponent(levelEntity, components::Level());
    this->entityCompMgr->CreateComponent(levelEntity, components::Pose(pose));
    this->entityCompMgr->CreateComponent(levelEntity, components::Name(name));
    this->entityCompMgr->CreateComponent(levelEntity,
                                         components::ParentEntity(worldEntity));
    this->entityCompMgr->CreateComponent(levelEntity,
                                         components::NameSet(entityNames));
    this->entityCompMgr->CreateComponent(levelEntity,
                                         components::Geometry(geometry));

    // Visualization
    // The levelLinkEntity and levelVisualEntities are only used for visualizing
    // the level for debugging. They may be removed if this is no longer needed.
    // TODO(addisu): This won't work with physics
    const bool visualizeLevels = false;
    if (visualizeLevels)
    {
      EntityId levelLinkEntity = this->entityCompMgr->CreateEntity();
      this->entityCompMgr->CreateComponent(levelLinkEntity, components::Link());
      this->entityCompMgr->CreateComponent(levelLinkEntity,
                                           components::Pose(math::Pose3d()));
      this->entityCompMgr->CreateComponent(levelLinkEntity,
                                           components::Name(name + "::link"));
      this->entityCompMgr->CreateComponent(
          levelLinkEntity, components::ParentEntity(levelEntity));

      EntityId levelVisualEntity = this->entityCompMgr->CreateEntity();
      this->entityCompMgr->CreateComponent(levelVisualEntity,
                                           components::Visual());
      this->entityCompMgr->CreateComponent(
          levelVisualEntity, components::ParentEntity(levelLinkEntity));
      this->entityCompMgr->CreateComponent(levelVisualEntity,
                                           components::Pose(math::Pose3d()));
      this->entityCompMgr->CreateComponent(levelVisualEntity,
                                           components::Name(name + "::visual"));
      this->entityCompMgr->CreateComponent(levelVisualEntity,
                                           components::Geometry(geometry));
      sdf::Material mat;
      mat.SetAmbient({1, 1, 1, 0.1});
      this->entityCompMgr->CreateComponent(levelVisualEntity,
                                           components::Material(mat));
    }
  }

  // Create the default level. This level contains all entities not contained by
  // any other level.
  EntityId defaultLevel = this->entityCompMgr->CreateEntity();
  std::unordered_set<std::string> entityNamesInDefault;

  // Go through all entities in the world and find ones not in the
  // set entityNamesInLevels

  // Models
  for (uint64_t modelIndex = 0; modelIndex < this->sdfWorld->ModelCount();
        ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->sdfWorld->ModelByIndex(modelIndex);
    // If model is a performer, it will be handled separately
    if (this->performerMap.find(model->Name()) != this->performerMap.end())
      continue;

    if (entityNamesInLevels.find(model->Name()) == entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(model->Name());
    }
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < this->sdfWorld->LightCount();
        ++lightIndex)
  {
    auto light = this->sdfWorld->LightByIndex(lightIndex);
    if (entityNamesInLevels.find(light->Name()) == entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(light->Name());
    }
  }
  // Components
  this->entityCompMgr->CreateComponent(defaultLevel, components::Level());
  this->entityCompMgr->CreateComponent(defaultLevel,
                                       components::ParentEntity(worldEntity));
  this->entityCompMgr->CreateComponent(
      defaultLevel, components::NameSet(entityNamesInDefault));

  // Add default level to levels to load
  this->levelsToLoad.insert(defaultLevel);
}

/////////////////////////////////////////////////
void LevelManager::CreatePerformers()
{
  EntityId worldEntity = kNullEntity;
  this->entityCompMgr->Each<components::World>(
      [&worldEntity](const EntityId _entity,
                     const components::World *) -> bool
      {
        worldEntity = _entity;
        return false;
      });

  // Models
  for (uint64_t modelIndex = 0; modelIndex < this->sdfWorld->ModelCount();
        ++modelIndex)
  {
    auto model = this->sdfWorld->ModelByIndex(modelIndex);
    if (this->performerMap.find(model->Name()) != this->performerMap.end() )
    {
      EntityId modelEntity = LoadModel(*model, worldEntity);
      // Create a component on the performer entity that points to this model
      this->entityCompMgr->CreateComponent(
          this->performerMap[model->Name()],
          components::ParentEntity(modelEntity));
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::UpdateLevels()
{
  this->entityCompMgr->Each<components::Performer, components::Geometry,
                                   components::ParentEntity>(
      [&](const EntityId &, const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity *_parent) -> bool
      {
        auto pose = this->entityCompMgr->Component<components::Pose>(
            _parent->Data());
        // We assume the geometry contains a box.
        auto perfBox = _geometry->Data().BoxShape();

        math::AxisAlignedBox performerVolume{
             pose->Data().Pos() - perfBox->Size() / 2,
             pose->Data().Pos() + perfBox->Size() / 2};


        // loop through levels and check for intersections
        this->entityCompMgr->Each<components::Level, components::Pose,
                                         components::Geometry>(
            [&](const EntityId &_entity, const components::Level *,
                const components::Pose *_pose,
                const components::Geometry *_levelGeometry) -> bool
            {
              // Check if the performer is in this level
              // assume a box for now
              auto box = _levelGeometry->Data().BoxShape();
              auto center = _pose->Data().Pos();
              math::AxisAlignedBox region{center - box->Size() / 2,
                                          center + box->Size() / 2};

              if (region.Intersects(performerVolume))
              {
                // mark the level to be loaded if it's not currently active
                if (this->activeLevels.find(_entity) ==
                    this->activeLevels.end())
                {
                  this->levelsToLoad.insert(_entity);
                }
              }
              else
              {
                // mark the level to be unloaded if it's currently active
                if (this->activeLevels.find(_entity) !=
                    this->activeLevels.end())
                {
                  this->levelsToUnload.insert(_entity);
                }
              }
              return true;
            });

        return true;
      });

  // Filter the levelsToUnload so that if a level is marked to be loaded by one
  // performer check and marked to be unloaded by another, we don't end up
  // unloading it
  for (auto it = this->levelsToUnload.begin();
       it != this->levelsToUnload.end();)
  {
    if (this->levelsToLoad.find(*it) != this->levelsToLoad.end())
    {
      this->levelsToUnload.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::LoadActiveLevels()
{
  EntityId worldEntity = kNullEntity;
  this->entityCompMgr->Each<components::World>(
      [&worldEntity](const EntityId _entity,
                     const components::World *) -> bool
      {
        worldEntity = _entity;
        return false;
      });
  if (worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while loading levels\n";
    return;
  }

  // Create a union of all the entity name sets
  std::unordered_set<std::string> entityNamesUnion;
  for (auto level : this->levelsToLoad) {
    auto names = this->entityCompMgr->Component<components::NameSet>(level);
    if (names != nullptr)
    {
      entityNamesUnion.insert(names->Data().begin(), names->Data().end());
      this->activeLevels.insert(level);
    }
  }

  // Models
  for (uint64_t modelIndex = 0; modelIndex < this->sdfWorld->ModelCount();
       ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->sdfWorld->ModelByIndex(modelIndex);
    if (entityNamesUnion.find(model->Name()) != entityNamesUnion.end())
    {
      igndbg << "Creating model: " << model->Name() << "\n";
      LoadModel(*model, worldEntity);
    }
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < this->sdfWorld->LightCount();
        ++lightIndex)
  {
    auto light = this->sdfWorld->LightByIndex(lightIndex);
    if (entityNamesUnion.find(light->Name()) != entityNamesUnion.end())
    {
      LoadLight(*light, worldEntity);
    }
  }

  this->levelsToLoad.clear();
}

/////////////////////////////////////////////////
void LevelManager::UnloadInactiveLevels()
{
  // for (auto level : this->levelsToUnload) {
  //   this->UnloadLevel(level);
  // }
  this->levelsToUnload.clear();
}

/////////////////////////////////////////////////
EntityId LevelManager::LoadModel(const sdf::Model &_model,
                                 const EntityId _worldEntity)
{
  // Entity
  EntityId modelEntity = this->entityCompMgr->CreateEntity();

  // Components
  this->entityCompMgr->CreateComponent(modelEntity, components::Model());
  this->entityCompMgr->CreateComponent(modelEntity,
                                       components::Pose(_model.Pose()));
  this->entityCompMgr->CreateComponent(modelEntity,
                                       components::Name(_model.Name()));
  this->entityCompMgr->CreateComponent(modelEntity,
                                       components::ParentEntity(_worldEntity));
  this->entityCompMgr->CreateComponent(modelEntity,
                                       components::Static(_model.Static()));
  this->entityCompMgr->CreateComponent(modelEntity,
                                       components::RenderState(true));

  if (this->performerMap.find(_model.Name()) != this->performerMap.end())
  {
    // Make this _model a parent to the performer entity
    this->entityCompMgr->CreateComponent(this->performerMap[_model.Name()],
                                         components::ParentEntity(modelEntity));
  }

  // used to map link names to EntityIds
  std::unordered_map<std::string, EntityId> linkMap;

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model.LinkCount();
      ++linkIndex)
  {
    auto link = _model.LinkByIndex(linkIndex);

    // Entity
    EntityId linkEntity = this->entityCompMgr->CreateEntity();

    // Components
    this->entityCompMgr->CreateComponent(linkEntity, components::Link());
    this->entityCompMgr->CreateComponent(linkEntity,
                                         components::Pose(link->Pose()));
    this->entityCompMgr->CreateComponent(linkEntity,
                                         components::Name(link->Name()));
    this->entityCompMgr->CreateComponent(
        linkEntity, components::Inertial(link->Inertial()));
    this->entityCompMgr->CreateComponent(linkEntity,
                                         components::ParentEntity(modelEntity));
    if (linkIndex == 0)
    {
      this->entityCompMgr->CreateComponent(linkEntity,
                                           components::CanonicalLink());
    }

    linkMap.insert(std::pair(link->Name(), linkEntity));

    // Visuals
    for (uint64_t visualIndex = 0; visualIndex < link->VisualCount();
         ++visualIndex)
    {
      auto visual = link->VisualByIndex(visualIndex);

      // Entity
      EntityId visualEntity = this->entityCompMgr->CreateEntity();

      // Components
      this->entityCompMgr->CreateComponent(visualEntity, components::Visual());
      this->entityCompMgr->CreateComponent(visualEntity,
                                           components::Pose(visual->Pose()));
      this->entityCompMgr->CreateComponent(visualEntity,
                                           components::Name(visual->Name()));
      this->entityCompMgr->CreateComponent(
          visualEntity, components::ParentEntity(linkEntity));

      if (visual->Geom())
      {
        this->entityCompMgr->CreateComponent(
            visualEntity, components::Geometry(*visual->Geom()));
      }

      // \todo(louise) Populate with default material if undefined
      if (visual->Material())
      {
        this->entityCompMgr->CreateComponent(
            visualEntity, components::Material(*visual->Material()));
      }
    }

    // Collisions
    for (uint64_t collisionIndex = 0; collisionIndex < link->CollisionCount();
         ++collisionIndex)
    {
      auto collision = link->CollisionByIndex(collisionIndex);

      // Entity
      EntityId collisionEntity = this->entityCompMgr->CreateEntity();

      // Components
      this->entityCompMgr->CreateComponent(collisionEntity,
                                           components::Collision());
      this->entityCompMgr->CreateComponent(collisionEntity,
                                           components::Pose(collision->Pose()));
      this->entityCompMgr->CreateComponent(collisionEntity,
                                           components::Name(collision->Name()));
      this->entityCompMgr->CreateComponent(
          collisionEntity, components::ParentEntity(linkEntity));

      if (collision->Geom())
      {
        this->entityCompMgr->CreateComponent(
            collisionEntity, components::Geometry(*collision->Geom()));
      }
    }

    // Lights
    for (uint64_t lightIndex = 0; lightIndex < link->LightCount(); ++lightIndex)
    {
      auto light = link->LightByIndex(lightIndex);

      // Entity
      EntityId lightEntity = this->entityCompMgr->CreateEntity();

      // Components
      this->entityCompMgr->CreateComponent(lightEntity,
                                           components::Light(*light));
      this->entityCompMgr->CreateComponent(lightEntity,
                                           components::Pose(light->Pose()));
      this->entityCompMgr->CreateComponent(lightEntity,
                                           components::Name(light->Name()));
      this->entityCompMgr->CreateComponent(
          lightEntity, components::ParentEntity(linkEntity));
    }
  }

  // Joints
  for (uint64_t jointIndex = 0; jointIndex < _model.JointCount(); ++jointIndex)
  {
    auto joint = _model.JointByIndex(jointIndex);

    // Entity
    EntityId jointEntity = this->entityCompMgr->CreateEntity();

    // Components
    this->entityCompMgr->CreateComponent(jointEntity, components::Joint());
    this->entityCompMgr->CreateComponent(jointEntity,
                                         components::JointType(joint->Type()));

    if (joint->Axis(0))
    {
      this->entityCompMgr->CreateComponent(
          jointEntity, components::JointAxis(*joint->Axis(0)));
    }

    if (joint->Axis(1))
    {
      this->entityCompMgr->CreateComponent(
          jointEntity, components::JointAxis2(*joint->Axis(1)));
    }

    this->entityCompMgr->CreateComponent(jointEntity,
                                         components::Pose(joint->Pose()));
    this->entityCompMgr->CreateComponent(jointEntity,
                                         components::Name(joint->Name()));
    this->entityCompMgr->CreateComponent(
        jointEntity, components::ThreadPitch(joint->ThreadPitch()));
    this->entityCompMgr->CreateComponent(jointEntity,
                                         components::ParentEntity(modelEntity));
    this->entityCompMgr->CreateComponent(
        jointEntity, components::ParentLinkName(joint->ParentLinkName()));
    this->entityCompMgr->CreateComponent(
        jointEntity, components::ChildLinkName(joint->ChildLinkName()));
  }

  return modelEntity;
}

/////////////////////////////////////////////////
EntityId LevelManager::LoadLight(const sdf::Light &_light,
                                 const EntityId _worldEntity)
{
  // Entity
  EntityId lightEntity = this->entityCompMgr->CreateEntity();

  // Components
  this->entityCompMgr->CreateComponent(lightEntity, components::Light(_light));
  this->entityCompMgr->CreateComponent(lightEntity,
                                       components::Pose(_light.Pose()));
  this->entityCompMgr->CreateComponent(lightEntity,
                                       components::Name(_light.Name()));
  this->entityCompMgr->CreateComponent(lightEntity,
                                       components::ParentEntity(_worldEntity));
  return lightEntity;
}
