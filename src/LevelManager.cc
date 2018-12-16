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
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;


/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner)
    : runner(_runner)
{
  // Do nothing
}

/////////////////////////////////////////////////
void LevelManager::ReadLevelPerformerInfo()
{
  this->worldEntity = this->runner->entityCompMgr.CreateEntity();

  // World components
  this->runner->entityCompMgr.CreateComponent(worldEntity,
                                               components::World());
  this->runner->entityCompMgr.CreateComponent(
      worldEntity, components::Name(this->runner->sdfWorld->Name()));

  auto worldElem = this->runner->sdfWorld->Element();

  for (auto performer = worldElem->GetElement("performer"); performer;
       performer = performer->GetNextElement("performer"))
  {
    std::string name = performer->Get<std::string>("name");

    EntityId performerEntity = this->runner->entityCompMgr.CreateEntity();
    // We use the ref to create a parent entity component later on
    std::string ref = performer->GetElement("ref")->GetValue()->GetAsString();
    if (this->performerMap.find(ref) == this->performerMap.end())
    {
      this->performerMap[ref] = performerEntity;
    }
    else
    {
      auto performer2 = this->runner->entityCompMgr.Component<components::Name>(
          this->performerMap[ref]);

      ignerr << "Found multiple performers (" << name << " and "
             << performer2->Data() << ")referring to the same entity\n";
    }

    sdf::Geometry geometry;
    geometry.Load(performer->GetElement("geometry"));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Performer());
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Name(name));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
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
    EntityId levelEntity = this->runner->entityCompMgr.CreateEntity();

    // Components
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::Level());
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::Pose(pose));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::Name(name));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::ParentEntity(worldEntity));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::NameSet(entityNames));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::Geometry(geometry));

    // Visualization
    // The levelLinkEntity and levelVisualEntities are only used for visualizing
    // the level for debugging. They may be removed if this is no longer needed.
    // TODO(addisu): This won't work with physics
    const bool visualizeLevels = false;
    if (visualizeLevels)
    {
      EntityId levelLinkEntity = this->runner->entityCompMgr.CreateEntity();
      this->runner->entityCompMgr.CreateComponent(
          levelLinkEntity, components::Link());
      this->runner->entityCompMgr.CreateComponent(
          levelLinkEntity, components::Pose(math::Pose3d()));
      this->runner->entityCompMgr.CreateComponent(
          levelLinkEntity, components::Name(name + "::link"));
      this->runner->entityCompMgr.CreateComponent(
          levelLinkEntity, components::ParentEntity(levelEntity));

      EntityId levelVisualEntity = this->runner->entityCompMgr.CreateEntity();
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::Visual());
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::ParentEntity(levelLinkEntity));
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::Pose(math::Pose3d()));
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::Name(name + "::visual"));
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::Geometry(geometry));
      sdf::Material mat;
      mat.SetAmbient({1, 1, 1, 0.1});
      this->runner->entityCompMgr.CreateComponent(
          levelVisualEntity, components::Material(mat));
    }
  }

  // Create the default level. This level contains all entities not contained by
  // any other level.
  EntityId defaultLevel = this->runner->entityCompMgr.CreateEntity();
  std::unordered_set<std::string> entityNamesInDefault;

  // Go through all entities in the world and find ones not in the
  // set entityNamesInLevels

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    // If model is a performer, it will be handled separately
    if (this->performerMap.find(model->Name()) != this->performerMap.end())
      continue;

    if (entityNamesInLevels.find(model->Name()) == entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(model->Name());
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (entityNamesInLevels.find(light->Name()) == entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(light->Name());
    }
  }
  // Components
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Level());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::ParentEntity(this->worldEntity));
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::NameSet(entityNamesInDefault));


  // Load world plugins.
  // \TODO (addisu) Find a better place to load plugins instead of this function
  this->LoadPlugins(this->runner->sdfWorld->Element(), this->worldEntity);
  // Add default level to levels to load
  this->levelsToLoad.insert(defaultLevel);
}

/////////////////////////////////////////////////
void LevelManager::CreatePerformers()
{
  if (this->worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while creating performers\n";
    return;
  }
  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    if (this->performerMap.find(model->Name()) != this->performerMap.end() )
    {
      EntityId modelEntity = this->CreateEntities(model);
      // Create a component on the performer entity that points to this model
      this->runner->entityCompMgr.CreateComponent(
          this->performerMap[model->Name()],
          components::ParentEntity(modelEntity));

      // Add parent world to the model
      this->runner->entityCompMgr.CreateComponent(
          modelEntity,
          components::ParentEntity(this->worldEntity));
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::UpdateLevels()
{
  this->runner->entityCompMgr.Each<components::Performer, components::Geometry,
                                   components::ParentEntity>(
      [&](const EntityId &, const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity *_parent) -> bool
      {
        auto pose = this->runner->entityCompMgr.Component<components::Pose>(
            _parent->Data());
        // We assume the geometry contains a box.
        auto perfBox = _geometry->Data().BoxShape();

        math::AxisAlignedBox performerVolume{
             pose->Data().Pos() - perfBox->Size() / 2,
             pose->Data().Pos() + perfBox->Size() / 2};


        // loop through levels and check for intersections
        this->runner->entityCompMgr.Each<components::Level, components::Pose,
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
  if (this->worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while loading levels\n";
    return;
  }

  // Create a union of all the entity name sets
  std::unordered_set<std::string> entityNamesUnion;
  for (auto level : this->levelsToLoad)
  {
    auto names =
        this->runner->entityCompMgr.Component<components::NameSet>(level);
    if (names != nullptr)
    {
      entityNamesUnion.insert(names->Data().begin(), names->Data().end());
      this->activeLevels.insert(level);
    }
  }

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    if (entityNamesUnion.find(model->Name()) != entityNamesUnion.end())
    {
      igndbg << "Creating model: " << model->Name() << "\n";
      EntityId modelEntity = this->CreateEntities(model);
      this->runner->entityCompMgr.CreateComponent(
          modelEntity, components::ParentEntity(this->worldEntity));
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (entityNamesUnion.find(light->Name()) != entityNamesUnion.end())
    {
      EntityId lightEntity = this->CreateEntities(light);
      this->runner->entityCompMgr.CreateComponent(
          lightEntity, components::ParentEntity(this->worldEntity));
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

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Model *_model)
{
  // Entity
  EntityId modelEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(modelEntity, components::Model());
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Pose(_model->Pose()));
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Name(_model->Name()));
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Static(_model->Static()));
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::RenderState(true));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->runner->entityCompMgr.CreateComponent(linkEntity,
        components::ParentEntity(modelEntity));
    if (linkIndex == 0)
    {
      this->runner->entityCompMgr.CreateComponent(linkEntity,
          components::CanonicalLink());
    }
  }

  // Joints
  for (uint64_t jointIndex = 0; jointIndex < _model->JointCount();
      ++jointIndex)
  {
    auto joint = _model->JointByIndex(jointIndex);
    auto linkEntity = this->CreateEntities(joint);

    this->runner->entityCompMgr.CreateComponent(linkEntity,
        components::ParentEntity(modelEntity));
  }

  // Model plugins
  this->LoadPlugins(_model->Element(), modelEntity);

  return modelEntity;
}

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Light *_light)
{
  // Entity
  EntityId lightEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(
      lightEntity, components::Light(*_light));
  this->runner->entityCompMgr.CreateComponent(lightEntity,
      components::Pose(_light->Pose()));
  this->runner->entityCompMgr.CreateComponent(lightEntity,
      components::Name(_light->Name()));

  return lightEntity;
}

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Link *_link)
{
  // Entity
  EntityId linkEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(linkEntity, components::Link());
  this->runner->entityCompMgr.CreateComponent(linkEntity,
      components::Pose(_link->Pose()));
  this->runner->entityCompMgr.CreateComponent(linkEntity,
      components::Name(_link->Name()));
  this->runner->entityCompMgr.CreateComponent(linkEntity,
      components::Inertial(_link->Inertial()));

  // Visuals
  for (uint64_t visualIndex = 0; visualIndex < _link->VisualCount();
      ++visualIndex)
  {
    auto visual = _link->VisualByIndex(visualIndex);
    auto visualEntity = this->CreateEntities(visual);

    this->runner->entityCompMgr.CreateComponent(visualEntity,
        components::ParentEntity(linkEntity));
  }

  // Collisions
  for (uint64_t collisionIndex = 0; collisionIndex < _link->CollisionCount();
      ++collisionIndex)
  {
    auto collision = _link->CollisionByIndex(collisionIndex);
    auto collisionEntity = this->CreateEntities(collision);

    this->runner->entityCompMgr.CreateComponent(collisionEntity,
        components::ParentEntity(linkEntity));
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _link->LightCount();
      ++lightIndex)
  {
    auto light = _link->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->runner->entityCompMgr.CreateComponent(lightEntity,
        components::ParentEntity(linkEntity));
  }

  return linkEntity;
}

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Joint *_joint)
{
  // Entity
  EntityId jointEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(jointEntity,
      components::Joint());
  this->runner->entityCompMgr.CreateComponent(jointEntity,
      components::JointType(_joint->Type()));

  if (_joint->Axis(0))
  {
    this->runner->entityCompMgr.CreateComponent(jointEntity,
        components::JointAxis(*_joint->Axis(0)));
  }

  if (_joint->Axis(1))
  {
    this->runner->entityCompMgr.CreateComponent(jointEntity,
        components::JointAxis2(*_joint->Axis(1)));
  }

  this->runner->entityCompMgr.CreateComponent(jointEntity,
      components::Pose(_joint->Pose()));
  this->runner->entityCompMgr.CreateComponent(jointEntity ,
      components::Name(_joint->Name()));
  this->runner->entityCompMgr.CreateComponent(jointEntity ,
      components::ThreadPitch(_joint->ThreadPitch()));
  this->runner->entityCompMgr.CreateComponent(jointEntity,
      components::ParentLinkName(_joint->ParentLinkName()));
  this->runner->entityCompMgr.CreateComponent(jointEntity,
      components::ChildLinkName(_joint->ChildLinkName()));

  return jointEntity;
}

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Visual *_visual)
{
  // Entity
  EntityId visualEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(
      visualEntity, components::Visual());
  this->runner->entityCompMgr.CreateComponent(visualEntity,
      components::Pose(_visual->Pose()));
  this->runner->entityCompMgr.CreateComponent(visualEntity,
      components::Name(_visual->Name()));

  if (_visual->Geom())
  {
    this->runner->entityCompMgr.CreateComponent(visualEntity,
        components::Geometry(*_visual->Geom()));
  }

  // \todo(louise) Populate with default material if undefined
  if (_visual->Material())
  {
    this->runner->entityCompMgr.CreateComponent(visualEntity,
        components::Material(*_visual->Material()));
  }

  return visualEntity;
}

//////////////////////////////////////////////////
EntityId LevelManager::CreateEntities(const sdf::Collision *_collision)
{
  // Entity
  EntityId collisionEntity = this->runner->entityCompMgr.CreateEntity();

  // Components
  this->runner->entityCompMgr.CreateComponent(collisionEntity,
      components::Collision());
  this->runner->entityCompMgr.CreateComponent(collisionEntity,
      components::Pose(_collision->Pose()));
  this->runner->entityCompMgr.CreateComponent(collisionEntity,
      components::Name(_collision->Name()));

  if (_collision->Geom())
  {
    this->runner->entityCompMgr.CreateComponent(collisionEntity,
        components::Geometry(*_collision->Geom()));
  }

  return collisionEntity;
}

//////////////////////////////////////////////////
void LevelManager::LoadPlugins(const sdf::ElementPtr &_sdf,
    const EntityId _id)
{
  if (!_sdf->HasElement("plugin"))
    return;

  sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
  while (pluginElem)
  {
    auto system = this->runner->systemLoader->LoadPlugin(pluginElem);
    if (system)
    {
      auto systemConfig = system.value()->QueryInterface<ISystemConfigure>();
      if (systemConfig != nullptr)
      {
        systemConfig->Configure(_id, pluginElem,
                                this->runner->entityCompMgr,
                                this->runner->eventMgr);
      }
      this->runner->AddSystem(system.value());
    }
    pluginElem = pluginElem->GetNextElement("plugin");
  }
}

