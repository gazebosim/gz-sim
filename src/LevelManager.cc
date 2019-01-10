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
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "LevelManager.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner, const bool _useLevels)
    : runner(_runner), useLevels(_useLevels)
{
  // Do nothing
}

/////////////////////////////////////////////////
void LevelManager::Configure()
{
  this->ReadLevelPerformerInfo();
  this->CreatePerformers();
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

  // TODO(anyone) This should probaly go somwhere else as it is a global
  // constant.
  const std::string kPluginName{"ignition::gazebo"};

  sdf::ElementPtr pluginElem;
  // Get the ignition::gazebo plugin element
  for (auto plugin = worldElem->GetElement("plugin"); plugin;
       plugin = plugin->GetNextElement("plugin"))
  {
    if (plugin->Get<std::string>("name") == kPluginName)
    {
      pluginElem = plugin;
    }
  }

  if (this->useLevels)
  {
    if (pluginElem == nullptr)
    {
      ignerr << "Could not find a plugin tag with name " << kPluginName << "\n";
      return;
    }
    this->ReadPerformers(pluginElem);
    this->ReadLevels(pluginElem);
  }

  this->ConfigureDefaultLevel();

  // Load world plugins.
  // \TODO (addisu) Find a better place to load plugins instead of this function
  this->LoadPlugins(this->runner->sdfWorld->Element(), this->worldEntity);
}


/////////////////////////////////////////////////
void LevelManager::ReadPerformers(const sdf::ElementPtr &_sdf)
{
  if (_sdf == nullptr)
    return;

  igndbg << "Reading performer info\n";
  for (auto performer = _sdf->GetElement("performer"); performer;
       performer = performer->GetNextElement("performer"))
  {
    auto name = performer->Get<std::string>("name");

    Entity performerEntity = this->runner->entityCompMgr.CreateEntity();
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
}

/////////////////////////////////////////////////
void LevelManager::ReadLevels(const sdf::ElementPtr &_sdf)
{
  igndbg << "Reading levels info\n";

  if (_sdf != nullptr)
  {
    for (auto level = _sdf->GetElement("level"); level;
         level = level->GetNextElement("level"))
    {
      auto name = level->Get<std::string>("name");
      auto pose = level->Get<math::Pose3d>("pose");
      sdf::Geometry geometry;
      geometry.Load(level->GetElement("geometry"));
      std::set<std::string> entityNames;

      for (auto ref = level->GetElement("ref"); ref;
           ref = ref->GetNextElement("ref"))
      {
        std::string entityName = ref->GetValue()->GetAsString();
        // TODO(addisu) Make sure the names are unique
        entityNames.insert(entityName);

        this->entityNamesInLevels.insert(entityName);
      }

      // Entity
      Entity levelEntity = this->runner->entityCompMgr.CreateEntity();

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
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::ConfigureDefaultLevel()
{
  // Create the default level. This level contains all entities not contained by
  // any other level.
  Entity defaultLevel = this->runner->entityCompMgr.CreateEntity();

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
    {
      continue;
    }

    if (this->entityNamesInLevels.find(model->Name()) ==
        this->entityNamesInLevels.end())
    {
      this->entityNamesInDefault.insert(model->Name());
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (this->entityNamesInLevels.find(light->Name()) ==
        this->entityNamesInLevels.end())
    {
      this->entityNamesInDefault.insert(light->Name());
    }
  }
  // Components
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Level());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::ParentEntity(this->worldEntity));
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::NameSet(this->entityNamesInDefault));

  // Add default level to levels to load
  this->entityNamesToLoad.insert(this->entityNamesInDefault.begin(),
                                 this->entityNamesInDefault.end());
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
      Entity modelEntity = this->CreateEntities(model);
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
void LevelManager::UpdateLevelsState()
{
  this->runner->entityCompMgr.Each<components::Performer, components::Geometry,
                                   components::ParentEntity>(
      [&](const Entity &, const components::Performer *,
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
                                         components::Geometry,
                                         components::NameSet>(
            [&](const Entity &, const components::Level *,
                const components::Pose *_pose,
                const components::Geometry *_levelGeometry,
                const components::NameSet *_nameSet) -> bool
            {
              // Check if the performer is in this level
              // assume a box for now
              auto box = _levelGeometry->Data().BoxShape();
              auto center = _pose->Data().Pos();
              math::AxisAlignedBox region{center - box->Size() / 2,
                                          center + box->Size() / 2};

              if (region.Intersects(performerVolume))
              {
                this->entityNamesToLoad.insert(_nameSet->Data().begin(),
                                               _nameSet->Data().end());
              }
              else
              {
                // mark the entity to be unloaded if it's currently active
                std::copy_if(_nameSet->Data().begin(), _nameSet->Data().end(),
                             std::inserter(this->entityNamesToUnload,
                                           this->entityNamesToUnload.begin()),
                             [this](const std::string &_name) -> bool
                             {
                               return this->activeEntityNames.find(_name) !=
                                      this->activeEntityNames.end();
                             });
              }
              return true;
            });

        return true;
      });

  // Filter the entityNamesToUnload so that if a level is marked to be loaded by
  // one performer check and marked to be unloaded by another, we don't end up
  // unloading it
  std::set<std::string> tmpToUnload;
  std::set_difference(
      this->entityNamesToUnload.begin(), this->entityNamesToUnload.end(),
      this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
      std::inserter(tmpToUnload, tmpToUnload.begin()));
  this->entityNamesToUnload = std::move(tmpToUnload);

  // Filter out the entities that are already active
  std::set<std::string> tmpToLoad;
  std::set_difference(
      this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
      this->activeEntityNames.begin(), this->activeEntityNames.end(),
      std::inserter(tmpToLoad, tmpToLoad.begin()));
  this->entityNamesToLoad = std::move(tmpToLoad);

  // ---------------------- DEBUG ---------------------
  static std::size_t counter = 0;

  if (this->entityNamesToLoad.size() > 0)
  {
    std::stringstream ss;
    ss << counter << ": Levels to load:";
    std::copy(this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
              std::ostream_iterator<std::string>(ss, " "));
    igndbg << ss.str() << std::endl;
  }

  if (this->entityNamesToUnload.size() > 0)
  {
    std::stringstream ss;
    ss << counter << ": Levels to unload:";
    std::copy(this->entityNamesToUnload.begin(),
              this->entityNamesToUnload.end(),
              std::ostream_iterator<std::string>(ss, " "));
    igndbg << ss.str() << std::endl;
  }
  ++counter;
  // ---------------------- END DEBUG ---------------------
}

/////////////////////////////////////////////////
void LevelManager::LoadActiveLevels()
{
  if (this->worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while loading levels\n";
    return;
  }

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    if (this->entityNamesToLoad.find(model->Name()) !=
        this->entityNamesToLoad.end())
    {
      Entity modelEntity = this->CreateEntities(model);
      this->runner->entityCompMgr.CreateComponent(
          modelEntity, components::ParentEntity(this->worldEntity));
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (this->entityNamesToLoad.find(light->Name()) !=
        this->entityNamesToLoad.end())
    {
      Entity lightEntity = this->CreateEntities(light);
      this->runner->entityCompMgr.CreateComponent(
          lightEntity, components::ParentEntity(this->worldEntity));
    }
  }

  this->activeEntityNames.insert(this->entityNamesToLoad.begin(),
                                 this->entityNamesToLoad.end());
  this->entityNamesToLoad.clear();
}

/////////////////////////////////////////////////
void LevelManager::UnloadInactiveLevels()
{
  this->runner->entityCompMgr.Each<components::Model, components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_name) -> bool
      {
        if (this->entityNamesToUnload.find(_name->Data()) !=
            this->entityNamesToUnload.end())
        {
          EraseEntityRecursively(_entity);
        }
        return true;
      });
  for (const auto &_name : this->entityNamesToUnload)
  {
    this->activeEntityNames.erase(_name);
  }
  this->entityNamesToUnload.clear();
}

//////////////////////////////////////////////////
Entity LevelManager::CreateEntities(const sdf::Model *_model)
{
  // Entity
  Entity modelEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_model->Name(), modelEntity, modelEntity);

  // Components
  this->runner->entityCompMgr.CreateComponent(modelEntity, components::Model());
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Pose(_model->Pose()));
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Name(_model->Name()));
  this->runner->entityCompMgr.CreateComponent(
      modelEntity, components::Static(_model->Static()));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);
    // Assume the link entity node is created in CreateEntities(sdf::Link)
    this->entityGraph.AddEdge({modelEntity, linkEntity}, true);

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
    auto jointEntity = this->CreateEntities(joint);
    this->entityGraph.AddEdge({modelEntity, jointEntity}, true);

    this->runner->entityCompMgr.CreateComponent(jointEntity,
        components::ParentEntity(modelEntity));
  }

  // Model plugins
  // TODO(addisu) Don't load plugins if they have been loaded already. Do we
  // need to unload plugins if they are attached to an entity that gets unloaded
  this->LoadPlugins(_model->Element(), modelEntity);

  return modelEntity;
}

//////////////////////////////////////////////////
Entity LevelManager::CreateEntities(const sdf::Light *_light)
{
  // Entity
  Entity lightEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_light->Name(), lightEntity, lightEntity);

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
Entity LevelManager::CreateEntities(const sdf::Link *_link)
{
  // Entity
  Entity linkEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_link->Name(), linkEntity, linkEntity);

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
    this->entityGraph.AddEdge({linkEntity, visualEntity}, true);

    this->runner->entityCompMgr.CreateComponent(visualEntity,
        components::ParentEntity(linkEntity));
  }

  // Collisions
  for (uint64_t collisionIndex = 0; collisionIndex < _link->CollisionCount();
      ++collisionIndex)
  {
    auto collision = _link->CollisionByIndex(collisionIndex);
    auto collisionEntity = this->CreateEntities(collision);
    this->entityGraph.AddEdge({linkEntity, collisionEntity}, true);

    this->runner->entityCompMgr.CreateComponent(collisionEntity,
        components::ParentEntity(linkEntity));
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _link->LightCount();
      ++lightIndex)
  {
    auto light = _link->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);
    this->entityGraph.AddEdge({linkEntity, lightEntity}, true);

    this->runner->entityCompMgr.CreateComponent(lightEntity,
        components::ParentEntity(linkEntity));
  }

  return linkEntity;
}

//////////////////////////////////////////////////
Entity LevelManager::CreateEntities(const sdf::Joint *_joint)
{
  // Entity
  Entity jointEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_joint->Name(), jointEntity, jointEntity);

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
Entity LevelManager::CreateEntities(const sdf::Visual *_visual)
{
  // Entity
  Entity visualEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_visual->Name(), visualEntity, visualEntity);

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
Entity LevelManager::CreateEntities(const sdf::Collision *_collision)
{
  // Entity
  Entity collisionEntity = this->runner->entityCompMgr.CreateEntity();

  this->entityGraph.AddVertex(_collision->Name(), collisionEntity,
                              collisionEntity);

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
    const Entity _entity)
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
        systemConfig->Configure(_entity, pluginElem,
                                this->runner->entityCompMgr,
                                this->runner->eventMgr);
      }
      this->runner->AddSystem(system.value());
    }
    pluginElem = pluginElem->GetNextElement("plugin");
  }
}

//////////////////////////////////////////////////
void LevelManager::EraseEntityRecursively(const Entity _entity)
{
  for (const auto &vertex : this->entityGraph.AdjacentsFrom(_entity))
  {
    this->EraseEntityRecursively(vertex.first);
  }

  // Remove from ECM
  this->runner->entityCompMgr.RequestEraseEntity(_entity);
  // Remove the vertex from the graph
  bool rc = this->entityGraph.RemoveVertex(_entity);
  if (!rc)
  {
    ignerr << "Removing node " << _entity << " failed\n";
  }
}
