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

#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

#include "network/components/PerformerActive.hh"
#include "LevelManager.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner, const bool _useLevels,
    const bool _useDistSim)
    : runner(_runner), useLevels(_useLevels), useDistSim(_useDistSim)
{
  if (nullptr == _runner)
  {
    ignerr << "Can't start level manager with null runner." << std::endl;
    return;
  }

  this->entityCreator = std::make_unique<SdfEntityCreator>(
      this->runner->entityCompMgr,
      this->runner->eventMgr);

  this->ReadLevelPerformerInfo();
  this->CreatePerformers();
}

/////////////////////////////////////////////////
void LevelManager::ReadLevelPerformerInfo()
{
  // \todo(anyone) Use SdfEntityCreator to avoid duplication
  this->worldEntity = this->runner->entityCompMgr.CreateEntity();

  // World components
  this->runner->entityCompMgr.CreateComponent(this->worldEntity,
                                               components::World());
  this->runner->entityCompMgr.CreateComponent(
      this->worldEntity, components::Name(this->runner->sdfWorld->Name()));

  this->runner->entityCompMgr.CreateComponent(this->worldEntity,
      components::Gravity(this->runner->sdfWorld->Gravity()));

  this->runner->entityCompMgr.CreateComponent(this->worldEntity,
      components::MagneticField(this->runner->sdfWorld->MagneticField()));

  auto worldElem = this->runner->sdfWorld->Element();

  // TODO(anyone) This should probably go somewhere else as it is a global
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
      break;
    }
  }

  if (this->useLevels || this->useDistSim)
  {
    if (pluginElem == nullptr)
    {
      ignerr << "Could not find a plugin tag with name " << kPluginName
             << ". Levels and distributed simulation will not work.\n";
    }
    else
    {
      this->ReadPerformers(pluginElem);
      if (this->useLevels)
        this->ReadLevels(pluginElem);
    }
  }

  this->ConfigureDefaultLevel();

  // Load world plugins.
  this->runner->EventMgr().Emit<events::LoadPlugins>(this->worldEntity,
      this->runner->sdfWorld->Element());
}

/////////////////////////////////////////////////
void LevelManager::ReadPerformers(const sdf::ElementPtr &_sdf)
{
  IGN_PROFILE("LevelManager::ReadPerformers");

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
             << performer2->Data() << ") referring to the same entity\n";
    }

    sdf::Geometry geometry;
    geometry.Load(performer->GetElement("geometry"));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Performer());
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::PerformerActive(true));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Name(name));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Geometry(geometry));
  }

  if (this->useLevels && performerMap.empty())
  {
    ignwarn << "Asked to use levels but no <performer>s were found - levels "
               "will not work.\n";
  }
}

/////////////////////////////////////////////////
void LevelManager::ReadLevels(const sdf::ElementPtr &_sdf)
{
  IGN_PROFILE("LevelManager::ReadLevels");

  igndbg << "Reading levels info\n";

  if (_sdf == nullptr)
    return;

  for (auto level = _sdf->GetElement("level"); level;
       level = level->GetNextElement("level"))
  {
    auto name = level->Get<std::string>("name");
    auto pose = level->Get<math::Pose3d>("pose");
    sdf::Geometry geometry;
    geometry.Load(level->GetElement("geometry"));

    if (nullptr == geometry.BoxShape())
    {
      ignerr << "Level [" << name << "]'s geometry is not a box, level won't "
             << "be created." << std::endl;
      continue;
    }

    double buffer = level->Get<double>("buffer", 0.0).first;
    if (buffer < 0)
    {
      ignwarn << "The buffer parameter for Level [" << name << "]cannot be a "
              << " negative number. Setting to 0.0\n";
      buffer = 0.0;
    }

    std::set<std::string> entityNames;

    for (auto ref = level->GetElement("ref"); ref;
         ref = ref->GetNextElement("ref"))
    {
      std::string entityName = ref->GetValue()->GetAsString();
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
        levelEntity, components::LevelEntityNames(entityNames));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::Geometry(geometry));
    this->runner->entityCompMgr.CreateComponent(
        levelEntity, components::LevelBuffer(buffer));

    this->entityCreator->SetParent(levelEntity, this->worldEntity);
  }
}

/////////////////////////////////////////////////
void LevelManager::ConfigureDefaultLevel()
{
  IGN_PROFILE("LevelManager::ConfigureDefaultLevel");

  // Create the default level. This level contains all entities not contained by
  // any other level.
  Entity defaultLevel = this->runner->entityCompMgr.CreateEntity();

  // Go through all entities in the world and find ones not in the
  // set entityNamesInLevels

  std::set<std::string> entityNamesInDefault;

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
      entityNamesInDefault.insert(model->Name());
    }
  }

  // Lights
  // We assume no performers are lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (this->entityNamesInLevels.find(light->Name()) ==
        this->entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(light->Name());
    }
  }
  // Components
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Level());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::DefaultLevel());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::LevelEntityNames(entityNamesInDefault));

  this->entityCreator->SetParent(defaultLevel, this->worldEntity);
}

/////////////////////////////////////////////////
void LevelManager::CreatePerformers()
{
  IGN_PROFILE("LevelManager::CreatePerformers");

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
    if (this->performerMap.find(model->Name()) != this->performerMap.end())
    {
      Entity modelEntity = this->entityCreator->CreateEntities(model);

      // Make the model a parent of this performer
      this->entityCreator->SetParent(this->performerMap[model->Name()],
                                     modelEntity);

      // Add parent world to the model
      this->entityCreator->SetParent(modelEntity, this->worldEntity);
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::UpdateLevelsState()
{
  IGN_PROFILE("LevelManager::UpdateLevelsState");

  std::vector<Entity> levelsToLoad;
  std::vector<Entity> levelsToUnload;

  {
    IGN_PROFILE("DefaultLevel");
    // Handle default level
    this->runner->entityCompMgr.Each<components::DefaultLevel>(
        [&](const Entity &_entity, const components::DefaultLevel *) -> bool
        {
          if (!this->IsLevelActive(_entity))
          {
            levelsToLoad.push_back(_entity);
          }
          // We assume one default level
          return false;
        });
  }

  this->runner->entityCompMgr.Each<components::Performer,
                                   components::Geometry,
                                   components::ParentEntity,
                                   components::PerformerActive>(
      [&](const Entity &_perfEntity, const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity *_parent,
          const components::PerformerActive *_active) -> bool
      {
        IGN_PROFILE("EachPerformer");

        if (!_active->Data())
        {
          return true;
        }

        auto pose = this->runner->entityCompMgr.Component<components::Pose>(
            _parent->Data());

        // We assume the geometry contains a box.
        auto perfBox = _geometry->Data().BoxShape();
        if (nullptr == perfBox)
        {
          ignerr << "Internal error: geometry of performer [" << _perfEntity
                 << "] missing box." << std::endl;
          return true;
        }

        math::AxisAlignedBox performerVolume{
             pose->Data().Pos() - perfBox->Size() / 2,
             pose->Data().Pos() + perfBox->Size() / 2};

        // loop through levels and check for intersections
        // Add all levels with inersections to the levelsToLoad even if they
        // are currently active.
        this->runner->entityCompMgr.Each<components::Level, components::Pose,
                                         components::Geometry,
                                         components::LevelBuffer >(
            [&](const Entity &_entity, const components::Level *,
                const components::Pose *_pose,
                const components::Geometry *_levelGeometry,
                const components::LevelBuffer *_levelBuffer) -> bool
            {
                IGN_PROFILE("CheckPerformerAgainstLevel");
                // Check if the performer is in this level
                // assume a box for now
                auto box = _levelGeometry->Data().BoxShape();
                auto buffer = _levelBuffer->Data();
                auto center = _pose->Data().Pos();
                math::AxisAlignedBox region{center - box->Size() / 2,
                                            center + box->Size() / 2};

                math::AxisAlignedBox outerRegion{
                    center - (box->Size() / 2 + buffer),
                    center + (box->Size() / 2 + buffer)};

                if (region.Intersects(performerVolume))
                {
                  levelsToLoad.push_back(_entity);
                }
                else
                {
                  // If the level is active, check if the performer is
                  // outside of the buffer of this level
                  if (this->IsLevelActive(_entity))
                  {
                    if (outerRegion.Intersects(performerVolume))
                    {
                      levelsToLoad.push_back(_entity);
                      return true;
                    }
                    // Otherwise, mark the level to be unloaded
                    levelsToUnload.push_back(_entity);
                  }
                }
                return true;
              });
        return true;
      });
  {
    auto pendingEnd = std::unique(levelsToLoad.begin(), levelsToLoad.end());
    levelsToLoad.erase(pendingEnd, levelsToLoad.end());
  }
  {
    auto pendingEnd = std::unique(levelsToUnload.begin(), levelsToUnload.end());
    levelsToUnload.erase(pendingEnd, levelsToUnload.end());
  }

  // Make a list of entity names from all the levels that have been marked to be
  // loaded
  std::set<std::string> entityNamesMarked;
  for (const auto &toLoad : levelsToLoad)
  {
    auto entityNames = this->runner->entityCompMgr
                           .Component<components::LevelEntityNames>(toLoad)
                           ->Data();
    for (const auto &name : entityNames)
    {
      entityNamesMarked.insert(name);
    }
  }

  // Filter out currently active entities from the marked entities and create a
  // new set of entities. These entities will be the ones that are loaded
  std::set<std::string> entityNamesToLoad;
  for (const auto &name : entityNamesMarked)
  {
    if (this->activeEntityNames.find(name) == this->activeEntityNames.end())
    {
      entityNamesToLoad.insert(name);
    }
  }

  // First filter levelsToUnload so it doesn't contain any levels that are
  // already in levelsToLoad
  auto pendingRemove = std::remove_if(
      levelsToUnload.begin(), levelsToUnload.end(), [&](Entity _entity)
      {
        return std::find(levelsToLoad.begin(), levelsToLoad.end(), _entity) !=
               levelsToLoad.end();
      });
  levelsToUnload.erase(pendingRemove, levelsToUnload.end());

  // Make a list of entity names to unload making sure to leave out the ones
  // that have been marked to be loaded above
  std::set<std::string> entityNamesToUnload;
  for (const auto &toUnload : levelsToUnload)
  {
    auto entityNames = this->runner->entityCompMgr
                           .Component<components::LevelEntityNames>(toUnload)
                           ->Data();

    for (const auto &name : entityNames)
    {
      if (entityNamesMarked.find(name) == entityNamesMarked.end())
      {
        entityNamesToUnload.insert(name);
      }
    }
  }

  // Load and unload the entities
  if (entityNamesToLoad.size() > 0)
  {
    this->LoadActiveEntities(entityNamesToLoad);
  }
  if (entityNamesToUnload.size() > 0)
  {
    this->UnloadInactiveEntities(entityNamesToUnload);
  }

  // Finally, upadte the list of active levels
  for (const auto &level : levelsToLoad)
  {
    if (!this->IsLevelActive(level))
    {
      this->activeLevels.push_back(level);
    }
  }

  auto pendingEnd = this->activeLevels.end();
  for (const auto &toUnload : levelsToUnload)
  {
    pendingEnd = std::remove(this->activeLevels.begin(), pendingEnd, toUnload);
  }
  // Erase from vector
  this->activeLevels.erase(pendingEnd, this->activeLevels.end());
}

/////////////////////////////////////////////////
void LevelManager::LoadActiveEntities(const std::set<std::string> &_namesToLoad)
{
  IGN_PROFILE("LevelManager::LoadActiveEntities");

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
    if (_namesToLoad.find(model->Name()) != _namesToLoad.end())
    {
      Entity modelEntity = this->entityCreator->CreateEntities(model);

      this->entityCreator->SetParent(modelEntity, this->worldEntity);
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (_namesToLoad.find(light->Name()) != _namesToLoad.end())
    {
      Entity lightEntity = this->entityCreator->CreateEntities(light);

      this->entityCreator->SetParent(lightEntity, this->worldEntity);
    }
  }

  this->activeEntityNames.insert(_namesToLoad.begin(), _namesToLoad.end());
}

/////////////////////////////////////////////////
void LevelManager::UnloadInactiveEntities(
    const std::set<std::string> &_namesToUnload)
{
  this->runner->entityCompMgr.Each<components::Model, components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_name) -> bool
      {
        if (_namesToUnload.find(_name->Data()) != _namesToUnload.end())
        {
          this->entityCreator->RequestRemoveEntity(_entity, true);
        }
        return true;
      });

  this->runner->entityCompMgr.Each<components::Light, components::Name>(
      [&](const Entity &_entity, const components::Light *,
          const components::Name *_name) -> bool
      {
        if (_namesToUnload.find(_name->Data()) != _namesToUnload.end())
        {
          this->entityCreator->RequestRemoveEntity(_entity, true);
        }
        return true;
      });

  for (const auto &name : _namesToUnload)
  {
    this->activeEntityNames.erase(name);
  }
}

/////////////////////////////////////////////////
bool LevelManager::IsLevelActive(const Entity _entity) const
{
  return std::find(this->activeLevels.begin(), this->activeLevels.end(),
                   _entity) != this->activeLevels.end();
}
