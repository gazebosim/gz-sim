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

#include <sdf/Actor.hh>
#include <sdf/Atmosphere.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/Atmosphere.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerLevels.hh"
#include "ignition/gazebo/components/PhysicsEnginePlugin.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/components/World.hh"

#include "LevelManager.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner, const bool _useLevels)
    : runner(_runner), useLevels(_useLevels)
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

  std::string service = "/world/";
  service += this->runner->sdfWorld->Name() + "/level/set_performer";
  this->node.Advertise(service, &LevelManager::OnSetPerformer, this);
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

  this->runner->entityCompMgr.CreateComponent(this->worldEntity,
      components::PhysicsEnginePlugin(
      this->runner->serverConfig.PhysicsEngine()));

  auto worldElem = this->runner->sdfWorld->Element();

  // Create Wind
  auto windEntity = this->runner->entityCompMgr.CreateEntity();
  this->runner->entityCompMgr.CreateComponent(windEntity, components::Wind());
  this->runner->entityCompMgr.CreateComponent(
      windEntity, components::WorldLinearVelocity(
                      this->runner->sdfWorld->WindLinearVelocity()));
  // Initially the wind linear velocity is used as the seed velocity
  this->runner->entityCompMgr.CreateComponent(
      windEntity, components::WorldLinearVelocitySeed(
                      this->runner->sdfWorld->WindLinearVelocity()));

  this->entityCreator->SetParent(windEntity, this->worldEntity);

  // scene
  if (this->runner->sdfWorld->Scene())
  {
    this->runner->entityCompMgr.CreateComponent(this->worldEntity,
        components::Scene(*this->runner->sdfWorld->Scene()));
  }

  // atmosphere
  if (this->runner->sdfWorld->Atmosphere())
  {
    this->runner->entityCompMgr.CreateComponent(this->worldEntity,
        components::Atmosphere(*this->runner->sdfWorld->Atmosphere()));
  }

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

  if (pluginElem == nullptr)
  {
    if (this->useLevels)
    {
      ignerr << "Could not find a plugin tag with name " << kPluginName
             << ". Levels and distributed simulation will not work.\n";
    }
  }
  else
  {
    this->ReadPerformers(pluginElem);
    if (this->useLevels)
      this->ReadLevels(pluginElem);
  }

  this->ConfigureDefaultLevel();

  // Load world plugins.
  this->runner->EventMgr().Emit<events::LoadPlugins>(this->worldEntity,
      this->runner->sdfWorld->Element());

  // Store the world's SDF DOM to be used when saving the world to file
  this->runner->entityCompMgr.CreateComponent(
      worldEntity, components::WorldSdf(*this->runner->sdfWorld));
}

/////////////////////////////////////////////////
void LevelManager::ReadPerformers(const sdf::ElementPtr &_sdf)
{
  IGN_PROFILE("LevelManager::ReadPerformers");

  if (_sdf == nullptr)
    return;

  if (_sdf->HasElement("performer"))
  {
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
        auto performer2 =
          this->runner->entityCompMgr.Component<components::Name>(
              this->performerMap[ref]);

        ignerr << "Found multiple performers (" << name << " and "
          << performer2->Data() << ") referring to the same entity\n";
      }

      sdf::Geometry geometry;
      geometry.Load(performer->GetElement("geometry"));
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Performer());
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::PerformerLevels());
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Name(name));
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Geometry(geometry));

      ignmsg << "Created performer [" << performerEntity << " / " << name << "]"
             << std::endl;
    }
  }

  if (this->useLevels && performerMap.empty())
  {
    igndbg << "Levels enabled, but no <performer>s were speficied in SDF. Use "
      << "the /world/<world_name>/level/set_performer service to specify "
      << "performers.\n";
  }
}

/////////////////////////////////////////////////
bool LevelManager::OnSetPerformer(const msgs::StringMsg &_req,
                                  msgs::Boolean &_rep)
{
  // \todo(nkoenig) This implementation store the request to be processed in
  // the update cycle. This approach is thread-safe, but is unable to
  // provide the caller with feedback because
  // entityCompMgr.EntityByComponents() is not thread safe. It would better
  // to have long running service calls in ign-transport so that this
  // function could get information out of the EntityComponent mangager
  // in a thread-safe manner and return information back to the caller.
  //
  // The commented out section at the end of this function was
  // the original implementation.

  std::string name = _req.data();
  _rep.set_data(false);
  if (!name.empty())
  {
    sdf::Geometry geom;
    geom.SetType(sdf::GeometryType::BOX);
    sdf::Box boxShape;

    // \todo(anyone) Use the bounding box instead of a hardcoded box.
    boxShape.SetSize({2, 2, 2});

    geom.SetBoxShape(boxShape);
    _rep.set_data(true);

    std::lock_guard<std::mutex> lock(this->performerToAddMutex);
    this->performersToAdd.push_back(std::make_pair(name, geom));
  }
  else
  {
    ignerr << "Empty performer name. Performer will not be created\n";
  }

  return true;

  // Orignial implementation
  //
  // _rep.set_data(false);
  // std::string name = _req.data();

  // // Find the model entity
  // Entity modelEntity = this->runner->entityCompMgr.EntityByComponents(
  //     components::Name(name));
  // if (modelEntity == kNullEntity)
  // {
  //   ignerr << "Unable to find model with name[" << name << "]. "
  //     << "Performer not created\n";
  //   return true;
  // }

  // // Check to see if the performer has already been set.
  // if (this->performerMap.find(name) == this->performerMap.end())
  // {
  //   sdf::Geometry geom;
  //   geom.SetType(sdf::GeometryType::BOX);
  //   sdf::Box boxShape;

  //   // \todo(anyone) Use the bounding box instead of a hardcoded box.
  //   boxShape.SetSize({2, 2, 2});

  //   geom.SetBoxShape(boxShape);
  //   this->performersToAdd.push_back(std::make_pair(name, geom));
  //   _rep.set_data(true);
  // }
  // else
  // {
  //   ignwarn << "Performer with name[" << name << "] "
  //     << "has already been set.\n";
  // }

  // // The response succeeded.
  // return true;
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

  // Actors
  for (uint64_t actorIndex = 0;
       actorIndex < this->runner->sdfWorld->ActorCount(); ++actorIndex)
  {
    // There is no sdf::World::ActorByName so we have to iterate by index and
    // check if the actor is in this level
    auto actor = this->runner->sdfWorld->ActorByIndex(actorIndex);
    // If actor is a performer, it will be handled separately
    if (this->performerMap.find(actor->Name()) != this->performerMap.end())
    {
      continue;
    }

    if (this->entityNamesInLevels.find(actor->Name()) ==
        this->entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(actor->Name());
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

  // Actors
  for (uint64_t actorIndex = 0;
       actorIndex < this->runner->sdfWorld->ActorCount(); ++actorIndex)
  {
    auto actor = this->runner->sdfWorld->ActorByIndex(actorIndex);
    if (this->performerMap.find(actor->Name()) != this->performerMap.end())
    {
      Entity actorEntity = this->entityCreator->CreateEntities(actor);

      // Make the actor a parent of this performer
      this->entityCreator->SetParent(this->performerMap[actor->Name()],
                                     actorEntity);

      // Add parent world to the actor
      this->entityCreator->SetParent(actorEntity, this->worldEntity);
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
    std::lock_guard<std::mutex> lock(this->performerToAddMutex);
    auto iter = this->performersToAdd.begin();
    while (iter != this->performersToAdd.end())
    {
      int result = this->CreatePerformerEntity(iter->first, iter->second);
      // Create the performer entity
      if (result >= 0)
        iter = this->performersToAdd.erase(iter);
      else
        ++iter;
    }
  }

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

  // If levels are not being used, we only process the default level.
  if (this->useLevels)
  {
    this->runner->entityCompMgr.Each<
      components::Performer,
      components::PerformerLevels,
      components::Geometry,
      components::ParentEntity>(
          [&](const Entity &_perfEntity,
            components::Performer *,
            components::PerformerLevels *_perfLevels,
            components::Geometry *_geometry,
            components::ParentEntity *_parent) -> bool
          {
          IGN_PROFILE("EachPerformer");

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

          std::set<Entity> newPerfLevels;

          // loop through levels and check for intersections
          // Add all levels with intersections to the levelsToLoad even if they
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
                if (nullptr == box)
                {
                ignerr << "Level [" << _entity
                << "]'s geometry is not a box." << std::endl;
                return true;
                }
                auto buffer = _levelBuffer->Data();
                auto center = _pose->Data().Pos();
                math::AxisAlignedBox region{center - box->Size() / 2,
                center + box->Size() / 2};

                math::AxisAlignedBox outerRegion{
                  center - (box->Size() / 2 + buffer),
                         center + (box->Size() / 2 + buffer)};

                if (region.Intersects(performerVolume))
                {
                  newPerfLevels.insert(_entity);
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
                      newPerfLevels.insert(_entity);
                      levelsToLoad.push_back(_entity);
                      return true;
                    }
                    // Otherwise, mark the level to be unloaded
                    levelsToUnload.push_back(_entity);
                  }
                }
                return true;
                });

          *_perfLevels = components::PerformerLevels(newPerfLevels);

          return true;
          });
  }

  // Sort levelsToLoad and levelsToUnload so as to run std::unique on them.
  std::sort(levelsToLoad.begin(), levelsToLoad.end());
  std::sort(levelsToUnload.begin(), levelsToUnload.end());
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
    const components::LevelEntityNames *lvlEntNames =
      this->runner->entityCompMgr.Component<components::LevelEntityNames>(
          toLoad);
    const auto &entityNames = lvlEntNames->Data();
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
      ignmsg << "Loaded level [" << level << "]" << std::endl;
      this->activeLevels.push_back(level);
    }
  }

  auto pendingEnd = this->activeLevels.end();
  for (const auto &toUnload : levelsToUnload)
  {
    ignmsg << "Unloaded level [" << toUnload << "]" << std::endl;
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

  // Actors
  for (uint64_t actorIndex = 0;
       actorIndex < this->runner->sdfWorld->ActorCount(); ++actorIndex)
  {
    // There is no sdf::World::ActorByName so we have to iterate by index and
    // check if the actor is in this level
    auto actor = this->runner->sdfWorld->ActorByIndex(actorIndex);
    if (_namesToLoad.find(actor->Name()) != _namesToLoad.end())
    {
      Entity actorEntity = this->entityCreator->CreateEntities(actor);

      this->entityCreator->SetParent(actorEntity, this->worldEntity);
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

  this->runner->entityCompMgr.Each<components::Actor, components::Name>(
      [&](const Entity &_entity, const components::Actor *,
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

/////////////////////////////////////////////////
int LevelManager::CreatePerformerEntity(const std::string &_name,
    const sdf::Geometry &_geom)
{
  // Find the model entity
  Entity modelEntity = this->runner->entityCompMgr.EntityByComponents(
      components::Name(_name));
  if (modelEntity == kNullEntity)
  {
    ignwarn << "Attempting to set performer with name ["
      << _name << "] "
      << ", but the entity could not be found. Another attempt will be made "
      << "in the next iteration.\n";
    return -1;
  }

  if (!this->runner->entityCompMgr.ChildrenByComponents(modelEntity,
        components::Performer()).empty())
  {
    ignwarn << "Attempting to set performer with name ["
      << _name << "], but the entity already has a performer.\n";
    return 1;
  }

  Entity performerEntity = this->runner->entityCompMgr.CreateEntity();
  this->performerMap[_name] = performerEntity;

  this->runner->entityCompMgr.CreateComponent(performerEntity,
      components::Performer());
  this->runner->entityCompMgr.CreateComponent(performerEntity,
      components::PerformerLevels());
  this->runner->entityCompMgr.CreateComponent(performerEntity,
      components::Name("perf_" + _name));
  this->runner->entityCompMgr.CreateComponent(performerEntity,
      components::Geometry(_geom));

  // Make the model a parent of this performer
  this->entityCreator->SetParent(this->performerMap[_name], modelEntity);
  return 0;
}
