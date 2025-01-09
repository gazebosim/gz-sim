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

#include "LevelManager.hh"

#include <algorithm>
#include <optional>

#include <sdf/Actor.hh>
#include <sdf/Atmosphere.hh>
#include <sdf/Element.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>
#include <sdf/World.hh>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

#include "gz/sim/Events.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Atmosphere.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Level.hh"
#include "gz/sim/components/LevelBuffer.hh"
#include "gz/sim/components/LevelEntityNames.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/PerformerLevels.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Scene.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/World.hh"

#include "SimulationRunner.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner, const bool _useLevels)
    : runner(_runner), useLevels(_useLevels)
{
  if (nullptr == _runner)
  {
    gzerr << "Can't start level manager with null runner." << std::endl;
    return;
  }

  this->entityCreator = std::make_unique<SdfEntityCreator>(
      this->runner->entityCompMgr,
      this->runner->eventMgr);

  std::string service = transport::TopicUtils::AsValidTopic("/world/" +
      this->runner->sdfWorld.Name() + "/level/set_performer");
  if (service.empty())
  {
    gzerr << "Failed to generate set_performer topic for world ["
           << this->runner->sdfWorld.Name() << "]" << std::endl;
    return;
  }
  this->node.Advertise(service, &LevelManager::OnSetPerformer, this);
}

/////////////////////////////////////////////////
void LevelManager::ReadLevelPerformerInfo(const sdf::World &_world)
{
  // TODO(anyone) This should probably go somewhere else as it is a global
  // constant.
  const std::string kPluginName{"gz::sim"};

  bool found = false;
  for (const sdf::Plugin &plugin : _world.Plugins())
  {
    if (plugin.Name() == kPluginName)
    {
      this->ReadPerformers(plugin);
      if (this->useLevels)
        this->ReadLevels(plugin);
      found = true;
      break;
    }
  }

  if (!found && this->useLevels)
  {
    gzerr << "Could not find a plugin tag with name " << kPluginName
      << ". Levels and distributed simulation will not work.\n";
  }
}

/////////////////////////////////////////////////
void LevelManager::ReadPerformers(const sdf::Plugin &_plugin)
{
  GZ_PROFILE("LevelManager::ReadPerformers");

  sdf::ElementPtr sdf = _plugin.ToElement();
  if (sdf == nullptr)
    return;

  if (sdf->HasElement("performer"))
  {
    gzdbg << "Reading performer info\n";
    for (auto performer = sdf->GetElement("performer"); performer;
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

        gzerr << "Found multiple performers (" << name << " and "
          << performer2->Data() << ") referring to the same entity\n";
      }

      sdf::Geometry geometry;
      geometry.Load(performer->GetElement("geometry"));
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Performer());
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::PerformerRef(ref));
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::PerformerLevels());
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Name(name));
      this->runner->entityCompMgr.CreateComponent(performerEntity,
          components::Geometry(geometry));

      gzmsg << "Created performer. EntityId[" << performerEntity
            << "] EntityName[" << name << "] Ref[" << ref << "]"
            << std::endl;
    }
  }

  if (this->useLevels && performerMap.empty())
  {
    gzdbg << "Levels enabled, but no <performer>s were speficied in SDF. Use "
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
  // to have long running service calls in gz-transport so that this
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
    gzerr << "Empty performer name. Performer will not be created\n";
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
  //   gzerr << "Unable to find model with name[" << name << "]. "
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
  //   gzwarn << "Performer with name[" << name << "] "
  //     << "has already been set.\n";
  // }

  // // The response succeeded.
  // return true;
}

/////////////////////////////////////////////////
void LevelManager::ReadLevels(const sdf::Plugin &_plugin)
{
  GZ_PROFILE("LevelManager::ReadLevels");

  gzdbg << "Reading levels info\n";

  sdf::ElementPtr sdf = _plugin.ToElement();
  if (sdf == nullptr)
    return;

  if (!sdf->HasElement("level"))
    return;

  for (auto level = sdf->GetElement("level"); level;
       level = level->GetNextElement("level"))
  {
    auto name = level->Get<std::string>("name");
    auto pose = level->Get<math::Pose3d>("pose");
    sdf::Geometry geometry;
    geometry.Load(level->GetElement("geometry"));

    if (nullptr == geometry.BoxShape())
    {
      gzerr << "Level [" << name << "]'s geometry is not a box, level won't "
             << "be created." << std::endl;
      continue;
    }

    double buffer = level->Get<double>("buffer", 0.0).first;
    if (buffer < 0)
    {
      gzwarn << "The buffer parameter for Level [" << name << "]cannot be a "
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

    auto worldEntity =
      this->runner->entityCompMgr.EntityByComponents(components::World());

    // All levels start inactive and unloaded.
    this->UnloadLevel(levelEntity);
    this->entityCreator->SetParent(levelEntity, worldEntity);

    gzdbg << "Created level with name[" << name << "] and pose["
      << pose << "]\n";
  }
}

/////////////////////////////////////////////////
void LevelManager::ConfigureDefaultLevel()
{
  GZ_PROFILE("LevelManager::ConfigureDefaultLevel");

  // Create the default level. This level contains all entities not contained by
  // any other level.
  Entity defaultLevel = this->runner->entityCompMgr.CreateEntity();

  // Go through all entities in the world and find ones not in the
  // set entityNamesInLevels

  std::set<std::string> entityNamesInDefault;

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld.ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld.ModelByIndex(modelIndex);
    if (!this->useLevels)
    {
      entityNamesInDefault.insert(model->Name());
      continue;
    }

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
       actorIndex < this->runner->sdfWorld.ActorCount(); ++actorIndex)
  {
    // There is no sdf::World::ActorByName so we have to iterate by index and
    // check if the actor is in this level
    auto actor = this->runner->sdfWorld.ActorByIndex(actorIndex);

    if (!this->useLevels)
    {
      entityNamesInDefault.insert(actor->Name());
      continue;
    }

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
       lightIndex < this->runner->sdfWorld.LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld.LightByIndex(lightIndex);
    if (this->entityNamesInLevels.find(light->Name()) ==
        this->entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(light->Name());
    }
  }

  // Joints
  // We assume no performers are joints
  for (uint64_t jointIndex = 0;
       jointIndex < this->runner->sdfWorld.JointCount(); ++jointIndex)
  {
    auto joint = this->runner->sdfWorld.JointByIndex(jointIndex);

    if (
        this->entityNamesInLevels.find(joint->Name()) ==
        this->entityNamesInLevels.end())
    {
      entityNamesInDefault.insert(joint->Name());
    }
  }

  // Components
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Level());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::DefaultLevel());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Name("default"));
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::LevelEntityNames(entityNamesInDefault));

  auto worldEntity =
      this->runner->entityCompMgr.EntityByComponents(components::World());

  this->entityCreator->SetParent(defaultLevel, worldEntity);
}

/////////////////////////////////////////////////
void LevelManager::UpdateLevelsState()
{
  GZ_PROFILE("LevelManager::UpdateLevelsState");

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
    GZ_PROFILE("DefaultLevel");
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
          GZ_PROFILE("EachPerformer");

          auto pose = this->runner->entityCompMgr.Component<components::Pose>(
              _parent->Data());

          // We assume the geometry contains a box.
          auto perfBox = _geometry->Data().BoxShape();
          if (nullptr == perfBox)
          {
          gzerr << "Internal error: geometry of performer [" << _perfEntity
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
                GZ_PROFILE("CheckPerformerAgainstLevel");
                // Check if the performer is in this level
                // assume a box for now
                auto box = _levelGeometry->Data().BoxShape();
                if (nullptr == box)
                {
                gzerr << "Level [" << _entity
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
  for (const Entity &toUnload : levelsToUnload)
  {
    this->UnloadLevel(toUnload, entityNamesMarked);
  }

  // Load the entities
  if (entityNamesToLoad.size() > 0)
    this->LoadActiveEntities(entityNamesToLoad);

  // Finally, update the list of active levels
  for (const auto &level : levelsToLoad)
  {
    if (!this->IsLevelActive(level))
    {
      const components::Name *lvlName =
      this->runner->entityCompMgr.Component<components::Name>(level);

      gzmsg << "Loaded level [" << lvlName->Data() << "]" << std::endl;
      this->activeLevels.insert(level);
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::LoadActiveEntities(const std::set<std::string> &_namesToLoad)
{
  GZ_PROFILE("LevelManager::LoadActiveEntities");

  auto worldEntity =
      this->runner->entityCompMgr.EntityByComponents(components::World());

  if (worldEntity == kNullEntity)
  {
    gzerr << "Could not find the world entity while loading levels\n";
    return;
  }

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld.ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld.ModelByIndex(modelIndex);
    if (_namesToLoad.find(model->Name()) != _namesToLoad.end() &&
        this->runner->EntityByName(model->Name()) == std::nullopt)
    {
      Entity modelEntity = this->entityCreator->CreateEntities(model);

      this->entityCreator->SetParent(modelEntity, worldEntity);
    }
  }

  // Actors
  for (uint64_t actorIndex = 0;
       actorIndex < this->runner->sdfWorld.ActorCount(); ++actorIndex)
  {
    // There is no sdf::World::ActorByName so we have to iterate by index and
    // check if the actor is in this level
    auto actor = this->runner->sdfWorld.ActorByIndex(actorIndex);
    if (_namesToLoad.find(actor->Name()) != _namesToLoad.end() &&
        this->runner->EntityByName(actor->Name()) == std::nullopt)
    {
      Entity actorEntity = this->entityCreator->CreateEntities(actor);

      this->entityCreator->SetParent(actorEntity, worldEntity);
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld.LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld.LightByIndex(lightIndex);
    if (_namesToLoad.find(light->Name()) != _namesToLoad.end() &&
        this->runner->EntityByName(light->Name()) == std::nullopt)
    {
      Entity lightEntity = this->entityCreator->CreateEntities(light);

      this->entityCreator->SetParent(lightEntity, worldEntity);
    }
  }

  // Joints
  for (uint64_t jointIndex = 0;
       jointIndex < this->runner->sdfWorld.JointCount(); ++jointIndex)
  {
    auto joint = this->runner->sdfWorld.JointByIndex(jointIndex);
    if (_namesToLoad.find(joint->Name()) != _namesToLoad.end())
    {
      Entity jointEntity = this->entityCreator->CreateEntities(joint);

      this->entityCreator->SetParent(jointEntity, worldEntity);
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

  this->runner->entityCompMgr.Each<components::Joint, components::Name>(
      [&](const Entity &_entity, const components::Joint *,
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
  return this->activeLevels.find(_entity) != this->activeLevels.end();
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
    gzwarn << "Attempting to set performer with name ["
      << _name << "] "
      << ", but the entity could not be found. Another attempt will be made "
      << "in the next iteration.\n";
    return -1;
  }

  if (!this->runner->entityCompMgr.ChildrenByComponents(modelEntity,
        components::Performer()).empty())
  {
    gzwarn << "Attempting to set performer with name ["
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

//////////////////////////////////////////////////
void LevelManager::UnloadLevel(const Entity &_entity,
    const std::set<std::string> &_entityNamesMarked)
{
  auto entityNames = this->runner->entityCompMgr
    .Component<components::LevelEntityNames>(_entity)
    ->Data();

  std::set<std::string> entityNamesToUnload;
  for (const auto &name : entityNames)
  {
    if (_entityNamesMarked.find(name) == _entityNamesMarked.end())
    {
      entityNamesToUnload.insert(name);
    }
  }

  if (!entityNamesToUnload.empty())
  {
    this->UnloadInactiveEntities(entityNamesToUnload);
  }
  this->activeLevels.erase(_entity);
  const components::Name *lvlName =
    this->runner->entityCompMgr.Component<components::Name>(_entity);

  gzmsg << "Unloaded level [" << lvlName->Data() << "]" << std::endl;
}
