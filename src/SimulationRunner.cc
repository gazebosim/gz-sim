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
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "SimulationRunner.hh"

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/WorldStatistics.hh"
#include "ignition/gazebo/SystemManager.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"

using namespace ignition;
using namespace gazebo;

using StringSet = std::unordered_set<std::string>;
using SystemPtr = SimulationRunner::SystemPtr;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world,
                                   const std::vector<SystemPtr> &_systems)
{
  for (auto &system : _systems)
  {
    this->systems.push_back(SystemInternal(system));
  }

  this->CreateEntities(_world);
}

//////////////////////////////////////////////////
SimulationRunner::~SimulationRunner()
{
}

/////////////////////////////////////////////////
void SimulationRunner::InitSystems()
{
  // Initialize all the systems in parallel.
  for (SystemInternal &system : this->systems)
  {
    this->workerPool.AddWork([&system, this] ()
    {
      system.system->Init();
    });
  }

  this->workerPool.WaitForResults();
}

/////////////////////////////////////////////////
void SimulationRunner::UpdateSystems()
{
  // Update all the systems in parallel
  for (SystemInternal &system : this->systems)
  {
    this->workerPool.AddWork([&system, this] ()
    {

    });
  }
  this->workerPool.WaitForResults();
}

/////////////////////////////////////////////////
void SimulationRunner::Stop()
{
  this->running = false;
}

/////////////////////////////////////////////////
bool SimulationRunner::Run(const uint64_t _iterations)
{
  // \todo(nkoenig) Systems will need a an update structure, such as
  // priorties, or a dependency chain.
  //
  // \todo(nkoenig) We should implement the two-phase update detailed
  // in the design.

  // Variables for time keeping.
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::duration sleepTime;
  std::chrono::steady_clock::duration actualSleep;

  this->running = true;

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (uint64_t startingIterations = this->iterations;
       this->running && (_iterations == 0 ||
         this->iterations < _iterations + startingIterations);
       ++this->iterations)
  {
    // Compute the time to sleep in order to match, as closely as possible,
    // the update period.
    sleepTime = std::max(0ns, this->prevUpdateWallTime +
        this->updatePeriod - std::chrono::steady_clock::now() -
        this->sleepOffset);
    actualSleep = 0ns;

    // Only sleep if needed.
    if (sleepTime > 0ns)
    {
      // Get the current time, sleep for the duration needed to match the
      // updatePeriod, and then record the actual time slept.
      startTime = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(sleepTime);
      actualSleep = std::chrono::steady_clock::now() - startTime;
    }

    // Exponentially average out the different between expected sleep time
    // and actual sleep time.
    this->sleepOffset =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (actualSleep - sleepTime) * 0.01 + this->sleepOffset * 0.99);

    // Record when the update step starts.
    this->prevUpdateWallTime = std::chrono::steady_clock::now();

    // Update all the systems.
    this->UpdateSystems();
  }

  this->running = false;
  return true;
}

//////////////////////////////////////////////////
void SimulationRunner::CreateEntities(const sdf::World *_world)
{
  // World entity
  EntityId worldEntity = this->entityCompMgr.CreateEntity();

  /// \todo(nkoenig) Computing the desired update period here is a bit
  /// hacky.
  components::World worldComponent(_world);
  std::chrono::steady_clock::duration stepSize = worldComponent.MaxStep();
  double rtf = worldComponent.DesiredRealTimeFactor();
  this->updatePeriod = std::chrono::nanoseconds(
      static_cast<int>(stepSize.count() / rtf));

  // World components
  this->entityCompMgr.CreateComponent(worldEntity, worldComponent);
  this->entityCompMgr.CreateComponent(worldEntity,
      components::WorldStatistics());
  this->entityCompMgr.CreateComponent(worldEntity,
      components::Name(_world->Name()));

  // Models
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    auto model = _world->ModelByIndex(modelIndex);

    // Entity
    EntityId modelEntity = this->entityCompMgr.CreateEntity();

    // Components
    this->entityCompMgr.CreateComponent(modelEntity, components::Model());
    this->entityCompMgr.CreateComponent(modelEntity,
        components::Pose(model->Pose()));
    this->entityCompMgr.CreateComponent(modelEntity,
        components::Name(model->Name()));
    this->entityCompMgr.CreateComponent(modelEntity,
        components::ParentEntity(worldEntity));

    // Links
    for (uint64_t linkIndex = 0; linkIndex < model->LinkCount();
        ++linkIndex)
    {
      auto link = model->LinkByIndex(linkIndex);

      // Entity
      EntityId linkEntity = this->entityCompMgr.CreateEntity();

      // Components
      this->entityCompMgr.CreateComponent(linkEntity, components::Link());
      this->entityCompMgr.CreateComponent(linkEntity,
          components::Pose(link->Pose()));
      this->entityCompMgr.CreateComponent(linkEntity,
          components::Name(link->Name()));
      this->entityCompMgr.CreateComponent(linkEntity,
          components::ParentEntity(modelEntity));

      // Visuals
      for (uint64_t visualIndex = 0; visualIndex < link->VisualCount();
          ++visualIndex)
      {
        auto visual = link->VisualByIndex(visualIndex);

        // Entity
        EntityId visualEntity = this->entityCompMgr.CreateEntity();

        // Components
        this->entityCompMgr.CreateComponent(visualEntity, components::Visual());
        this->entityCompMgr.CreateComponent(visualEntity,
            components::Pose(visual->Pose()));
        this->entityCompMgr.CreateComponent(visualEntity,
            components::Name(visual->Name()));
        this->entityCompMgr.CreateComponent(visualEntity,
            components::ParentEntity(linkEntity));

        if (visual->Geom())
        {
          this->entityCompMgr.CreateComponent(visualEntity,
              components::Geometry(*visual->Geom()));
        }

        // \todo(louise) Populate with default material if undefined
        if (visual->Material())
        {
          this->entityCompMgr.CreateComponent(visualEntity,
              components::Material(*visual->Material()));
        }
      }

      // Collisions
      for (uint64_t collisionIndex = 0; collisionIndex < link->CollisionCount();
          ++collisionIndex)
      {
        auto collision = link->CollisionByIndex(collisionIndex);

        // Entity
        EntityId collisionEntity = this->entityCompMgr.CreateEntity();

        // Components
        this->entityCompMgr.CreateComponent(collisionEntity,
            components::Collision());
        this->entityCompMgr.CreateComponent(collisionEntity,
            components::Pose(collision->Pose()));
        this->entityCompMgr.CreateComponent(collisionEntity,
            components::Name(collision->Name()));
        this->entityCompMgr.CreateComponent(collisionEntity,
            components::ParentEntity(linkEntity));

        if (collision->Geom())
        {
          this->entityCompMgr.CreateComponent(collisionEntity,
              components::Geometry(*collision->Geom()));
        }
      }
    }
  }
}

/////////////////////////////////////////////////
bool SimulationRunner::Running() const
{
  return this->running;
}

/////////////////////////////////////////////////
uint64_t SimulationRunner::IterationCount() const
{
  return this->iterations;
}

/////////////////////////////////////////////////
size_t SimulationRunner::EntityCount() const
{
  return this->entityCompMgr.EntityCount();
}

/////////////////////////////////////////////////
size_t SimulationRunner::SystemCount() const
{
  return this->systems.size();
}

/////////////////////////////////////////////////
void SimulationRunner::SetUpdatePeriod(
    const std::chrono::steady_clock::duration &_updatePeriod)
{
  this->updatePeriod = _updatePeriod;
}
