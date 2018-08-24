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

#include <sdf/Model.hh>
#include <sdf/World.hh>

#include "SimulationRunner.hh"

#include "ignition/gazebo/SystemManager.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/WorldStatistics.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world,
                                   const std::unordered_set<std::string> &_systems,
                                   SystemManager *_system_manager)
{
  for (auto& system : _systems) {
    auto systemPlugin = _system_manager->Instantiate(system);
    this->systems.push_back(SystemInternal(systemPlugin));
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
      EntityQueryRegistrar registrar;
      system.system->Init(registrar);
      for (const EntityQueryRegistration &registration :
           registrar.Registrations())
      {
        const EntityQuery &query = registration.first;
        const EntityQueryCallback &cb = registration.second;
        EntityQueryId queryId = this->entityCompMgr.AddQuery(query);
        system.updates.push_back({queryId, cb});
      }
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
      for (std::pair<EntityQueryId, EntityQueryCallback> &cb : system.updates)
      {
        const std::optional<std::reference_wrapper<EntityQuery>> query =
          this->entityCompMgr.Query(cb.first);
        if (query && query->get().EntityCount() > 0)
        {
          SystemQueryResponse response(query->get(), this->entityCompMgr);
          cb.second(response);
        }
      }
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
  // The server needs at least one world, so create it here.
  EntityId worldEntity = this->entityCompMgr.CreateEntity();

  /// \todo(nkoenig) Computing the desired update period here is a bit
  /// hacky.
  components::World worldComponent(_world);
  std::chrono::steady_clock::duration stepSize = worldComponent.MaxStep();
  double rtf = worldComponent.DesiredRealTimeFactor();
  this->updatePeriod = std::chrono::nanoseconds(
      static_cast<int>(stepSize.count() / rtf));

  // Create the world component for the world entity.
  this->entityCompMgr.CreateComponent(worldEntity, worldComponent);

  // Create the world statistcs component for the world entity.
  this->entityCompMgr.CreateComponent(
      worldEntity, components::WorldStatistics());

  // Process each model in the world
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    // Get the SDF model
    const sdf::Model *model = _world->ModelByIndex(modelIndex);

    // Create an entity for the model.
    EntityId entityId = this->entityCompMgr.CreateEntity();

    // Create the pose component for the model.
    this->entityCompMgr.CreateComponent(
        entityId, components::Pose(model->Pose()));
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
