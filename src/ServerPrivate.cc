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

#include "ServerPrivate.hh"

#include <chrono>
#include <functional>
#include <ignition/common/Console.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include "ignition/gazebo/WorldStatisticsSystem.hh"
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/PoseComponent.hh"
#include "WorldStatisticsComponent.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
ServerPrivate::ServerPrivate()
  : entityCompMgr(new EntityComponentManager())
{
  // Add the signal handler
  this->sigHandler.AddCallback(
      std::bind(&ServerPrivate::OnSignal, this, std::placeholders::_1));

  // Create a world statistics system
  this->systems.push_back(SystemInternal(
        std::move(std::make_unique<WorldStatisticsSystem>())));

  // Create a physics system
  this->systems.push_back(SystemInternal(
      std::move(std::make_unique<PhysicsSystem>())));

  // The server needs at least one world, so create it here.
  this->worldEntity = this->entityCompMgr->CreateEntity();

  // Create the world statistcs component for the world entity.
  this->worldStatsComp = this->entityCompMgr->CreateComponent(
        this->worldEntity, WorldStatisticsComponent());
  auto *worldStats =
    this->entityCompMgr->ComponentMutable<WorldStatisticsComponent>(
        this->worldStatsComp);
  worldStats->RealTime().Start();
}

/////////////////////////////////////////////////
ServerPrivate::~ServerPrivate()
{
  if (this->runThread.joinable())
  {
    this->running = false;
    this->runThread.join();
  }
}

/////////////////////////////////////////////////
void ServerPrivate::UpdateSystems()
{
  std::cout << "==========================\n";
  // \todo(nkoenig) Update the systems in parallel
  for (SystemInternal &system : this->systems)
  {
    for (std::pair<EntityQueryId, EntityQueryCallback> &cb : system.updates)
    {
      const std::optional<std::reference_wrapper<EntityQuery>> query =
        this->entityCompMgr->Query(cb.first);
      if (query && query->get().EntityCount() > 0)
      {
        cb.second(query->get(), *this->entityCompMgr.get());
      }
    }
  }
}

/////////////////////////////////////////////////
bool ServerPrivate::Run(const uint64_t _iterations,
    std::optional<std::condition_variable *> _cond)
{
  this->runMutex.lock();
  this->running = true;
  if (_cond)
    _cond.value()->notify_all();
  this->runMutex.unlock();

  // Initialize all the systems.
  for (SystemInternal &system : this->systems)
  {
    EntityQueryRegistrar registrar;
    system.system->Init(registrar);
    for (const EntityQueryRegistration &registration :
         registrar.Registrations())
    {
      const EntityQuery &query = registration.first;
      const EntityQueryCallback &cb = registration.second;
      EntityQueryId queryId = this->entityCompMgr->AddQuery(query);
      system.updates.push_back({queryId, cb});
    }
  }

  // Duration that each step simulates.
  std::chrono::steady_clock::duration stepTime = 10ms;

  // Update period. If this matches stepTime, then simulation should run
  // in realtime. If this is less than stepTime, then simulation should
  // faster than real time.
  std::chrono::steady_clock::duration updatePeriod = 10ms;

  // \todo(nkoenig) Systems will need a an update structure, such as
  // priorties, or a dependency chain.
  //
  // \todo(nkoenig) We should implement the two-phase update detailed
  // in the design.

  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::duration sleepTime;
  std::chrono::steady_clock::duration actualSleep;

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (uint64_t startingIterations = this->iterations;
       this->running && (_iterations == 0 ||
                         this->iterations < _iterations + startingIterations);
       ++this->iterations)
  {
    // Compute the time to sleep in order to match, as closely as possible,
    // the update period.
    sleepTime = std::max(0ns, this->prevStepWallTime + updatePeriod -
        std::chrono::steady_clock::now() - this->sleepOffset);

    // Get the current time, sleep for the duration needed to match the
    // updatePeriod, and then record the actual time slept.
    startTime = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(sleepTime);
    actualSleep = std::chrono::steady_clock::now() - startTime;

    // Exponentially average out the different between expected sleep time
    // and actual sleep time.
    this->sleepOffset = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (actualSleep - sleepTime) * 0.01 + this->sleepOffset * 0.99);

    // Record when the update step starts.
    this->prevStepWallTime = std::chrono::steady_clock::now();

    this->UpdateSystems();
  }

  this->running = false;
  return true;
}

//////////////////////////////////////////////////
void ServerPrivate::OnSignal(int _sig)
{
  igndbg << "Server received signal[" << _sig  << "]\n";
  this->running = false;
}

//////////////////////////////////////////////////
void ServerPrivate::CreateEntities(const sdf::Root &_root)
{
  for (uint64_t i = 0; i < _root.ModelCount(); ++i)
  {
    // Get the SDF model
    const sdf::Model *model = _root.ModelByIndex(i);

    // Create an entity for the model.
    EntityId entityId = this->entityCompMgr->CreateEntity();

    // Create the pose component for the model.
    this->entityCompMgr->CreateComponent(
        entityId, PoseComponent(model->Pose()));
  }
}
