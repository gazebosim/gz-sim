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

#include <functional>
#include <ignition/common/Console.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/PoseComponent.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ServerPrivate::ServerPrivate()
  : entityCompMgr(new EntityComponentManager())
{
  // Add the signal handler
  this->sigHandler.AddCallback(
      std::bind(&ServerPrivate::OnSignal, this, std::placeholders::_1));

  // Create a physics system
  this->systems.push_back(SystemInternal(
      std::move(std::make_unique<PhysicsSystem>())));
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
  // Update all the systems in parallel
  for (SystemInternal &system : this->systems)
  {
    this->workerPool.AddWork([&system, this] ()
    {
      for (std::pair<EntityQueryId, EntityQueryCallback> &cb : system.updates)
      {
        const std::optional<std::reference_wrapper<EntityQuery>> query =
          this->entityCompMgr->Query(cb.first);
        if (query)
        {
          cb.second(query->get(), *this->entityCompMgr.get());
        }
      }
    });
  }
  this->workerPool.WaitForResults();
}

/////////////////////////////////////////////////
void ServerPrivate::InitSystems()
{
  // Initialize all the systems in parallel.
  for (SystemInternal &system : this->systems)
  {
    this->workerPool.AddWork([&system, this] ()
    {
      EntityQueryRegistrar registrar;
      system.system->Init(registrar);
      for (EntityQueryRegistration &registration : registrar.Registrations())
      {
        EntityQuery &query = registration.first;
        EntityQueryCallback &cb = registration.second;
        EntityQueryId queryId = this->entityCompMgr->AddQuery(query);
        system.updates.push_back({queryId, cb});
      }
    });
  }

  this->workerPool.WaitForResults();
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

  // \todo(nkoenig) Systems will need a an update structure, such as
  // priorties, or a dependency chain.
  //
  // \todo(nkoenig) We should implement the two-phase update detailed
  // in the design.

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (uint64_t startingIterations = this->iterations;
       this->running && (_iterations == 0 ||
                         this->iterations < _iterations + startingIterations);
       ++this->iterations)
  {
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
