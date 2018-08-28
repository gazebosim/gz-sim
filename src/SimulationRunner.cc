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
#include <sdf/Physics.hh>
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
#include "ignition/gazebo/SystemQueryResponse.hh"

#include "ignition/gazebo/systems/Physics.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world)
{
  // Keep world name
  this->worldName = _world->Name();

  // Create a physics system
  this->systems.push_back(SystemInternal(
      std::move(std::make_unique<systems::Physics>())));

  // Step size
  auto dur = std::chrono::duration<double>(
      _world->PhysicsDefault()->MaxStepSize());

  this->stepSize =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      dur);

  // Desired real time factor
  double desiredRtf = _world->PhysicsDefault()->RealTimeFactor();

  // Desired simulation update period
  this->simUpdatePeriod = std::chrono::nanoseconds(
      static_cast<int>(this->stepSize.count() / desiredRtf));

  // Create entities and components
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
void SimulationRunner::PublishStats(const UpdateInfo &_info)
{
  // Create the world statistics publisher.
  if (this->statsPub.Valid())
  {
    transport::AdvertiseMessageOptions advertOpts;
    advertOpts.SetMsgsPerSec(5);
    this->statsPub = this->node.Advertise<ignition::msgs::WorldStatistics>(
          "/world/" + this->worldName + "/stats", advertOpts);
  }

  // Create the world statistics message.
  ignition::msgs::WorldStatistics msg;
  msg.set_real_time_factor(this->realTimeFactor);

  auto realTimeSecNsec =
    ignition::math::durationToSecNsec(_info.realTime);

  auto simTimeSecNsec =
    ignition::math::durationToSecNsec(_info.simTime);

  msg.mutable_real_time()->set_sec(realTimeSecNsec.first);
  msg.mutable_real_time()->set_nsec(realTimeSecNsec.second);

  msg.mutable_sim_time()->set_sec(simTimeSecNsec.first);
  msg.mutable_sim_time()->set_nsec(simTimeSecNsec.second);

  msg.set_iterations(_info.iterations);

  // Publish the message
  this->statsPub.Publish(msg);
}

/////////////////////////////////////////////////
void SimulationRunner::UpdateSystems(const UpdateInfo &_info)
{
  // Update all the systems in parallel
  for (SystemInternal &system : this->systems)
  {
    this->workerPool.AddWork([&system, &_info, this] ()
    {
      for (std::pair<EntityQueryId, EntityQueryCallback> &cb : system.updates)
      {
        const std::optional<std::reference_wrapper<EntityQuery>> query =
          this->entityCompMgr.Query(cb.first);
        if (query && query->get().EntityCount() > 0)
        {
          SystemQueryResponse response(query->get(), this->entityCompMgr);
          cb.second(_info, response);
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

  // Keep track of wall clock time
  this->realTimeWatch.Start();

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
    // the ECS update period.
    sleepTime = std::max(0ns, this->ecsPrevUpdateRealTime +
        this->ecsUpdatePeriod - std::chrono::steady_clock::now() -
        this->ecsSleepOffset);
    actualSleep = 0ns;

    // Only sleep if needed.
    if (sleepTime > 0ns)
    {
      // Get the current time, sleep for the duration needed to match the
      // ecsUpdatePeriod, and then record the actual time slept.
      startTime = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(sleepTime);
      actualSleep = std::chrono::steady_clock::now() - startTime;
    }

    // Exponentially average out the difference between expected sleep time
    // and actual sleep time.
    this->ecsSleepOffset =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (actualSleep - sleepTime) * 0.01 + this->ecsSleepOffset * 0.99);

    // Store the real time, and maintain a window size of 20.
    this->realTimes.push_back(this->realTimeWatch.ElapsedRunTime());
    if (this->realTimes.size() > 20)
    {
      this->realTimes.pop_front();
    }

    // Store the sim time, and maintain a window size of 20.
    this->simTimes.push_back(this->simTime);
    if (this->simTimes.size() > 20)
    {
      this->simTimes.pop_front();
    }

    // Compute the average sim and real times.
    std::chrono::steady_clock::duration simAvg{0}, realAvg{0};
    std::list<std::chrono::steady_clock::duration>::iterator simIter,
      realIter;

    simIter = ++(this->simTimes.begin());
    realIter = ++(this->realTimes.begin());
    for (;simIter != this->simTimes.end() &&
        realIter != this->realTimes.end(); ++simIter, ++realIter)
    {
      simAvg += ((*simIter) - this->simTimes.front());
      realAvg += ((*realIter) - this->realTimes.front());
    }

    // RTF
    if (realAvg != 0ns)
    {
      this->realTimeFactor = math::precision(
            static_cast<double>(simAvg.count()) / realAvg.count(), 4);
    }

    // Fill the current update info
    UpdateInfo info;
    info.dt = this->simUpdatePeriod;
    info.simTime = this->simTime;
    info.realTime = this->realTimeWatch.ElapsedRunTime();
    info.iterations = this->iterations;

    // Publish info
    this->PublishStats(info);

    // Record when the update step starts.
    this->ecsPrevUpdateRealTime = std::chrono::steady_clock::now();

    // Update all the systems.
    this->UpdateSystems(info);

    // Update sim time
    this->simTime += this->stepSize;
  }

  this->running = false;
  return true;
}

//////////////////////////////////////////////////
void SimulationRunner::CreateEntities(const sdf::World *_world)
{
  // World entity
  EntityId worldEntity = this->entityCompMgr.CreateEntity();

  // World components
  this->entityCompMgr.CreateComponent(worldEntity, worldComponent);
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
    const std::chrono::steady_clock::duration &_ecsUpdatePeriod)
{
  this->ecsUpdatePeriod = _ecsUpdatePeriod;
}
