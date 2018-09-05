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
#include "ignition/gazebo/SystemManager.hh"

using namespace ignition;
using namespace gazebo;

using StringSet = std::unordered_set<std::string>;
using SystemPtr = SimulationRunner::SystemPtr;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world,
                                   const std::vector<SystemPtr> &_systems)
{
  // Keep world name
  this->worldName = _world->Name();

  // Store systems
  for (auto &system : _systems)
  {
    this->systems.push_back(SystemInternal(system));
  }

  // Get the first physics profile
  // \todo(louise) Support picking a specific profile
  auto physics = _world->PhysicsByIndex(0);
  if (!physics)
  {
    physics = _world->PhysicsDefault();
  }

  // Step size
  auto dur = std::chrono::duration<double>(physics->MaxStepSize());

  this->stepSize =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      dur);

  // Desired real time factor
  double desiredRtf = _world->PhysicsDefault()->RealTimeFactor();

  // The instantaneous real time factor is given as:
  //
  // RTF = sim_time / real_time
  //
  // Where the sim time is the step size times the number of sim iterations:
  //
  // sim_time = sim_it * step_size
  //
  // And the real time is the period times the number of iterations:
  //
  // real_time = it * period
  //
  // So we have:
  //
  // RTF = sim_it * step_size / it * period
  //
  // Considering no pause, sim_it equals it, so:
  //
  // RTF = step_size / period
  //
  // So to get a given RTF, our desired period is:
  //
  // period = step_size / RTF
  this->updatePeriod = std::chrono::nanoseconds(
      static_cast<int>(this->stepSize.count() / desiredRtf));

  // Create entities and components
  this->CreateEntities(_world);

  // World control
  this->node.Advertise("/world/" + this->worldName + "/control",
        &SimulationRunner::OnWorldControl, this);

  ignmsg << "World [" << _world->Name() << "] initialized with ["
         << physics->Name() << "] physics profile." << std::endl;
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
      system.system->Init(system.updates);
    });
  }

  this->workerPool.WaitForResults();
}

/////////////////////////////////////////////////
UpdateInfo SimulationRunner::UpdatedInfo()
{
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
  while (simIter != this->simTimes.end() && realIter != this->realTimes.end())
  {
    simAvg += ((*simIter) - this->simTimes.front());
    realAvg += ((*realIter) - this->realTimes.front());
    ++simIter;
    ++realIter;
  }

  // RTF
  if (realAvg != 0ns)
  {
    this->realTimeFactor = math::precision(
          static_cast<double>(simAvg.count()) / realAvg.count(), 4);
  }

  // Fill the current update info
  UpdateInfo info;
  info.simTime = this->simTime;
  info.realTime = this->realTimeWatch.ElapsedRunTime();
  info.iterations = this->simIterations;
  if (!this->paused || this->pendingSimIterations > 0)
  {
    info.dt = this->stepSize;
  }
  else
  {
    info.dt = std::chrono::steady_clock::duration::zero();
  }

  return info;
}

/////////////////////////////////////////////////
void SimulationRunner::PublishStats(const UpdateInfo &_info)
{
  // Create the world statistics publisher.
  if (!this->statsPub.Valid())
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

  msg.set_paused(this->paused);

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
      for (EntityQueryCallback &cb : system.updates)
      {
        cb(_info, this->entityCompMgr);
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
    // the update period.
    sleepTime = std::max(0ns, this->prevUpdateRealTime +
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

    // Exponentially average out the difference between expected sleep time
    // and actual sleep time.
    this->sleepOffset =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (actualSleep - sleepTime) * 0.01 + this->sleepOffset * 0.99);

    // Get updated time information
    auto info = this->UpdatedInfo();

    // Publish info
    this->PublishStats(info);

    // Record when the update step starts.
    this->prevUpdateRealTime = std::chrono::steady_clock::now();

    // Update all the systems.
    this->UpdateSystems(info);

    // Update sim time and sim iterations
    if (!this->paused || this->pendingSimIterations > 0)
    {
      this->simTime += this->stepSize;
      ++this->simIterations;

      if (this->pendingSimIterations > 0)
        --this->pendingSimIterations;
    }
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
  this->entityCompMgr.CreateComponent(worldEntity, components::World());
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

/////////////////////////////////////////////////
bool SimulationRunner::OnWorldControl(const msgs::WorldControl &_req,
                                      msgs::Boolean &_res)
{
  // Play / pause
  this->paused = _req.pause();

  // Step
  if (_req.multi_step() > 0)
  {
    // Pause for stepping, if not paused yet
    this->paused = true;

    this->pendingSimIterations += _req.multi_step();
  }

  _res.set_data(true);
  return true;
}
