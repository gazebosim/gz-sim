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

#include "SimulationRunner.hh"

#include "ignition/gazebo/Events.hh"

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;

using StringSet = std::unordered_set<std::string>;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world,
                                   const SystemLoaderPtr &_systemLoader)
{
  // Keep world name
  this->worldName = _world->Name();

  // Keep system loader to plugins can be loaded at runtime
  this->systemLoader = _systemLoader;

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

  this->pauseConn = this->eventMgr.Connect<events::Pause>(
      std::bind(&SimulationRunner::SetPaused, this, std::placeholders::_1));

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
void SimulationRunner::UpdateCurrentInfo()
{
  // Store the real time and sim time only if not paused.
  if (this->realTimeWatch.Running())
  {
    this->realTimes.push_back(this->realTimeWatch.ElapsedRunTime());
    this->simTimes.push_back(this->currentInfo.simTime);
  }

  // Maintain a window size of 20 for realtime and simtime.
  if (this->realTimes.size() > 20)
    this->realTimes.pop_front();
  if (this->simTimes.size() > 20)
    this->simTimes.pop_front();

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

  // RTF, only compute this if the realTime count is greater than zero. The
  // realtTime count could be zero if simulation was started paused.
  if (realAvg.count() > 0)
  {
    this->realTimeFactor = math::precision(
          static_cast<double>(simAvg.count()) / realAvg.count(), 4);
  }

  // Fill the current update info
  this->currentInfo.realTime = this->realTimeWatch.ElapsedRunTime();
  if (!this->currentInfo.paused)
  {
    this->currentInfo.simTime += this->stepSize;
    ++this->currentInfo.iterations;
    this->currentInfo.dt = this->stepSize;
  }
  else
  {
    this->currentInfo.dt = std::chrono::steady_clock::duration::zero();
  }
}

/////////////////////////////////////////////////
void SimulationRunner::PublishStats()
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
    ignition::math::durationToSecNsec(this->currentInfo.realTime);

  auto simTimeSecNsec =
    ignition::math::durationToSecNsec(this->currentInfo.simTime);

  msg.mutable_real_time()->set_sec(realTimeSecNsec.first);
  msg.mutable_real_time()->set_nsec(realTimeSecNsec.second);

  msg.mutable_sim_time()->set_sec(simTimeSecNsec.first);
  msg.mutable_sim_time()->set_nsec(simTimeSecNsec.second);

  msg.set_iterations(this->currentInfo.iterations);

  msg.set_paused(this->currentInfo.paused);

  // Publish the message
  this->statsPub.Publish(msg);
}

/////////////////////////////////////////////////
void SimulationRunner::AddSystem(const SystemPluginPtr &_system)
{
  this->systems.push_back(SystemInternal(_system));

  const auto &system = this->systems.back();

  if (system.preupdate)
    this->systemsPreupdate.push_back(system.preupdate);

  if (system.update)
    this->systemsUpdate.push_back(system.update);

  if (system.postupdate)
    this->systemsPostupdate.push_back(system.postupdate);
}

/////////////////////////////////////////////////
void SimulationRunner::UpdateSystems()
{
  // \todo(nkoenig)  Systems used to be updated in parallel using
  // an ignition::common::WorkerPool. There is overhead associated with
  // this, most notably the creation and destruction of WorkOrders (see
  // WorkerPool.cc). We could turn on parallel updates in the future, and/or
  // turn it on if there are sufficient systems. More testing is required.

  for (auto& system : this->systemsPreupdate)
    system->PreUpdate(this->currentInfo, this->entityCompMgr);

  for (auto& system : this->systemsUpdate)
    system->Update(this->currentInfo, this->entityCompMgr);

  for (auto& system : this->systemsPostupdate)
    system->PostUpdate(this->currentInfo, this->entityCompMgr);
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

  // Keep track of wall clock time. Only start the realTimeWatch if this
  // runner is not paused.
  if (!this->currentInfo.paused)
    this->realTimeWatch.Start();

  // Variables for time keeping.
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::duration sleepTime;
  std::chrono::steady_clock::duration actualSleep;

  this->running = true;

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (uint64_t startingIterations = this->currentInfo.iterations;
       this->running && (_iterations == 0 ||
         this->currentInfo.iterations < _iterations + startingIterations);)
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

    // Update time information. This will update the iteration count, RTF,
    // and other values.
    this->UpdateCurrentInfo();

    // Publish info
    this->PublishStats();

    // Record when the update step starts.
    this->prevUpdateRealTime = std::chrono::steady_clock::now();

    // Update all the systems.
    this->UpdateSystems();

    if (!this->Paused() && this->pendingSimIterations > 0)
    {
      // Decrement the pending sim iterations, if there are any.
      --this->pendingSimIterations;
      // If this is was the last sim iterations, then re-pause simulation.
      if (this->pendingSimIterations <= 0)
      {
        this->SetPaused(true);
      }
    }

    // Process world control messages.
    this->ProcessMessages();

    // Process entity erasures.
    this->entityCompMgr.ProcessEraseEntityRequests();
  }

  this->running = false;
  return true;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::World *_world)
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
    auto modelEntity = this->CreateEntities(model);

    this->entityCompMgr.CreateComponent(modelEntity,
        components::ParentEntity(worldEntity));
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _world->LightCount();
      ++lightIndex)
  {
    auto light = _world->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->entityCompMgr.CreateComponent(lightEntity,
        components::ParentEntity(worldEntity));
  }

  this->LoadPlugins(_world->Element(), worldEntity);

  return worldEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Model *_model)
{
  // Entity
  EntityId modelEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(modelEntity, components::Model());
  this->entityCompMgr.CreateComponent(modelEntity,
      components::Pose(_model->Pose()));
  this->entityCompMgr.CreateComponent(modelEntity,
      components::Name(_model->Name()));
  this->entityCompMgr.CreateComponent(modelEntity,
      components::Static(_model->Static()));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->entityCompMgr.CreateComponent(linkEntity,
        components::ParentEntity(modelEntity));
    if (linkIndex == 0)
    {
      this->entityCompMgr.CreateComponent(linkEntity,
          components::CanonicalLink());
    }
  }

  // Joints
  for (uint64_t jointIndex = 0; jointIndex < _model->JointCount();
      ++jointIndex)
  {
    auto joint = _model->JointByIndex(jointIndex);
    auto linkEntity = this->CreateEntities(joint);

    this->entityCompMgr.CreateComponent(linkEntity,
        components::ParentEntity(modelEntity));
  }

  // Model plugins
  this->LoadPlugins(_model->Element(), modelEntity);

  return modelEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Light *_light)
{
  // Entity
  EntityId lightEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(lightEntity, components::Light(*_light));
  this->entityCompMgr.CreateComponent(lightEntity,
      components::Pose(_light->Pose()));
  this->entityCompMgr.CreateComponent(lightEntity,
      components::Name(_light->Name()));

  return lightEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Link *_link)
{
  // Entity
  EntityId linkEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(linkEntity, components::Link());
  this->entityCompMgr.CreateComponent(linkEntity,
      components::Pose(_link->Pose()));
  this->entityCompMgr.CreateComponent(linkEntity,
      components::Name(_link->Name()));
  this->entityCompMgr.CreateComponent(linkEntity,
      components::Inertial(_link->Inertial()));

  // Visuals
  for (uint64_t visualIndex = 0; visualIndex < _link->VisualCount();
      ++visualIndex)
  {
    auto visual = _link->VisualByIndex(visualIndex);
    auto visualEntity = this->CreateEntities(visual);

    this->entityCompMgr.CreateComponent(visualEntity,
        components::ParentEntity(linkEntity));
  }

  // Collisions
  for (uint64_t collisionIndex = 0; collisionIndex < _link->CollisionCount();
      ++collisionIndex)
  {
    auto collision = _link->CollisionByIndex(collisionIndex);
    auto collisionEntity = this->CreateEntities(collision);

    this->entityCompMgr.CreateComponent(collisionEntity,
        components::ParentEntity(linkEntity));
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _link->LightCount();
      ++lightIndex)
  {
    auto light = _link->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->entityCompMgr.CreateComponent(lightEntity,
        components::ParentEntity(linkEntity));
  }

  return linkEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Joint *_joint)
{
  // Entity
  EntityId jointEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(jointEntity,
      components::Joint());
  this->entityCompMgr.CreateComponent(jointEntity,
      components::JointType(_joint->Type()));

  if (_joint->Axis(0))
  {
    this->entityCompMgr.CreateComponent(jointEntity,
        components::JointAxis(*_joint->Axis(0)));
  }

  if (_joint->Axis(1))
  {
    this->entityCompMgr.CreateComponent(jointEntity,
        components::JointAxis2(*_joint->Axis(1)));
  }

  this->entityCompMgr.CreateComponent(jointEntity,
      components::Pose(_joint->Pose()));
  this->entityCompMgr.CreateComponent(jointEntity ,
      components::Name(_joint->Name()));
  this->entityCompMgr.CreateComponent(jointEntity ,
      components::ThreadPitch(_joint->ThreadPitch()));
  this->entityCompMgr.CreateComponent(jointEntity,
      components::ParentLinkName(_joint->ParentLinkName()));
  this->entityCompMgr.CreateComponent(jointEntity,
      components::ChildLinkName(_joint->ChildLinkName()));

  return jointEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Visual *_visual)
{
  // Entity
  EntityId visualEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(visualEntity, components::Visual());
  this->entityCompMgr.CreateComponent(visualEntity,
      components::Pose(_visual->Pose()));
  this->entityCompMgr.CreateComponent(visualEntity,
      components::Name(_visual->Name()));

  if (_visual->Geom())
  {
    this->entityCompMgr.CreateComponent(visualEntity,
        components::Geometry(*_visual->Geom()));
  }

  // \todo(louise) Populate with default material if undefined
  if (_visual->Material())
  {
    this->entityCompMgr.CreateComponent(visualEntity,
        components::Material(*_visual->Material()));
  }

  return visualEntity;
}

//////////////////////////////////////////////////
EntityId SimulationRunner::CreateEntities(const sdf::Collision *_collision)
{
  // Entity
  EntityId collisionEntity = this->entityCompMgr.CreateEntity();

  // Components
  this->entityCompMgr.CreateComponent(collisionEntity,
      components::Collision());
  this->entityCompMgr.CreateComponent(collisionEntity,
      components::Pose(_collision->Pose()));
  this->entityCompMgr.CreateComponent(collisionEntity,
      components::Name(_collision->Name()));

  if (_collision->Geom())
  {
    this->entityCompMgr.CreateComponent(collisionEntity,
        components::Geometry(*_collision->Geom()));
  }

  return collisionEntity;
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugins(const sdf::ElementPtr &_sdf,
    const EntityId _id)
{
  if (!_sdf->HasElement("plugin"))
    return;

  sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
  while (pluginElem)
  {
    auto system = this->systemLoader->LoadPlugin(pluginElem);
    if (system)
    {
      auto systemConfig = system.value()->QueryInterface<ISystemConfigure>();
      if (systemConfig != nullptr)
      {
        systemConfig->Configure(_id, pluginElem,
                                this->entityCompMgr,
                                this->eventMgr);
      }
      this->AddSystem(system.value());
    }
    pluginElem = pluginElem->GetNextElement("plugin");
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
  return this->currentInfo.iterations;
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
void SimulationRunner::SetPaused(const bool _paused)
{
  // Only update the realtime clock if Run() has been called.
  if (this->running)
  {
    // Start or stop the realtime stopwatch based on _paused. We don't need to
    // check the stopwatch state here since the stopwatch class checks its
    // running state inside Stop() and Start().
    if (_paused)
    {
      this->realTimeWatch.Stop();
    }
    else
      this->realTimeWatch.Start();
  }

  // Store the pause state
  this->currentInfo.paused = _paused;
}

/////////////////////////////////////////////////
bool SimulationRunner::OnWorldControl(const msgs::WorldControl &_req,
    msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);
  this->worldControlMsgs.push_back(_req);
  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessMessages()
{
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);
  this->ProcessWorldControl();
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessWorldControl()
{
  for (const msgs::WorldControl &msg : this->worldControlMsgs)
  {
    // Play / pause
    this->SetPaused(msg.pause());

    // Step, only if we are paused.
    if (this->Paused() && msg.multi_step() > 0)
    {
      this->pendingSimIterations += msg.multi_step();
      // Unpause so that stepping can occur.
      this->SetPaused(false);
    }
  }

  this->worldControlMsgs.clear();
}

/////////////////////////////////////////////////
bool SimulationRunner::Paused() const
{
  return this->currentInfo.paused;
}

/////////////////////////////////////////////////
const EntityComponentManager &SimulationRunner::EntityCompMgr() const
{
  return this->entityCompMgr;
}

/////////////////////////////////////////////////
const UpdateInfo &SimulationRunner::CurrentInfo() const
{
  return this->currentInfo;
}

/////////////////////////////////////////////////
const std::chrono::steady_clock::duration &
SimulationRunner::UpdatePeriod() const
{
  return this->updatePeriod;
}

/////////////////////////////////////////////////
const ignition::math::clock::duration &SimulationRunner::StepSize() const
{
  return this->stepSize;
}

/////////////////////////////////////////////////
void SimulationRunner::SetStepSize(const ignition::math::clock::duration &_step)
{
  this->stepSize = _step;
}

/////////////////////////////////////////////////
bool SimulationRunner::HasEntity(const std::string &_name) const
{
  bool result = false;
  this->entityCompMgr.Each<components::Name>([&](const EntityId,
        const components::Name *_entityName)->bool
    {
      if (_entityName->Data() == _name)
      {
        result = true;
        return false;
      }
      return true;
    });

  return result;
}

/////////////////////////////////////////////////
bool SimulationRunner::RequestEraseEntity(const std::string &_name)
{
  bool result = false;
  this->entityCompMgr.Each<components::Name>([&](const EntityId _id,
        const components::Name *_entityName)->bool
    {
      if (_entityName->Data() == _name)
      {
        this->entityCompMgr.RequestEraseEntity(_id);
        result = true;
        return false;
      }
      return true;
    });

  return result;
}

/////////////////////////////////////////////////
std::optional<EntityId> SimulationRunner::EntityByName(
    const std::string &_name) const
{
  std::optional<EntityId> id;
  this->entityCompMgr.Each<components::Name>([&](const EntityId _id,
        const components::Name *_entityName)->bool
    {
      if (_entityName->Data() == _name)
      {
        id = _id;
        return false;
      }
      return true;
    });

  return id;
}

/////////////////////////////////////////////////
bool SimulationRunner::RequestEraseEntity(const EntityId _id)
{
  if (this->entityCompMgr.HasEntity(_id))
  {
    this->entityCompMgr.RequestEraseEntity(_id);
    return true;
  }

  return false;
}
