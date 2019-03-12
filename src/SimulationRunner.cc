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

#include "ignition/common/Profiler.hh"

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

using namespace ignition;
using namespace gazebo;

using StringSet = std::unordered_set<std::string>;

//////////////////////////////////////////////////
SimulationRunner::SimulationRunner(const sdf::World *_world,
                                   const SystemLoaderPtr &_systemLoader,
                                   const ServerConfig &_config)
    // \todo(nkoenig) Either copy the world, or add copy constructor to the
    // World and other elements.
    : sdfWorld(_world), serverConfig(_config)
{
  if (nullptr == _world)
  {
    ignerr << "Can't start simulation runner with null world." << std::endl;
    return;
  }

  // Keep world name
  this->worldName = _world->Name();

  // Keep system loader so plugins can be loaded at runtime
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

  this->pauseConn = this->eventMgr.Connect<events::Pause>(
      std::bind(&SimulationRunner::SetPaused, this, std::placeholders::_1));

  this->stopConn = this->eventMgr.Connect<events::Stop>(
      std::bind(&SimulationRunner::OnStop, this));

  this->loadPluginsConn = this->eventMgr.Connect<events::LoadPlugins>(
      std::bind(&SimulationRunner::LoadPlugins, this, std::placeholders::_1,
      std::placeholders::_2));

  // Create the level manager
  this->levelMgr = std::make_unique<LevelManager>(
      this, _config.UseLevels(), _config.UseDistributedSimulation());

  // Check if this is going to be a distributed runner
  // Attempt to create the manager based on environment variables.
  // If the configuration is invalid, then networkMgr will be `nullptr`.
  if (_config.UseDistributedSimulation())
  {
    this->networkMgr = NetworkManager::Create(&this->eventMgr);

    if (this->networkMgr->IsPrimary())
    {
      ignmsg << "Network Primary, expects ["
             << this->networkMgr->Config().numSecondariesExpected
             << "] seondaries." << std::endl;
    }
    else if (this->networkMgr->IsSecondary())
    {
      ignmsg << "Network Secondary, with namespace ["
             << this->networkMgr->Namespace() << "]." << std::endl;
    }

    // Create the sync manager
    this->syncMgr = std::make_unique<SyncManager>(this);
  }

  // Load the active levels
  this->levelMgr->UpdateLevelsState();

  // World control
  transport::NodeOptions opts;
  if (this->networkMgr)
  {
    opts.SetNameSpace(this->networkMgr->Namespace() +
                      "/world/" + this->worldName);
  }
  else
  {
    opts.SetNameSpace("/world/" + this->worldName);
  }

  this->node = std::make_unique<transport::Node>(opts);

  this->node->Advertise("control", &SimulationRunner::OnWorldControl, this);

  // Publish empty GUI messages for worlds that have no GUI in the beginning.
  // In the future, support modifying GUI from the server at runtime.
  if (_world->Gui())
  {
    this->guiMsg = convert<msgs::GUI>(*_world->Gui());
  }

  std::string infoService{"gui/info"};
  this->node->Advertise(infoService, &SimulationRunner::GuiInfoService, this);

  ignmsg << "Serving GUI information on [" << opts.NameSpace() << "/"
         << infoService << "]" << std::endl;

  ignmsg << "World [" << _world->Name() << "] initialized with ["
         << physics->Name() << "] physics profile." << std::endl;
}

//////////////////////////////////////////////////
SimulationRunner::~SimulationRunner() = default;

/////////////////////////////////////////////////
void SimulationRunner::UpdateCurrentInfo()
{
  IGN_PROFILE("SimulationRunner::UpdateCurrentInfo");

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
  this->currentInfo.dt = std::chrono::steady_clock::duration::zero();

  // In the case that networking is not running, or this is a primary.
  // If this is a network secondary, this data is populated via the network.
  if (!this->currentInfo.paused &&
      (!this->networkMgr || this->networkMgr->IsPrimary()))
  {
    this->currentInfo.simTime += this->stepSize;
    ++this->currentInfo.iterations;
    this->currentInfo.dt = this->stepSize;
  }
}

/////////////////////////////////////////////////
void SimulationRunner::PublishStats()
{
  IGN_PROFILE("SimulationRunner::PublishStats");

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

  // Publish the stats message. The stats message is throttled.
  this->statsPub.Publish(msg);

  // Create and publish the clock message. The clock message is not
  // throttled.
  ignition::msgs::Clock clockMsg;
  clockMsg.mutable_real()->set_sec(realTimeSecNsec.first);
  clockMsg.mutable_real()->set_nsec(realTimeSecNsec.second);
  clockMsg.mutable_sim()->set_sec(simTimeSecNsec.first);
  clockMsg.mutable_sim()->set_nsec(simTimeSecNsec.second);
  clockMsg.mutable_system()->set_sec(IGN_SYSTEM_TIME_S());
  clockMsg.mutable_system()->set_nsec(
      IGN_SYSTEM_TIME_NS() - IGN_SYSTEM_TIME_S() * IGN_SEC_TO_NANO);
  this->clockPub.Publish(clockMsg);
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
  IGN_PROFILE("SimulationRunner::UpdateSystems");
  // \todo(nkoenig)  Systems used to be updated in parallel using
  // an ignition::common::WorkerPool. There is overhead associated with
  // this, most notably the creation and destruction of WorkOrders (see
  // WorkerPool.cc). We could turn on parallel updates in the future, and/or
  // turn it on if there are sufficient systems. More testing is required.

  {
    IGN_PROFILE("PreUpdate");
    for (auto& system : this->systemsPreupdate)
      system->PreUpdate(this->currentInfo, this->entityCompMgr);
  }

  {
    IGN_PROFILE("Update");
    for (auto& system : this->systemsUpdate)
      system->Update(this->currentInfo, this->entityCompMgr);
  }

  {
    IGN_PROFILE("PostUpdate");
    for (auto& system : this->systemsPostupdate)
      system->PostUpdate(this->currentInfo, this->entityCompMgr);
  }
}

/////////////////////////////////////////////////
void SimulationRunner::Stop()
{
  this->eventMgr.Emit<events::Stop>();
}

/////////////////////////////////////////////////
void SimulationRunner::OnStop()
{
  this->stopReceived = true;
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
  IGN_PROFILE_THREAD_NAME("SimulationRunner");

  // Initialize network communications.
  if (this->networkMgr)
  {
    // todo(mjcarroll) improve guard conditions around the busy loops.
    igndbg << "Initializing network configuration" << std::endl;
    this->networkMgr->Initialize();

    if (!this->stopReceived)
    {
      this->syncMgr->DistributePerformers();
    }
    else
    {
      this->running = false;
      return false;
    }
  }

  // Keep track of wall clock time. Only start the realTimeWatch if this
  // runner is not paused.
  if (!this->currentInfo.paused)
    this->realTimeWatch.Start();

  // Variables for time keeping.
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::duration sleepTime;
  std::chrono::steady_clock::duration actualSleep;

  this->running = true;

  // Create the world statistics publisher.
  if (!this->statsPub.Valid())
  {
    transport::AdvertiseMessageOptions advertOpts;
    advertOpts.SetMsgsPerSec(5);
    this->statsPub = this->node->Advertise<ignition::msgs::WorldStatistics>(
        "stats", advertOpts);
  }

  // Create the clock publisher.
  if (!this->clockPub.Valid())
    this->clockPub = this->node->Advertise<ignition::msgs::Clock>("clock");

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (uint64_t startingIterations = this->currentInfo.iterations;
       this->running && (_iterations == 0 ||
         this->currentInfo.iterations < _iterations + startingIterations);)
  {
    IGN_PROFILE("SimulationRunner::Run - Iteration");
    // Compute the time to sleep in order to match, as closely as possible,
    // the update period.
    sleepTime = 0ns;
    actualSleep = 0ns;

    if (!this->networkMgr || this->networkMgr->IsPrimary())
    {
      sleepTime = std::max(0ns, this->prevUpdateRealTime +
          this->updatePeriod - std::chrono::steady_clock::now() -
          this->sleepOffset);
    }

    // Only sleep if needed.
    if (sleepTime > 0ns)
    {
      IGN_PROFILE("Sleep");
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

    if (this->networkMgr)
    {
      IGN_PROFILE("NetworkSync - SendStep");
      // \todo(anyone) Replace busy loop with a condition.
      while (this->running && !this->networkMgr->Step(this->currentInfo))
      {
      }
    }

    // Publish info
    this->PublishStats();

    // Record when the update step starts.
    this->prevUpdateRealTime = std::chrono::steady_clock::now();

    this->levelMgr->UpdateLevelsState();

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

    // Clear all new entities
    this->entityCompMgr.ClearNewlyCreatedEntities();

    // Process entity removals.
    this->entityCompMgr.ProcessRemoveEntityRequests();


    if (this->networkMgr)
    {
      IGN_PROFILE("NetworkSync - SecondaryAck");

      this->syncMgr->Sync();

      // \todo(anyone) Replace busy loop with a condition.
      while (this->running && !this->networkMgr->StepAck(
            this->currentInfo.iterations))
      {
      }
    }
  }

  this->running = false;
  return true;
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugins(const Entity _entity,
    const sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
  while (pluginElem)
  {
    // No error message for the 'else' case of the following 'if' statement
    // because SDF create a default <plugin> element even if it's not
    // specified. An error message would result in spamming
    // the console. \todo(nkoenig) Fix SDF should so that elements are not
    // automatically added.
    if (pluginElem->Get<std::string>("filename") != "__default__" &&
        pluginElem->Get<std::string>("name") != "__default__")
    {
      std::optional<SystemPluginPtr> system =
        this->systemLoader->LoadPlugin(pluginElem);
      if (system)
      {
        auto systemConfig = system.value()->QueryInterface<ISystemConfigure>();
        if (systemConfig != nullptr)
        {
          systemConfig->Configure(_entity, pluginElem,
              this->entityCompMgr,
              this->eventMgr);
        }
        this->AddSystem(system.value());
      }
    }

    pluginElem = pluginElem->GetNextElement("plugin");
  }

  // \todo(nkoenig) Remove plugins from the server config after they have
  // been added. We might not want to do this if we want to support adding
  // the same plugin to multiple entities, for example via a regex
  // expression.
  //
  // Check plugins from the ServerConfig for matching entities.
  for (const ServerConfig::PluginInfo &plugin : this->serverConfig.Plugins())
  {
    // \todo(anyone) Type + name is not enough to uniquely identify an entity
    // \todo(louise) The runner shouldn't care about specific components, this
    // logic should be moved somewhere else.
    Entity entity{kNullEntity};

    if ("model" == plugin.EntityType())
    {
      entity = this->entityCompMgr.EntityByComponents(
          components::Name(plugin.EntityName()), components::Model());
    }
    else if ("world" == plugin.EntityType())
    {
      entity = this->entityCompMgr.EntityByComponents(
          components::Name(plugin.EntityName()), components::World());
    }
    else
    {
      ignwarn << "No support for attaching plugins to entity of type ["
              << plugin.EntityType() << "]" << std::endl;
    }

    // Skip plugins that do not match the provided entity
    if (entity != _entity)
      continue;

    std::optional<SystemPluginPtr> system =
      this->systemLoader->LoadPlugin(plugin.Filename(), plugin.Name(), nullptr);
    if (system)
    {
      auto systemConfig = system.value()->QueryInterface<ISystemConfigure>();
      if (systemConfig != nullptr)
      {
        systemConfig->Configure(_entity, plugin.Sdf(), this->entityCompMgr,
                                this->eventMgr);
      }
      this->AddSystem(system.value());
    }
  }
}

/////////////////////////////////////////////////
bool SimulationRunner::Running() const
{
  return this->running;
}

/////////////////////////////////////////////////
bool SimulationRunner::StopReceived() const
{
  return this->stopReceived;
}

/////////////////////////////////////////////////
bool SimulationRunner::Ready() const
{
  bool ready = true;
  if (this->networkMgr && !this->networkMgr->Ready())
  {
    ready = false;
  }
  return ready;
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
  IGN_PROFILE("SimulationRunner::ProcessMessages");
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);
  this->ProcessWorldControl();
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessWorldControl()
{
  IGN_PROFILE("SimulationRunner::ProcessWorldControl");
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
EventManager &SimulationRunner::EventMgr()
{
  return this->eventMgr;
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
  this->entityCompMgr.Each<components::Name>([&](const Entity,
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
bool SimulationRunner::RequestRemoveEntity(const std::string &_name,
    bool _recursive)
{
  bool result = false;
  this->entityCompMgr.Each<components::Name>([&](const Entity _entity,
        const components::Name *_entityName)->bool
    {
      if (_entityName->Data() == _name)
      {
        this->entityCompMgr.RequestRemoveEntity(_entity, _recursive);
        result = true;
        return false;
      }
      return true;
    });

  return result;
}

/////////////////////////////////////////////////
std::optional<Entity> SimulationRunner::EntityByName(
    const std::string &_name) const
{
  std::optional<Entity> entity;
  this->entityCompMgr.Each<components::Name>([&](const Entity _entity,
        const components::Name *_entityName)->bool
    {
      if (_entityName->Data() == _name)
      {
        entity = _entity;
        return false;
      }
      return true;
    });

  return entity;
}

/////////////////////////////////////////////////
bool SimulationRunner::RequestRemoveEntity(const Entity _entity,
    bool _recursive)
{
  if (this->entityCompMgr.HasEntity(_entity))
  {
    this->entityCompMgr.RequestRemoveEntity(_entity, _recursive);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool SimulationRunner::GuiInfoService(ignition::msgs::GUI &_res)
{
  _res.Clear();

  _res.CopyFrom(this->guiMsg);

  return true;
}
