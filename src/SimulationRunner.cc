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

#include <sdf/Root.hh>

#include "ignition/common/Profiler.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/Util.hh"
#include "network/NetworkManagerPrimary.hh"
#include "SdfGenerator.hh"

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
  this->levelMgr = std::make_unique<LevelManager>(this, _config.UseLevels());

  // Check if this is going to be a distributed runner
  // Attempt to create the manager based on environment variables.
  // If the configuration is invalid, then networkMgr will be `nullptr`.
  if (_config.UseDistributedSimulation())
  {
    if (_config.NetworkRole().empty())
    {
      /// \todo(nkoenig) Remove part of the 'if' statement in ign-gazebo3.
      this->networkMgr = NetworkManager::Create(
          std::bind(&SimulationRunner::Step, this, std::placeholders::_1),
          this->entityCompMgr, &this->eventMgr);
    }
    else
    {
      this->networkMgr = NetworkManager::Create(
          std::bind(&SimulationRunner::Step, this, std::placeholders::_1),
          this->entityCompMgr, &this->eventMgr,
          NetworkConfig::FromValues(
            _config.NetworkRole(), _config.NetworkSecondaries()));
    }

    if (this->networkMgr)
    {
      if (this->networkMgr->IsPrimary())
      {
        ignmsg << "Network Primary, expects ["
          << this->networkMgr->Config().numSecondariesExpected
          << "] secondaries." << std::endl;
      }
      else if (this->networkMgr->IsSecondary())
      {
        ignmsg << "Network Secondary, with namespace ["
          << this->networkMgr->Namespace() << "]." << std::endl;
      }
    }
  }

  // Load the active levels
  this->levelMgr->UpdateLevelsState();

  // Load any additional plugins from the Server Configuration
  this->LoadServerPlugins(this->serverConfig);

  // If we have reached this point and no systems have been loaded, then load
  // a default set of systems.
  // Order:
  // 1. File at IGN_GAZEBO_DEFAULT_SYSTEMS
  // 2. File at ${IGN_HOMEDIR}/.ignition/gazebo/systems.config
  // 3. File at ${IGN_DATA_INSTALL_DIR}/systems/systems.config

  if (this->systems.empty() && this->pendingSystems.empty())
  {
    ignmsg << "No systems loaded, loading defaults" << std::endl;

    std::string envConfig;
    ignition::common::env("IGN_GAZEBO_DEFAULT_SYSTEMS", envConfig);

    std::string defaultConfig;
    ignition::common::env(IGN_HOMEDIR, defaultConfig);
    defaultConfig = ignition::common::joinPaths(defaultConfig, ".ignition",
      "gazebo", "systems.config");

    auto installedConfig = ignition::common::joinPaths(
        IGNITION_GAZEBO_SYSTEM_CONFIG_PATH, "systems.config");

    std::string configPath;
    if (envConfig.size() && ignition::common::exists(envConfig))
    {
      configPath = envConfig;
    }
    else if (ignition::common::exists(defaultConfig))
    {
      configPath = defaultConfig;
    }
    else
    {
      configPath = installedConfig;
    }

    igndbg << "Loading default plugins from: " << configPath << std::endl;

    ServerConfig tmpConfig;
    auto plugins = ignition::gazebo::ParsePluginsFromFile(configPath);
    for (auto plugin : plugins)
    {
      tmpConfig.AddPlugin(plugin);
    }
    this->LoadServerPlugins(tmpConfig);
  }

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

  // TODO(louise) Combine both messages into one.
  this->node->Advertise("control", &SimulationRunner::OnWorldControl, this);
  this->node->Advertise("playback/control",
      &SimulationRunner::OnPlaybackControl, this);

  ignmsg << "Serving world controls on [" << opts.NameSpace()
         << "/control] and [" << opts.NameSpace() << "/playback/control]"
         << std::endl;

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

  std::string genWorldSdfService{"generate_world_sdf"};
  this->node->Advertise(
      genWorldSdfService, &SimulationRunner::GenerateWorldSdf, this);

  ignmsg << "Serving world SDF generation service on [" << opts.NameSpace()
         << "/" << genWorldSdfService << "]" << std::endl;
}

//////////////////////////////////////////////////
SimulationRunner::~SimulationRunner()
{
  this->StopWorkerThreads();
}

/////////////////////////////////////////////////
void SimulationRunner::UpdateCurrentInfo()
{
  IGN_PROFILE("SimulationRunner::UpdateCurrentInfo");

  // Rewind
  if (this->requestedRewind)
  {
    igndbg << "Rewinding simulation back to time zero." << std::endl;
    this->realTimes.clear();
    this->simTimes.clear();
    this->realTimeFactor = 0;

    this->currentInfo.dt = -this->currentInfo.simTime;
    this->currentInfo.simTime = std::chrono::steady_clock::duration::zero();
    this->currentInfo.realTime = std::chrono::steady_clock::duration::zero();
    this->currentInfo.iterations = 0;
    this->realTimeWatch.Reset();
    if (!this->currentInfo.paused)
      this->realTimeWatch.Start();

    this->requestedRewind = false;

    return;
  }

  // Seek
  if (this->requestedSeek >= std::chrono::steady_clock::duration::zero())
  {
    igndbg << "Seeking to " << std::chrono::duration_cast<std::chrono::seconds>(
        this->requestedSeek).count() << "s." << std::endl;

    this->realTimes.clear();
    this->simTimes.clear();
    this->realTimeFactor = 0;

    this->currentInfo.dt = this->requestedSeek - this->currentInfo.simTime;
    this->currentInfo.simTime = this->requestedSeek;
    this->currentInfo.iterations = 0;

    this->currentInfo.realTime = this->realTimeWatch.ElapsedRunTime();

    this->requestedSeek = std::chrono::steady_clock::duration{-1};

    return;
  }

  // Regular time flow

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

  if (this->rootStatsPub.Valid())
    this->rootStatsPub.Publish(msg);

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

  // Only publish to root topic if no others are.
  if (this->rootClockPub.Valid())
    this->rootClockPub.Publish(clockMsg);
}

/////////////////////////////////////////////////
void SimulationRunner::AddSystem(const SystemPluginPtr &_system)
{
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
  this->pendingSystems.push_back(_system);
}

/////////////////////////////////////////////////
void SimulationRunner::AddSystemToRunner(const SystemPluginPtr &_system)
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
void SimulationRunner::ProcessSystemQueue()
{
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
  auto pending = this->pendingSystems.size();

  if (pending > 0)
  {
    // If additional systems are to be added, stop the existing threads.
    this->StopWorkerThreads();
  }

  for (const auto &system : this->pendingSystems)
  {
    this->AddSystemToRunner(system);
  }

  this->pendingSystems.clear();

  // If additional systems were added, recreate the worker threads.
  if (pending > 0)
  {
    igndbg << "Creating PostUpdate worker threads: "
      << this->systemsPostupdate.size() + 1 << std::endl;

    this->postUpdateStartBarrier =
      std::make_unique<Barrier>(this->systemsPostupdate.size() + 1);
    this->postUpdateStopBarrier =
      std::make_unique<Barrier>(this->systemsPostupdate.size() + 1);

    this->postUpdateThreadsRunning = true;
    int id = 0;

    for (auto &system : this->systemsPostupdate)
    {
      igndbg << "Creating postupdate worker thread (" << id << ")" << std::endl;

      this->postUpdateThreads.push_back(std::thread([&, id]()
      {
        std::stringstream ss;
        ss << "PostUpdateThread: " << id;
        IGN_PROFILE_THREAD_NAME(ss.str().c_str());
        while (this->postUpdateThreadsRunning)
        {
          this->postUpdateStartBarrier->Wait();
          if (this->postUpdateThreadsRunning)
          {
            system->PostUpdate(this->currentInfo, this->entityCompMgr);
          }
          this->postUpdateStopBarrier->Wait();
        }
        igndbg << "Exiting postupdate worker thread ("
          << id << ")" << std::endl;
      }));
      id++;
    }
  }
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
    // If no systems implementing PostUpdate have been added, then
    // the barriers will be uninitialized, so guard against that condition.
    if (this->postUpdateStartBarrier && this->postUpdateStopBarrier)
    {
      this->postUpdateStartBarrier->Wait();
      this->postUpdateStopBarrier->Wait();
    }
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
void SimulationRunner::StopWorkerThreads()
{
  this->postUpdateThreadsRunning = false;
  if (this->postUpdateStartBarrier)
  {
    this->postUpdateStartBarrier->Cancel();
  }
  if (this->postUpdateStopBarrier)
  {
    this->postUpdateStopBarrier->Cancel();
  }
  for (auto &thread : this->postUpdateThreads)
  {
    thread.join();
  }
  this->postUpdateThreads.clear();
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
    igndbg << "Initializing network configuration" << std::endl;
    this->networkMgr->Handshake();

    // Secondaries are stepped through the primary, just keep alive until
    // simulation is over
    if (this->networkMgr->IsSecondary())
    {
      igndbg << "Secondary running." << std::endl;
      while (!this->stopReceived)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      igndbg << "Secondary finished run." << std::endl;
      return true;
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

  if (!this->rootStatsPub.Valid())
  {
    // Check for the existence of other publishers on `/stats`
    std::vector<ignition::transport::MessagePublisher> publishers;
    this->node->TopicInfo("/stats", publishers);

    if (!publishers.empty())
    {
      ignwarn << "Found additional publishers on /stats," <<
                 " using namespaced stats topic only" << std::endl;
      igndbg << "Publishers [Address, Message Type]:\n";

      /// List the publishers
      for (auto & pub : publishers)
      {
        igndbg << "  " << pub.Addr() << ", "
          << pub.MsgTypeName() << std::endl;
      }
    }
    else
    {
      ignmsg << "Found no publishers on /stats, adding root stats topic"
             << std::endl;
      this->rootStatsPub = this->node->Advertise<msgs::WorldStatistics>(
          "/stats");
    }
  }

  // Create the clock publisher.
  if (!this->clockPub.Valid())
    this->clockPub = this->node->Advertise<ignition::msgs::Clock>("clock");

  // Create the global clock publisher.
  if (!this->rootClockPub.Valid())
  {
    // Check for the existence of other publishers on `/clock`
    std::vector<ignition::transport::MessagePublisher> publishers;
    this->node->TopicInfo("/clock", publishers);

    if (!publishers.empty())
    {
      ignwarn << "Found additional publishers on /clock," <<
                 " using namespaced clock topic only" << std::endl;
      igndbg << "Publishers [Address, Message Type]:\n";

      /// List the publishers
      for (auto & pub : publishers)
      {
        igndbg << "  " << pub.Addr() << ", "
          << pub.MsgTypeName() << std::endl;
      }
    }
    else
    {
      ignmsg << "Found no publishers on /clock, adding root clock topic"
             << std::endl;
      this->rootClockPub = this->node->Advertise<ignition::msgs::Clock>(
          "/clock");
    }
  }

  // Keep number of iterations requested by caller
  uint64_t processedIterations{0};

  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  while (this->running && (_iterations == 0 ||
       processedIterations < _iterations))
  {
    IGN_PROFILE("SimulationRunner::Run - Iteration");
    // Compute the time to sleep in order to match, as closely as possible,
    // the update period.
    sleepTime = 0ns;
    actualSleep = 0ns;

    sleepTime = std::max(0ns, this->prevUpdateRealTime +
        this->updatePeriod - std::chrono::steady_clock::now() -
        this->sleepOffset);

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
    if (!this->currentInfo.paused)
    {
      processedIterations++;
    }

    // If network, wait for network step, otherwise do our own step
    if (this->networkMgr)
    {
      auto netPrimary =
          dynamic_cast<NetworkManagerPrimary *>(this->networkMgr.get());
      netPrimary->Step(this->currentInfo);
    }
    else
    {
      this->Step(this->currentInfo);
    }

    // Handle Server::RunOnce(false) in which a single paused run is executed
    if (this->currentInfo.paused && this->blockingPausedStepPending)
    {
      processedIterations++;
      this->currentInfo.iterations++;
      this->blockingPausedStepPending = false;
    }
  }

  this->running = false;

  return true;
}

/////////////////////////////////////////////////
void SimulationRunner::Step(const UpdateInfo &_info)
{
  IGN_PROFILE("SimulationRunner::Step");
  this->currentInfo = _info;

  // Publish info
  this->PublishStats();

  // Record when the update step starts.
  this->prevUpdateRealTime = std::chrono::steady_clock::now();

  this->levelMgr->UpdateLevelsState();

  // Handle pending systems
  this->ProcessSystemQueue();

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

  // Each network manager takes care of marking its components as unchanged
  if (!this->networkMgr)
    this->entityCompMgr.SetAllComponentsUnchanged();
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugin(const Entity _entity,
                                  const std::string &_fname,
                                  const std::string &_name,
                                  const sdf::ElementPtr &_sdf)
{
  std::optional<SystemPluginPtr> system;
  {
    std::lock_guard<std::mutex> lock(this->systemLoaderMutex);
    system = this->systemLoader->LoadPlugin(_fname, _name, _sdf);
  }

  // System correctly loaded from library, try to configure
  if (system)
  {
    auto systemConfig = system.value()->QueryInterface<ISystemConfigure>();
    if (systemConfig != nullptr)
    {
      systemConfig->Configure(_entity, _sdf,
          this->entityCompMgr,
          this->eventMgr);
    }
    this->AddSystem(system.value());
    igndbg << "Loaded system [" << _name
           << "] for entity [" << _entity << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void SimulationRunner::LoadServerPlugins(const ServerConfig &_config)
{
  // \todo(nkoenig) Remove plugins from the server config after they have
  // been added. We might not want to do this if we want to support adding
  // the same plugin to multiple entities, for example via a regex
  // expression.
  //
  // Check plugins from the ServerConfig for matching entities.
  for (const ServerConfig::PluginInfo &plugin : _config.Plugins())
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
      // Allow wildcard for world name
      if (plugin.EntityName() == "*")
      {
        entity = this->entityCompMgr.EntityByComponents(components::World());
      }
      else
      {
        entity = this->entityCompMgr.EntityByComponents(
            components::Name(plugin.EntityName()), components::World());
      }
    }
    else if ("sensor" == plugin.EntityType())
    {
      // TODO(louise) Use scoped names for models and worlds too
      auto sensors = this->entityCompMgr.EntitiesByComponents(
          components::Sensor());

      for (auto sensor : sensors)
      {
        if (scopedName(sensor, this->entityCompMgr, "::", false) ==
            plugin.EntityName())
        {
          entity = sensor;
          break;
        }
      }
    }
    else if ("visual" == plugin.EntityType())
    {
      // TODO(anyone) Use scoped names for models and worlds too
      auto visuals = this->entityCompMgr.EntitiesByComponents(
          components::Visual());

      for (auto visual : visuals)
      {
        if (scopedName(visual, this->entityCompMgr, "::", false) ==
            plugin.EntityName())
        {
          entity = visual;
          break;
        }
      }
    }
    else
    {
      ignwarn << "No support for attaching plugins to entity of type ["
              << plugin.EntityType() << "]" << std::endl;
    }

    this->LoadPlugin(entity, plugin.Filename(), plugin.Name(), plugin.Sdf());
  }
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugins(const Entity _entity,
    const sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
  while (pluginElem)
  {
    auto filename = pluginElem->Get<std::string>("filename");
    auto name = pluginElem->Get<std::string>("name");
    // No error message for the 'else' case of the following 'if' statement
    // because SDF create a default <plugin> element even if it's not
    // specified. An error message would result in spamming
    // the console. \todo(nkoenig) Fix SDF should so that elements are not
    // automatically added.
    if (filename != "__default__" && name != "__default__")
    {
      this->LoadPlugin(_entity, filename, name, pluginElem);
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
  std::lock_guard<std::mutex> lock(this->pendingSystemsMutex);
  return this->systems.size() + this->pendingSystems.size();
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

  WorldControl control;
  control.pause = _req.pause();

  if (_req.multi_step() != 0)
    control.multiStep = _req.multi_step();
  else if (_req.step())
    control.multiStep = 1;

  if (_req.has_reset())
  {
    control.rewind = _req.reset().all() || _req.reset().time_only();

    if (_req.reset().model_only())
    {
      ignwarn << "Model only reset is not supported." << std::endl;
    }
  }

  if (_req.seed() != 0)
  {
    ignwarn << "Changing seed is not supported." << std::endl;
  }

  this->worldControls.push_back(control);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool SimulationRunner::OnPlaybackControl(const msgs::LogPlaybackControl &_req,
    msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);

  WorldControl control;
  control.pause = _req.pause();
  control.multiStep = _req.multi_step();
  control.rewind = _req.rewind();

  if (_req.has_seek())
  {
    control.seek = std::chrono::seconds(_req.seek().sec()) +
                   std::chrono::nanoseconds(_req.seek().nsec());
  }

  if (_req.forward())
  {
    ignwarn << "Log forwarding is not supported, use seek." << std::endl;
  }

  this->worldControls.push_back(control);

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
  for (const auto &control : this->worldControls)
  {
    // Play / pause
    this->SetPaused(control.pause);

    // Step, only if we are paused.
    if (this->Paused() && control.multiStep > 0)
    {
      this->pendingSimIterations += control.multiStep;
      // Unpause so that stepping can occur.
      this->SetPaused(false);
    }

    // Rewind / reset
    this->requestedRewind = control.rewind;

    // Seek
    if (control.seek >= std::chrono::steady_clock::duration::zero())
    {
      this->requestedSeek = control.seek;
    }
  }

  this->worldControls.clear();
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

//////////////////////////////////////////////////
bool SimulationRunner::GenerateWorldSdf(const msgs::SdfGeneratorConfig &_req,
                                        msgs::StringMsg &_res)
{
  // TODO(addisu) This is not thread-safe. Wait until it is safe to access the
  // ECM.
  Entity world = this->entityCompMgr.EntityByComponents(components::World());
  std::optional<std::string> genString = sdf_generator::generateWorld(
      this->entityCompMgr, world, this->fuelUriMap, _req);
  if (genString.has_value())
  {
    _res.set_data(*genString);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
void SimulationRunner::SetFuelUriMap(
    const std::unordered_map<std::string, std::string> &_map)
{
  this->fuelUriMap = _map;
}

//////////////////////////////////////////////////
void SimulationRunner::AddToFuelUriMap(const std::string &_path,
                                       const std::string &_uri)
{
  this->fuelUriMap[_path] = _uri;
}

//////////////////////////////////////////////////
bool SimulationRunner::NextStepIsBlockingPaused() const
{
  return this->blockingPausedStepPending;
}

//////////////////////////////////////////////////
void SimulationRunner::SetNextStepAsBlockingPaused(const bool value)
{
  this->blockingPausedStepPending = value;
}
