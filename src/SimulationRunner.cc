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

#include <algorithm>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/gui.pb.h>
#include <gz/msgs/log_playback_control.pb.h>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_control_state.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <sdf/Root.hh>

#include "gz/common/Profiler.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/PhysicsCmd.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Util.hh"

#include "network/NetworkManagerPrimary.hh"
#include "SdfGenerator.hh"

using namespace gz;
using namespace sim;

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
    gzerr << "Can't start simulation runner with null world." << std::endl;
    return;
  }

  // Keep world name
  this->worldName = _world->Name();

  this->parametersRegistry = std::make_unique<
    gz::transport::parameters::ParametersRegistry>(
      std::string{"world/"} + this->worldName);

  // Get the physics profile
  // TODO(luca): remove duplicated logic in SdfEntityCreator and LevelManager
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
  this->desiredRtf = physics->RealTimeFactor();

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
  if (this->desiredRtf < 1e-9)
  {
    this->updatePeriod = 0ms;
  }
  else
  {
    this->updatePeriod = std::chrono::nanoseconds(
        static_cast<int>(this->stepSize.count() / this->desiredRtf));
  }

  // Epoch
  this->simTimeEpoch = std::chrono::round<std::chrono::nanoseconds>(
    std::chrono::duration<double>{_config.InitialSimTime()}
  );
  this->currentInfo.simTime = this->simTimeEpoch;

  // World control
  transport::NodeOptions opts;
  std::string ns{"/world/" + this->worldName};
  if (this->networkMgr)
  {
    ns = this->networkMgr->Namespace() + ns;
  }

  auto validNs = transport::TopicUtils::AsValidTopic(ns);
  if (validNs.empty())
  {
    gzerr << "Invalid namespace [" << ns
           << "], not initializing runner transport." << std::endl;
    return;
  }
  opts.SetNameSpace(validNs);

  this->node = std::make_unique<transport::Node>(opts);

  // Create the system manager
  this->systemMgr = std::make_unique<SystemManager>(
      _systemLoader, &this->entityCompMgr, &this->eventMgr, validNs,
      this->parametersRegistry.get());

  this->pauseConn = this->eventMgr.Connect<events::Pause>(
      std::bind(&SimulationRunner::SetPaused, this, std::placeholders::_1));

  this->stopConn = this->eventMgr.Connect<events::Stop>(
      std::bind(&SimulationRunner::OnStop, this));

  this->loadPluginsConn = this->eventMgr.Connect<events::LoadSdfPlugins>(
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
      /// \todo(nkoenig) Remove part of the 'if' statement in gz-sim3.
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
        gzmsg << "Network Primary, expects ["
          << this->networkMgr->Config().numSecondariesExpected
          << "] secondaries." << std::endl;
      }
      else if (this->networkMgr->IsSecondary())
      {
        gzmsg << "Network Secondary, with namespace ["
          << this->networkMgr->Namespace() << "]." << std::endl;
      }
    }
  }

  // Load the active levels
  this->levelMgr->UpdateLevelsState();

  // Store the initial state of the ECM;
  this->initialEntityCompMgr.CopyFrom(this->entityCompMgr);

  // Load any additional plugins from the Server Configuration
  this->LoadServerPlugins(this->serverConfig.Plugins());

  // If we have reached this point and no world systems have been loaded, then
  // load a default set of systems.
  if (this->systemMgr->TotalByEntity(
      worldEntity(this->entityCompMgr)).empty())
  {
    gzmsg << "No systems loaded from SDF, loading defaults" << std::endl;
    bool isPlayback = !this->serverConfig.LogPlaybackPath().empty();
    auto plugins = sim::loadPluginInfo(isPlayback);
    this->LoadServerPlugins(plugins);
  }

  this->LoadLoggingPlugins(this->serverConfig);

  // TODO(louise) Combine both messages into one.
  this->node->Advertise("control", &SimulationRunner::OnWorldControl, this);
  this->node->Advertise("control/state", &SimulationRunner::OnWorldControlState,
      this);
  this->node->Advertise("playback/control",
      &SimulationRunner::OnPlaybackControl, this);

  gzmsg << "Serving world controls on [" << opts.NameSpace()
         << "/control], [" << opts.NameSpace() << "/control/state] and ["
         << opts.NameSpace() << "/playback/control]" << std::endl;

  // Publish empty GUI messages for worlds that have no GUI in the beginning.
  // In the future, support modifying GUI from the server at runtime.
  if (_world->Gui())
  {
    this->guiMsg = convert<msgs::GUI>(*_world->Gui());
  }

  std::string infoService{"gui/info"};
  this->node->Advertise(infoService, &SimulationRunner::GuiInfoService, this);

  gzmsg << "Serving GUI information on [" << opts.NameSpace() << "/"
         << infoService << "]" << std::endl;

  gzmsg << "World [" << _world->Name() << "] initialized with ["
         << physics->Name() << "] physics profile." << std::endl;

  std::string genWorldSdfService{"generate_world_sdf"};
  this->node->Advertise(
      genWorldSdfService, &SimulationRunner::GenerateWorldSdf, this);

  gzmsg << "Serving world SDF generation service on [" << opts.NameSpace()
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
  GZ_PROFILE("SimulationRunner::UpdateCurrentInfo");

  // Rewind
  if (this->requestedRewind)
  {
    gzdbg << "Rewinding simulation back to initial time." << std::endl;
    this->realTimeFactor = 0;

    this->currentInfo.dt = this->simTimeEpoch - this->currentInfo.simTime;
    this->currentInfo.simTime = this->simTimeEpoch;
    this->currentInfo.realTime = std::chrono::steady_clock::duration::zero();
    this->currentInfo.iterations = 0;
    this->realTimeWatch.Reset();
    if (!this->currentInfo.paused)
      this->realTimeWatch.Start();
    this->resetInitiated = true;
    this->requestedRewind = false;

    return;
  }

  // Seek
  if (this->requestedSeek && this->requestedSeek.value() >= this->simTimeEpoch)
  {
    gzdbg << "Seeking to " << std::chrono::duration_cast<std::chrono::seconds>(
        this->requestedSeek.value()).count() << "s." << std::endl;

    this->realTimeFactor = 0;

    this->currentInfo.dt = this->requestedSeek.value() -
      this->currentInfo.simTime;
    this->currentInfo.simTime = this->requestedSeek.value();
    this->currentInfo.iterations = 0;

    this->currentInfo.realTime = this->realTimeWatch.ElapsedRunTime();

    this->requestedSeek = {};

    return;
  }

  // Regular time flow

  const double simTimeCount =
      static_cast<double>(this->currentInfo.simTime.count());
  const double realTimeCount =
      static_cast<double>(this->currentInfo.realTime.count());


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
  const double simTimeDiff =
      static_cast<double>(this->currentInfo.simTime.count()) - simTimeCount;
  const double realTimeDiff =
      static_cast<double>(this->currentInfo.realTime.count()) - realTimeCount;

  if (realTimeDiff > 0)
  {
    this->realTimeFactor = simTimeDiff / realTimeDiff;
  }
}

/////////////////////////////////////////////////
void SimulationRunner::UpdatePhysicsParams()
{
  auto worldEntity =
    this->entityCompMgr.EntityByComponents(components::World());
  const auto physicsCmdComp =
    this->entityCompMgr.Component<components::PhysicsCmd>(worldEntity);
  if (!physicsCmdComp)
  {
    return;
  }
  auto physicsComp =
    this->entityCompMgr.Component<components::Physics>(worldEntity);

  const auto& physicsParams = physicsCmdComp->Data();
  const auto newStepSize =
    std::chrono::duration<double>(physicsParams.max_step_size());
  const double newRTF = physicsParams.real_time_factor();

  const double eps = 0.00001;
  if (newStepSize != this->stepSize ||
      std::abs(newRTF - this->desiredRtf) > eps)
  {
    bool updated = false;
    // Make sure the values are valid before setting them
    if (newStepSize.count() > 0.0)
    {
      this->SetStepSize(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          newStepSize));
      physicsComp->Data().SetMaxStepSize(physicsParams.max_step_size());
      updated = true;
    }
    if (newRTF > 0.0)
    {
      this->desiredRtf = newRTF;
      this->updatePeriod = std::chrono::nanoseconds(
          static_cast<int>(this->stepSize.count() / this->desiredRtf));
      physicsComp->Data().SetRealTimeFactor(newRTF);
      updated = true;
    }
    if (updated)
    {
      // Set as OneTimeChange to make sure the update is not missed
      this->entityCompMgr.SetChanged(worldEntity, components::Physics::typeId,
          ComponentState::OneTimeChange);
    }
  }
  this->entityCompMgr.RemoveComponent<components::PhysicsCmd>(worldEntity);
}

/////////////////////////////////////////////////
void SimulationRunner::PublishStats()
{
  GZ_PROFILE("SimulationRunner::PublishStats");

  // Create the world statistics message.
  msgs::WorldStatistics msg;
  msg.set_real_time_factor(this->realTimeFactor);

  auto realTimeSecNsec =
    math::durationToSecNsec(this->currentInfo.realTime);

  auto simTimeSecNsec =
    math::durationToSecNsec(this->currentInfo.simTime);

  msg.mutable_real_time()->set_sec(realTimeSecNsec.first);
  msg.mutable_real_time()->set_nsec(realTimeSecNsec.second);

  msg.mutable_sim_time()->set_sec(simTimeSecNsec.first);
  msg.mutable_sim_time()->set_nsec(simTimeSecNsec.second);

  msg.set_iterations(this->currentInfo.iterations);

  msg.set_paused(this->currentInfo.paused);

  if (this->Stepping())
  {
    // (deprecated) Remove this header in Gazebo H
    auto headerData = msg.mutable_header()->add_data();
    headerData->set_key("step");

    msg.set_stepping(true);
  }

  // Publish the stats message. The stats message is throttled.
  this->statsPub.Publish(msg);

  if (this->rootStatsPub.Valid())
    this->rootStatsPub.Publish(msg);

  // Create and publish the clock message. The clock message is not
  // throttled.
  msgs::Clock clockMsg;
  clockMsg.mutable_real()->set_sec(realTimeSecNsec.first);
  clockMsg.mutable_real()->set_nsec(realTimeSecNsec.second);
  clockMsg.mutable_sim()->set_sec(simTimeSecNsec.first);
  clockMsg.mutable_sim()->set_nsec(simTimeSecNsec.second);
  clockMsg.mutable_system()->set_sec(GZ_SYSTEM_TIME_S());
  clockMsg.mutable_system()->set_nsec(
      GZ_SYSTEM_TIME_NS() - GZ_SYSTEM_TIME_S() * GZ_SEC_TO_NANO);
  this->clockPub.Publish(clockMsg);

  // Only publish to root topic if no others are.
  if (this->rootClockPub.Valid())
    this->rootClockPub.Publish(clockMsg);
}

namespace {

// Create an sdf::ElementPtr that corresponds to an empty `<plugin>` element.
sdf::ElementPtr createEmptyPluginElement()
{
  auto plugin = std::make_shared<sdf::Element>();
  sdf::initFile("plugin.sdf", plugin);
  return plugin;
}
}
//////////////////////////////////////////////////
void SimulationRunner::AddSystem(const SystemPluginPtr &_system,
      std::optional<Entity> _entity,
      std::optional<std::shared_ptr<const sdf::Element>> _sdf)
{
  auto entity = _entity.value_or(worldEntity(this->entityCompMgr));
  auto sdf = _sdf.value_or(createEmptyPluginElement());
  this->systemMgr->AddSystem(_system, entity, sdf);
}

//////////////////////////////////////////////////
void SimulationRunner::AddSystem(
      const std::shared_ptr<System> &_system,
      std::optional<Entity> _entity,
      std::optional<std::shared_ptr<const sdf::Element>> _sdf)
{
  auto entity = _entity.value_or(worldEntity(this->entityCompMgr));
  auto sdf = _sdf.value_or(createEmptyPluginElement());
  this->systemMgr->AddSystem(_system, entity, sdf);
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessSystemQueue()
{
  auto pending = this->systemMgr->PendingCount();

  if (0 == pending)
    return;

  // If additional systems are to be added, stop the existing threads.
  this->StopWorkerThreads();

  this->systemMgr->ActivatePendingSystems();

  unsigned int threadCount =
    static_cast<unsigned int>(this->systemMgr->SystemsPostUpdate().size() + 1u);

  gzdbg << "Creating PostUpdate worker threads: "
    << threadCount << std::endl;

  this->postUpdateStartBarrier = std::make_unique<Barrier>(threadCount);
  this->postUpdateStopBarrier = std::make_unique<Barrier>(threadCount);

  this->postUpdateThreadsRunning = true;
  int id = 0;

  for (auto &system : this->systemMgr->SystemsPostUpdate())
  {
    gzdbg << "Creating postupdate worker thread (" << id << ")" << std::endl;

    this->postUpdateThreads.push_back(std::thread([&, id]()
    {
      std::stringstream ss;
      ss << "PostUpdateThread: " << id;
      GZ_PROFILE_THREAD_NAME(ss.str().c_str());
      while (this->postUpdateThreadsRunning)
      {
        this->postUpdateStartBarrier->Wait();
        if (this->postUpdateThreadsRunning)
        {
          system->PostUpdate(this->currentInfo, this->entityCompMgr);
        }
        this->postUpdateStopBarrier->Wait();
      }
      gzdbg << "Exiting postupdate worker thread ("
        << id << ")" << std::endl;
    }));
    id++;
  }
}

/////////////////////////////////////////////////
void SimulationRunner::UpdateSystems()
{
  GZ_PROFILE("SimulationRunner::UpdateSystems");
  // \todo(nkoenig)  Systems used to be updated in parallel using
  // a common::WorkerPool. There is overhead associated with
  // this, most notably the creation and destruction of WorkOrders (see
  // WorkerPool.cc). We could turn on parallel updates in the future, and/or
  // turn it on if there are sufficient systems. More testing is required.

  if (this->resetInitiated)
  {
    GZ_PROFILE("Reset");
    this->systemMgr->Reset(this->currentInfo, this->entityCompMgr);
    return;
  }

  {
    GZ_PROFILE("PreUpdate");
    for (auto& system : this->systemMgr->SystemsPreUpdate())
      system->PreUpdate(this->currentInfo, this->entityCompMgr);
  }

  {
    GZ_PROFILE("Update");
    for (auto& system : this->systemMgr->SystemsUpdate())
      system->Update(this->currentInfo, this->entityCompMgr);
  }

  {
    GZ_PROFILE("PostUpdate");
    this->entityCompMgr.LockAddingEntitiesToViews(true);
    // If no systems implementing PostUpdate have been added, then
    // the barriers will be uninitialized, so guard against that condition.
    if (this->postUpdateStartBarrier && this->postUpdateStopBarrier)
    {
      this->postUpdateStartBarrier->Wait();
      this->postUpdateStopBarrier->Wait();
    }
    this->entityCompMgr.LockAddingEntitiesToViews(false);
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
  GZ_PROFILE_THREAD_NAME("SimulationRunner");

  // Initialize network communications.
  if (this->networkMgr)
  {
    gzdbg << "Initializing network configuration" << std::endl;
    this->networkMgr->Handshake();

    // Secondaries are stepped through the primary, just keep alive until
    // simulation is over
    if (this->networkMgr->IsSecondary())
    {
      gzdbg << "Secondary running." << std::endl;
      while (!this->stopReceived)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      gzdbg << "Secondary finished run." << std::endl;
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
    // publish 10 world statistics msgs/second. A smaller number isn't used
    // because the GUI listens to these msgs to receive confirmation that
    // pause/play GUI requests have been processed by the server, so we want to
    // make sure that GUI requests are acknowledged quickly (see
    // https://github.com/gazebosim/gz-gui/pull/306 and
    // https://github.com/gazebosim/gz-sim/pull/1163)
    advertOpts.SetMsgsPerSec(10);
    this->statsPub = this->node->Advertise<msgs::WorldStatistics>(
        "stats", advertOpts);
  }

  if (!this->rootStatsPub.Valid())
  {
    // Check for the existence of other publishers on `/stats`
    std::vector<transport::MessagePublisher> publishers;
    this->node->TopicInfo("/stats", publishers);

    if (!publishers.empty())
    {
      gzwarn << "Found additional publishers on /stats," <<
                 " using namespaced stats topic only" << std::endl;
      gzdbg << "Publishers [Address, Message Type]:\n";

      /// List the publishers
      for (auto & pub : publishers)
      {
        gzdbg << "  " << pub.Addr() << ", "
          << pub.MsgTypeName() << std::endl;
      }
    }
    else
    {
      gzmsg << "Found no publishers on /stats, adding root stats topic"
             << std::endl;
      this->rootStatsPub = this->node->Advertise<msgs::WorldStatistics>(
          "/stats");
    }
  }

  // Create the clock publisher.
  if (!this->clockPub.Valid())
    this->clockPub = this->node->Advertise<msgs::Clock>("clock");

  // Create the global clock publisher.
  if (!this->rootClockPub.Valid())
  {
    // Check for the existence of other publishers on `/clock`
    std::vector<transport::MessagePublisher> publishers;
    this->node->TopicInfo("/clock", publishers);

    if (!publishers.empty())
    {
      gzwarn << "Found additional publishers on /clock," <<
                 " using namespaced clock topic only" << std::endl;
      gzdbg << "Publishers [Address, Message Type]:\n";

      /// List the publishers
      for (auto & pub : publishers)
      {
        gzdbg << "  " << pub.Addr() << ", "
          << pub.MsgTypeName() << std::endl;
      }
    }
    else
    {
      gzmsg << "Found no publishers on /clock, adding root clock topic"
             << std::endl;
      this->rootClockPub = this->node->Advertise<msgs::Clock>(
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
    GZ_PROFILE("SimulationRunner::Run - Iteration");

    // Update the step size and desired rtf
    this->UpdatePhysicsParams();

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
      GZ_PROFILE("Sleep");
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
    if (this->resetInitiated)
    {
      this->entityCompMgr.ResetTo(this->initialEntityCompMgr);
    }

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

    this->resetInitiated = false;
  }

  this->running = false;

  return true;
}

/////////////////////////////////////////////////
void SimulationRunner::Step(const UpdateInfo &_info)
{
  GZ_PROFILE("SimulationRunner::Step");
  this->currentInfo = _info;

  // Process new ECM state information, typically sent from the GUI after
  // a change was made to the GUI's ECM.
  this->ProcessNewWorldControlState();

  // Publish info
  this->PublishStats();

  // Record when the update step starts.
  this->prevUpdateRealTime = std::chrono::steady_clock::now();

  this->levelMgr->UpdateLevelsState();

  // Handle pending systems
  this->ProcessSystemQueue();

  // Handle entities that need to be recreated.
  // Put in a request to mark them as removed so that in the UpdateSystem call
  // the systems can remove them first before new ones are created. This is
  // so that we can recreate entities with the same name.
  this->ProcessRecreateEntitiesRemove();

  // handle systems that need to be added
  this->systemMgr->ProcessPendingEntitySystems();

  // Update all the systems.
  this->UpdateSystems();

  if (!this->Paused() && this->requestedRunToSimTime &&
       this->requestedRunToSimTime.value() > this->simTimeEpoch &&
       this->currentInfo.simTime >= this->requestedRunToSimTime.value())
  {
    this->SetPaused(true);
    this->requestedRunToSimTime = {};
  }

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

  // Recreate any entities that have the Recreate component
  // The entities will have different Entity ids but keep the same name
  // Make sure this happens after ClearNewlyCreatedEntities, otherwise the
  // cloned entities will loose their "New" state.
  this->ProcessRecreateEntitiesCreate();

  // Process entity removals.
  this->entityCompMgr.ProcessRemoveEntityRequests();

  // Process components removals
  this->entityCompMgr.ClearRemovedComponents();

  // Each network manager takes care of marking its components as unchanged
  if (!this->networkMgr)
    this->entityCompMgr.SetAllComponentsUnchanged();
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugin(const Entity _entity,
                                  const sdf::Plugin &_plugin)
{
  this->systemMgr->LoadPlugin(_entity, _plugin);
}

//////////////////////////////////////////////////
void SimulationRunner::LoadServerPlugins(
    const std::list<ServerConfig::PluginInfo> &_plugins)
{
  // \todo(nkoenig) Remove plugins from the server config after they have
  // been added. We might not want to do this if we want to support adding
  // the same plugin to multiple entities, for example via a regex
  // expression.
  //
  // Check plugins from the ServerConfig for matching entities.

  for (const ServerConfig::PluginInfo &plugin : _plugins)
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
      gzwarn << "No support for attaching plugins to entity of type ["
              << plugin.EntityType() << "]" << std::endl;
    }


    if (kNullEntity != entity)
    {
      this->LoadPlugin(entity, plugin.Plugin());
    }
  }
}

//////////////////////////////////////////////////
void SimulationRunner::LoadLoggingPlugins(const ServerConfig &_config)
{
  std::list<ServerConfig::PluginInfo> plugins;

  if (_config.UseLogRecord() && !_config.LogPlaybackPath().empty())
  {
    gzwarn <<
      "Both recording and playback are specified, defaulting to playback\n";
  }

  if (!_config.LogPlaybackPath().empty())
  {
    auto playbackPlugin = _config.LogPlaybackPlugin();
    plugins.push_back(playbackPlugin);
  }
  else if (_config.UseLogRecord())
  {
    auto recordPlugin = _config.LogRecordPlugin();
    plugins.push_back(recordPlugin);
  }

  this->LoadServerPlugins(plugins);
}

//////////////////////////////////////////////////
void SimulationRunner::LoadPlugins(const Entity _entity,
    const sdf::Plugins &_plugins)
{
  for (const sdf::Plugin &plugin : _plugins)
  {
    // No error message for the 'else' case of the following 'if' statement
    // because SDF create a default <plugin> element even if it's not
    // specified. An error message would result in spamming
    // the console.
    if (plugin.Filename() != "__default__" && plugin.Name() != "__default__")
    {
      this->LoadPlugin(_entity, plugin);
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
  return this->systemMgr->TotalCount();
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
void SimulationRunner::SetStepping(bool _stepping)
{
  this->stepping = _stepping;
}

/////////////////////////////////////////////////
bool SimulationRunner::Stepping() const
{
  return this->stepping;
}

/////////////////////////////////////////////////
void SimulationRunner::SetRunToSimTime(
    const std::chrono::steady_clock::duration &_time)
{
  if (_time >= this->simTimeEpoch && _time > this->currentInfo.simTime)
  {
    this->requestedRunToSimTime = _time;
  }
  else
  {
    this->requestedRunToSimTime = {};
  }
}

/////////////////////////////////////////////////
bool SimulationRunner::OnWorldControl(const msgs::WorldControl &_req,
    msgs::Boolean &_res)
{
  msgs::WorldControlState req;
  req.mutable_world_control()->CopyFrom(_req);

  return this->OnWorldControlState(req, _res);
}

/////////////////////////////////////////////////
bool SimulationRunner::OnWorldControlState(const msgs::WorldControlState &_req,
    msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);

  // Copy the state information if it exists
  if (_req.has_state())
  {
    if (this->newWorldControlState == nullptr)
      this->newWorldControlState.reset(_req.New());
    this->newWorldControlState->CopyFrom(_req);
  }

  WorldControl control;
  control.pause = _req.world_control().pause();

  if (_req.world_control().multi_step() != 0)
    control.multiStep = _req.world_control().multi_step();
  else if (_req.world_control().step())
    control.multiStep = 1;

  if (_req.world_control().has_reset())
  {
    control.rewind = _req.world_control().reset().all() ||
      _req.world_control().reset().time_only();

    if (_req.world_control().reset().model_only())
    {
      gzwarn << "Model only reset is not supported." << std::endl;
    }
  }

  if (_req.world_control().seed() != 0)
  {
    gzwarn << "Changing seed is not supported." << std::endl;
  }

  if (_req.world_control().has_run_to_sim_time())
  {
    control.runToSimTime = std::chrono::seconds(
        _req.world_control().run_to_sim_time().sec()) +
      std::chrono::nanoseconds(_req.world_control().run_to_sim_time().nsec());
  }

  this->worldControls.push_back(control);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessNewWorldControlState()
{
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);
  // update the server ECM if the request contains SerializedState information
  if (this->newWorldControlState && this->newWorldControlState->has_state())
  {
    this->entityCompMgr.SetState(this->newWorldControlState->state());

    this->newWorldControlState.reset();
  }
  // TODO(anyone) notify server systems of changes made to the ECM, if there
  // were any?
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
    gzwarn << "Log forwarding is not supported, use seek." << std::endl;
  }

  this->worldControls.push_back(control);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessMessages()
{
  GZ_PROFILE("SimulationRunner::ProcessMessages");
  std::lock_guard<std::mutex> lock(this->msgBufferMutex);
  this->ProcessWorldControl();
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessWorldControl()
{
  GZ_PROFILE("SimulationRunner::ProcessWorldControl");

  // assume no stepping unless WorldControl msgs say otherwise
  this->SetStepping(false);

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
      this->SetStepping(true);
    }

    // Rewind / reset
    this->requestedRewind = control.rewind;

    // Seek
    if (control.seek >= this->simTimeEpoch)
    {
      this->requestedSeek = control.seek;
    }

    this->SetRunToSimTime(control.runToSimTime);
  }

  this->worldControls.clear();
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessRecreateEntitiesRemove()
{
  GZ_PROFILE("SimulationRunner::ProcessRecreateEntitiesRemove");

  // store the original entities to recreate and put in request to remove them
  this->entityCompMgr.EachNoCache<components::Model,
                           components::Recreate>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Recreate *)->bool
      {
        this->entitiesToRecreate.insert(_entity);
        this->entityCompMgr.RequestRemoveEntity(_entity, true);
        return true;
      });
}

/////////////////////////////////////////////////
void SimulationRunner::ProcessRecreateEntitiesCreate()
{
  GZ_PROFILE("SimulationRunner::ProcessRecreateEntitiesCreate");

  // clone the original entities
  for (auto & ent : this->entitiesToRecreate)
  {
    auto nameComp = this->entityCompMgr.Component<components::Name>(ent);
    auto parentComp =
        this->entityCompMgr.Component<components::ParentEntity>(ent);
    if (nameComp  && parentComp)
    {
      // set allowRenaming to false so the entities keep their original name
      Entity clonedEntity = this->entityCompMgr.Clone(ent,
         parentComp->Data(), nameComp->Data(), false);

      // remove the Recreate component so they do not get recreated again in the
      // next iteration
      if (!this->entityCompMgr.RemoveComponent<components::Recreate>(ent))
      {
        gzerr << "Failed to remove Recreate component from entity["
          << ent << "]" << std::endl;
      }

      if (!this->entityCompMgr.RemoveComponent<components::Recreate>(
            clonedEntity))
      {
        gzerr << "Failed to remove Recreate component from entity["
          << clonedEntity << "]" << std::endl;
      }
    }
    else if (!nameComp)
    {
      gzerr << "Missing name component for entity[" << ent << "]. "
        << "The entity will not be cloned during the recreation process."
        << std::endl;
    }
    else if (!parentComp)
    {
      gzerr << "Missing parent component for entity[" << ent << "]. "
        << "The entity will not be cloned during the recreation process."
         << std::endl;
    }
  }

  this->entitiesToRecreate.clear();
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
  SimulationRunner::SimTimeEpoch() const
{
  return this->simTimeEpoch;
}

/////////////////////////////////////////////////
const std::chrono::steady_clock::duration &
SimulationRunner::UpdatePeriod() const
{
  return this->updatePeriod;
}

/////////////////////////////////////////////////
const std::chrono::steady_clock::duration &SimulationRunner::StepSize() const
{
  return this->stepSize;
}

/////////////////////////////////////////////////
void SimulationRunner::SetStepSize(
    const std::chrono::steady_clock::duration &_step)
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
bool SimulationRunner::GuiInfoService(msgs::GUI &_res)
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
