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

#include "Sensors.hh"

#include <atomic>
#include <chrono>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>

#include <gz/rendering/Scene.hh>
#include <gz/sensors/BoundingBoxCameraSensor.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <gz/sensors/RgbdCameraSensor.hh>
#include <gz/sensors/ThermalCameraSensor.hh>
#include <gz/sensors/SegmentationCameraSensor.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/WideAngleCameraSensor.hh>
#include <gz/sensors/Manager.hh>

#include "gz/sim/components/Atmosphere.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BoundingBoxCamera.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/RenderEngineServerApiBackend.hh"
#include "gz/sim/components/RenderEngineServerHeadless.hh"
#include "gz/sim/components/RenderEngineServerPlugin.hh"
#include "gz/sim/components/RgbdCamera.hh"
#include "gz/sim/components/SegmentationCamera.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/WideAngleCamera.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "gz/sim/rendering/Events.hh"
#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/rendering/GlobalIlluminationVct.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief GI parameters holding default data
struct GiDefaultData
{
  /// \brief See rendering::GlobalIlluminationVct::SetResolution
  math::Vector3d resolution{16, 16, 16};

  /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
  math::Vector3d octantCount{1, 1, 1};

  /// \brief See rendering::GlobalIlluminationVct::SetBounceCount
  uint32_t bounceCount = 6;

  /// \brief See rendering::GlobalIlluminationVct::SetHighQuality
  bool highQuality = true;

  /// \brief See rendering::GlobalIlluminationVct::SetAnisotropic
  bool anisotropic = true;

  /// \brief See rendering::GlobalIlluminationVct::SetThinWallCounter
  float thinWallCounter = 1.0f;

  /// \brief See rendering::GlobalIlluminationVct::SetConserveMemory
  bool conserveMemory = false;

  /// \brief See rendering::GlobalIlluminationVct::DebugVisualizationMode
  uint32_t debugVisMode = rendering::GlobalIlluminationVct::DVM_None;
};

/// \brief GI VCT flag and parameters
struct GiVctParameters
{
  /// \brief VCT enabled flag
  bool enabled = false;

  /// \brief See rendering::GlobalIlluminationVct::SetResolution
  uint32_t resolution[3];

  /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
  uint32_t octantCount[3];

  /// \brief See rendering::GlobalIlluminationVct::SetBounceCount
  uint32_t bounceCount;

  /// \brief See rendering::GlobalIlluminationVct::SetHighQuality
  bool highQuality;

  /// \brief See rendering::GlobalIlluminationVct::SetAnisotropic
  bool anisotropic;

  /// \brief See rendering::GlobalIlluminationVct::SetThinWallCounter
  float thinWallCounter;

  /// \brief See rendering::GlobalIlluminationVct::SetConserveMemory
  bool conserveMemory;

  /// \brief See rendering::GlobalIlluminationVct::DebugVisualizationMode
  uint32_t debugVisMode;
};

// Private data class.
class gz::sim::systems::SensorsPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: std::atomic<bool> initialized { false };

  /// \brief Main rendering interface
  public: RenderUtil renderUtil;

  /// \brief Unique set of sensor ids
  public: std::set<sensors::SensorId> sensorIds;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Pointer to GlobalIlluminationVct
  public: rendering::GlobalIlluminationVctPtr giVct;

  /// \brief GI VCT parameters passed to giVct
  public: GiVctParameters giVctParameters;

  /// \brief Default GI data
  public: GiDefaultData giDefaultData;

  /// \brief GI built flag
  public: bool giBuilt = false;

  /// \brief Temperature used by thermal camera. Defaults to temperature at
  /// sea level
  public: double ambientTemperature = 288.15;

  /// \brief Temperature gradient with respect to increasing altitude at sea
  /// level in units of K/m.
  public: double ambientTemperatureGradient = -0.0065;

  /// \brief Keep track of cameras, in case we need to handle stereo cameras.
  /// Key: Camera's parent scoped name
  /// Value: Pointer to camera
  public: std::unordered_map<std::string, sensors::CameraSensor *> cameras;

  /// \brief Maps gazebo entity to its matching sensor ID
  ///
  /// Useful for detecting when a sensor Entity has been deleted and trigger
  /// the destruction of the corresponding gz::sensors Sensor object
  public: std::unordered_map<Entity, sensors::SensorId> entityToIdMap;

  /// \brief Flag to indicate if worker threads are running
  public: std::atomic<bool> running { false };

  /// \brief Flag to signal if initialization should occur
  public: bool doInit { false };

  /// \brief Flag to signal if rendering update is needed
  public: std::atomic<bool> updateAvailable { false };

  /// \brief Flag to signal if a rendering update must be done
  public: std::atomic<bool> forceUpdate { false };

  /// \brief Thread that rendering will occur in
  public: std::thread renderThread;

  /// \brief Mutex to protect rendering data
  public: std::mutex renderMutex;

  /// \brief Mutex to protect renderUtil changes
  public: std::mutex renderUtilMutex;

  /// \brief Condition variable to signal rendering thread
  ///
  /// This variable is used to block/unblock operations in the rendering
  /// thread.  For a more detailed explanation on the flow refer to the
  /// documentation on RenderThread.
  public: std::condition_variable renderCv;

  /// \brief Condition variable to signal update time applied
  ///
  /// This variable is used to block/unblock operations in PostUpdate thread
  /// to make sure renderUtil's ECM updates are applied to the scene first
  /// before they are overriden by PostUpdate
  public: std::condition_variable updateTimeCv;

  /// \brief Connection to events::Stop event, used to stop thread
  public: common::ConnectionPtr stopConn;

  /// \brief Connection to events::ForceRender event, used to force rendering
  public: common::ConnectionPtr forceRenderConn;

  /// \brief Update time for the next rendering iteration
  public: std::chrono::steady_clock::duration updateTime;

  /// \brief Update time applied in the rendering thread
  public: std::chrono::steady_clock::duration updateTimeApplied;

  /// \brief Update time to be appplied in the rendering thread
  public: std::chrono::steady_clock::duration updateTimeToApply;

  /// \brief Next sensors update time
  public: std::chrono::steady_clock::duration nextUpdateTime;

  /// \brief Sensors to include in the next rendering iteration
  public: std::set<sensors::SensorId> activeSensors;

  /// \brief Sensors to be updated next
  public: std::set<sensors::SensorId> sensorsToUpdate;

  /// \brief Mutex to protect sensorMask
  public: std::mutex sensorsMutex;

  /// \brief Pointer to the event manager
  public: EventManager *eventManager{nullptr};

  /// \brief Wait for initialization to happen
  private: void WaitForInit();

  /// \brief Run one rendering iteration
  private: void RunOnce();

  /// \brief Top level function for the rendering thread
  ///
  /// This function captures all of the behavior of the rendering thread.
  /// The behavior is captured in two phases: initialization and steady state.
  ///
  /// When the thread is first started, it waits on renderCv until the
  /// prerequisites for initialization are met, and the `doInit` flag is set.
  /// In order for initialization to proceed, rendering sensors must be
  /// available in the EntityComponentManager.
  ///
  /// When doInit is set, and renderCv is notified, initialization
  /// is performed (creating the render context and scene). During
  /// initialization, execution is blocked for the caller of PostUpdate.
  /// When initialization is complete, PostUpdate will be notified via
  /// renderCv and execution will continue.
  ///
  /// Once in steady state, a rendering operation is triggered by setting
  /// updateAvailable to true, and notifying via the renderCv.
  /// The rendering operation is done in `RunOnce`.
  ///
  /// The caller of PostUpdate will not be blocked if there is no
  /// rendering operation currently ongoing. Rendering will occur
  /// asyncronously.
  //
  /// The caller of PostUpdate will be blocked if there is a rendering
  /// operation currently ongoing, until that completes.
  private: void RenderThread();

  /// \brief Launch the rendering thread
  public: void Run();

  /// \brief Stop the rendering thread
  public: void Stop();

  /// \brief Force rendering thread to render
  public: void ForceRender();

  /// \brief Update battery state of sensors in model
  /// \param[in] _ecm Entity component manager
  public: void UpdateBatteryState(const EntityComponentManager &_ecm);

  /// \brief Get the next closest sensor update time
  public: std::chrono::steady_clock::duration NextUpdateTime(
      std::set<sensors::SensorId> &_sensorsToUpdate,
      const std::chrono::steady_clock::duration &_currentTime);

  /// \brief Check if any of the sensors have connections
  public: bool SensorsHaveConnections();

  /// \brief Returns all sensors that have a pending trigger
  public: std::unordered_set<sensors::SensorId> SensorsWithPendingTrigger();

  /// \brief Use to optionally set the background color.
  public: std::optional<math::Color> backgroundColor;

  /// \brief Use to optionally set the ambient light.
  public: std::optional<math::Color> ambientLight;

  /// \brief A map between model entity ids in the ECM to its battery state
  /// True means has charge, false means drained
  public: std::unordered_map<Entity, bool> modelBatteryState;

  /// \brief A map between model entity ids in the ECM to whether its battery
  /// state has changed.
  /// True means has charge, false means drained
  public: std::unordered_map<Entity, bool> modelBatteryStateChanged;

  /// \brief A map of sensor ids to their active state
  public: std::unordered_map<sensors::SensorId, bool> sensorStateChanged;

  /// \brief Disable sensors if parent model's battery is drained
  /// Affects sensors that are in nested models
  public: bool disableOnDrainedBattery = false;

  /// \brief Mutex to protect access to sensorStateChanged
  public: std::mutex sensorStateMutex;
};

//////////////////////////////////////////////////
void SensorsPrivate::WaitForInit()
{
  while (!this->initialized && this->running)
  {
    gzdbg << "Waiting for init" << std::endl;
    std::unique_lock<std::mutex> lock(this->renderMutex);
    // Wait to be ready for initialization or stopped running.
    // We need rendering sensors to be available to initialize.
    this->renderCv.wait(lock, [this]()
    {
      return this->doInit || !this->running;
    });

    if (this->doInit)
    {
      // Only initialize if there are rendering sensors
      gzdbg << "Initializing render context" << std::endl;
      if (this->backgroundColor)
        this->renderUtil.SetBackgroundColor(*this->backgroundColor);
      if (this->ambientLight)
        this->renderUtil.SetAmbientLight(*this->ambientLight);
#ifndef __APPLE__

      this->renderUtil.Init();
#else
      // On macOS the render engine must be initialised on the main thread.
      // See Sensors::Update.
#endif
      this->scene = this->renderUtil.Scene();
      this->scene->SetCameraPassCountPerGpuFlush(6u);

      if (this->giVctParameters.enabled)
      {
        this->giVct = this->scene->CreateGlobalIlluminationVct();
        this->giVct->SetParticipatingVisuals(
            rendering::GlobalIlluminationBase::DYNAMIC_VISUALS |
            rendering::GlobalIlluminationBase::STATIC_VISUALS);

        this->giVct->SetResolution(this->giVctParameters.resolution);
        this->giVct->SetOctantCount(this->giVctParameters.octantCount);
        this->giVct->SetBounceCount(this->giVctParameters.bounceCount);
        this->giVct->SetAnisotropic(this->giVctParameters.anisotropic);
        this->giVct->SetHighQuality(this->giVctParameters.highQuality);
        this->giVct->SetConserveMemory(this->giVctParameters.conserveMemory);
        this->giVct->SetThinWallCounter(this->giVctParameters.thinWallCounter);

        this->giVct->SetDebugVisualization(
            rendering::GlobalIlluminationVct::DVM_None);

        this->scene->SetActiveGlobalIllumination(this->giVct);
      }

      this->initialized = true;
    }

    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
  gzdbg << "Rendering Thread initialized" << std::endl;
}

//////////////////////////////////////////////////
void SensorsPrivate::RunOnce()
{
  {
    std::unique_lock<std::mutex> cvLock(this->renderMutex);
    this->renderCv.wait_for(cvLock, std::chrono::microseconds(1000), [this]()
    {
      return !this->running || this->updateAvailable;
    });
  }

  if (!this->updateAvailable)
    return;

  if (!this->running)
    return;

  if (!this->scene)
    return;

  GZ_PROFILE("SensorsPrivate::RunOnce");
  {
    GZ_PROFILE("Update");
    std::unique_lock<std::mutex> timeLock(this->renderUtilMutex);
    this->renderUtil.Update();
    this->updateTimeApplied = this->updateTime;
    this->updateTimeCv.notify_one();
  }

  bool activeSensorsEmpty = true;
  {
    std::unique_lock<std::mutex> lk(this->sensorsMutex);
    activeSensorsEmpty = this->activeSensors.empty();
  }

  if (!activeSensorsEmpty || this->forceUpdate)
  {
    // disable sensors that are out of battery or re-enable sensors that are
    // being charged
    if (this->disableOnDrainedBattery)
    {
      std::unique_lock<std::mutex> lock2(this->sensorStateMutex);
      for (const auto &sensorIt : this->sensorStateChanged)
      {
        sensors::Sensor *s =
            this->sensorManager.Sensor(sensorIt.first);
        if (s)
        {
          s->SetActive(sensorIt.second);
        }
      }
      this->sensorStateChanged.clear();
    }

    {
      GZ_PROFILE("PreRender");
      this->eventManager->Emit<events::PreRender>();
      this->scene->SetTime(this->updateTimeApplied);
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor

      if (!this->giBuilt)
      {
        if (this->giVctParameters.enabled)
        {
          this->giVct->Build();
          this->giVct->SetDebugVisualization(static_cast<
              rendering::GlobalIlluminationVct::DebugVisualizationMode>
              (this->giVctParameters.debugVisMode));
          this->giBuilt = true;
        }
      }

      this->scene->PreRender();
    }

    // disable sensors that have no subscribers to prevent doing unnecessary
    // work
    std::unordered_set<sensors::RenderingSensor *> tmpDisabledSensors;
    this->sensorsMutex.lock();
    for (auto id : this->sensorIds)
    {
      sensors::Sensor *s = this->sensorManager.Sensor(id);
      auto rs = dynamic_cast<sensors::RenderingSensor *>(s);
      if (rs->IsActive() && !rs->HasConnections())
      {
        rs->SetActive(false);
        tmpDisabledSensors.insert(rs);
      }
    }
    this->sensorsMutex.unlock();

    // safety check to see if reset occurred while we're rendering
    // avoid publishing outdated data if reset occurred
    std::unique_lock<std::mutex> timeLock(this->renderMutex);
    if (this->updateTimeApplied <= this->updateTime)
    {
      // publish data
      GZ_PROFILE("RunOnce");
      this->sensorManager.RunOnce(this->updateTimeApplied);
      this->eventManager->Emit<events::Render>();
    }

    // re-enble sensors
    for (auto &rs : tmpDisabledSensors)
    {
      rs->SetActive(true);
    }

    {
      GZ_PROFILE("PostRender");
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor
      this->scene->PostRender();
      this->eventManager->Emit<events::PostRender>();
    }

    std::unique_lock<std::mutex> lk(this->sensorsMutex);
    this->activeSensors.clear();
  }

  this->forceUpdate = false;
  {
    std::unique_lock<std::mutex> cvLock(this->renderMutex);
    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
}

//////////////////////////////////////////////////
void SensorsPrivate::RenderThread()
{
  GZ_PROFILE_THREAD_NAME("RenderThread");

  gzdbg << "SensorsPrivate::RenderThread started" << std::endl;

  // We have to wait for rendering sensors to be available
  this->WaitForInit();

  while (this->running)
  {
    this->RunOnce();
  }

  this->eventManager->Emit<events::RenderTeardown>();

  // clean up before exiting
  for (const auto id : this->sensorIds)
    this->sensorManager.Remove(id);

  this->giVct.reset();
  this->scene.reset();
  this->renderUtil.Destroy();
  gzdbg << "SensorsPrivate::RenderThread stopped" << std::endl;
}

//////////////////////////////////////////////////
void SensorsPrivate::Run()
{
  gzdbg << "SensorsPrivate::Run" << std::endl;
  this->running = true;
  this->renderThread = std::thread(&SensorsPrivate::RenderThread, this);
}

//////////////////////////////////////////////////
void SensorsPrivate::Stop()
{
  gzdbg << "SensorsPrivate::Stop" << std::endl;
  std::unique_lock<std::mutex> lock(this->renderMutex);
  this->running = false;

  if (this->stopConn)
  {
    // Clear connection to stop additional incoming events.
    this->stopConn.reset();
  }

  lock.unlock();
  this->renderCv.notify_all();

  if (this->renderThread.joinable())
  {
    this->renderThread.join();
  }
}

//////////////////////////////////////////////////
void SensorsPrivate::ForceRender()
{
  this->forceUpdate = true;
}

//////////////////////////////////////////////////
void Sensors::RemoveSensor(const Entity &_entity)
{
  auto idIter = this->dataPtr->entityToIdMap.find(_entity);
  if (idIter != this->dataPtr->entityToIdMap.end())
  {
    // Remove from active sensors as well
    // Locking mutex to make sure the vector is not being changed while
    // the rendering thread is iterating over it
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->sensorsMutex);
      this->dataPtr->activeSensors.erase(idIter->second);
      this->dataPtr->sensorsToUpdate.erase(idIter->second);
    }

    // update cameras list
    for (auto &it : this->dataPtr->cameras)
    {
      if (it.second->Id() == idIter->second)
      {
        this->dataPtr->cameras.erase(it.first);
        break;
      }
    }

    this->dataPtr->sensorIds.erase(idIter->second);
    this->dataPtr->sensorManager.Remove(idIter->second);
    this->dataPtr->entityToIdMap.erase(idIter);
  }
}

//////////////////////////////////////////////////
Sensors::Sensors() : System(), dataPtr(std::make_unique<SensorsPrivate>())
{
}

//////////////////////////////////////////////////
Sensors::~Sensors()
{
  this->dataPtr->Stop();
}

/// \brief Helper to convert math::Vector3d to uint32_t array
/// \param[in] _valueToSet Array values to set
/// \param[in] _vecValues Vector values to convert
static void convertVector3dToUInt32Array(uint32_t _valueToSet[3],
    const math::Vector3d &_vecValues)
{
  _valueToSet[0] = static_cast<uint32_t>(_vecValues[0]);
  _valueToSet[1] = static_cast<uint32_t>(_vecValues[1]);
  _valueToSet[2] = static_cast<uint32_t>(_vecValues[2]);
}

/// \brief Helper to parse math::Vector3d as uint32_t array
/// \param[in] _parentElem Parent element to look through
/// \param[in] _childName Child element name to look for
/// \param[in] _valueToSet Array values to set
/// \param[in] _defaultValue Default vector values to use
static void parseVector3dAsUInt32Array(sdf::ElementConstPtr _parentElem,
    const char *_childName, uint32_t _valueToSet[3],
    const math::Vector3d &_defaultValue)
{
  math::Vector3d parsedValues = (_parentElem == nullptr) ? _defaultValue :
      _parentElem->Get<math::Vector3d>(_childName, _defaultValue).first;

  convertVector3dToUInt32Array(_valueToSet, parsedValues);
}

/// \brief Helper to set debug visualization mode (DVM)
/// \param[in] _text String text to parse
/// \param[in] _modeToSet DVM to set
/// \param[in] _defaultMode Default DVM to use
static void SetDebugVisMode(const std::string &_text,
    uint32_t &_modeToSet, uint32_t _defaultMode)
{
  if (_text == "albedo")
  {
    _modeToSet = rendering::GlobalIlluminationVct::DVM_Albedo;
  }
  else if (_text == "normal")
  {
    _modeToSet = rendering::GlobalIlluminationVct::DVM_Normal;
  }
  else if (_text == "emissive")
  {
    _modeToSet = rendering::GlobalIlluminationVct::DVM_Emissive;
  }
  else if (_text == "lighting")
  {
    _modeToSet = rendering::GlobalIlluminationVct::DVM_Lighting;
  }
  else if (_text == "none")
  {
    _modeToSet = rendering::GlobalIlluminationVct::DVM_None;
  }
  else
  {
    _modeToSet = _defaultMode;
  }
}

//////////////////////////////////////////////////
void Sensors::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  gzdbg << "Configuring Sensors system" << std::endl;

  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

  std::string apiBackend =
    _sdf->Get<std::string>("render_engine_api_backend", "").first;

  // get whether or not to disable sensor when model battery is drained
  this->dataPtr->disableOnDrainedBattery =
      _sdf->Get<bool>("disable_on_drained_battery",
     this->dataPtr-> disableOnDrainedBattery).first;

  // Get the background color, if specified.
  if (_sdf->HasElement("background_color"))
    this->dataPtr->backgroundColor = _sdf->Get<math::Color>("background_color");

  // Get the ambient light, if specified.
  if (_sdf->HasElement("ambient_light"))
    this->dataPtr->ambientLight = _sdf->Get<math::Color>("ambient_light");

  // Get the global illumination technique and its parameters, if specified
  if (_sdf->HasElement("global_illumination"))
  {
    if (engineName != "ogre2")
    {
      gzerr << "Global illumination is only supported by the ogre2 "
            << "render engine" << std::endl;
    }
    else
    {
      auto giElem = _sdf->FindElement("global_illumination");
      std::string giType = giElem->GetAttribute("type")->GetAsString();
      if (giType == "vct")
      {
        this->dataPtr->giVctParameters.enabled = giElem->Get<bool>(
            "enabled", this->dataPtr->giVctParameters.enabled).first;

        // Use helper functions to parse the inputted set of values
        // as a uint32_t array
        if (giElem->HasElement("resolution"))
        {
          parseVector3dAsUInt32Array(giElem, "resolution",
              this->dataPtr->giVctParameters.resolution,
              this->dataPtr->giDefaultData.resolution);
        }
        else
        {
          convertVector3dToUInt32Array(
              this->dataPtr->giVctParameters.resolution,
              this->dataPtr->giDefaultData.resolution);
        }

        if (giElem->HasElement("octant_count"))
        {
          parseVector3dAsUInt32Array(giElem, "octant_count",
              this->dataPtr->giVctParameters.octantCount,
              this->dataPtr->giDefaultData.octantCount);
        }
        else
        {
          convertVector3dToUInt32Array(
              this->dataPtr->giVctParameters.octantCount,
              this->dataPtr->giDefaultData.octantCount);
        }

        this->dataPtr->giVctParameters.bounceCount =
            giElem->Get<uint32_t>("bounce_count",
            this->dataPtr->giVctParameters.bounceCount).first;
        this->dataPtr->giVctParameters.highQuality =
            giElem->Get<bool>("high_quality",
            this->dataPtr->giVctParameters.highQuality).first;
        this->dataPtr->giVctParameters.anisotropic =
            giElem->Get<bool>("anisotropic",
            this->dataPtr->giVctParameters.anisotropic).first;
        this->dataPtr->giVctParameters.thinWallCounter =
            giElem->Get<float>("thin_wall_counter",
            this->dataPtr->giVctParameters.thinWallCounter).first;
        this->dataPtr->giVctParameters.conserveMemory =
            giElem->Get<bool>("conserve_memory",
            this->dataPtr->giVctParameters.conserveMemory).first;

        if (giElem->HasElement("debug_vis_mode"))
        {
          const std::string text = giElem->Get<std::string>(
              "debug_vis_mode", "none").first;
          SetDebugVisMode(text, this->dataPtr->giVctParameters.debugVisMode,
              this->dataPtr->giDefaultData.debugVisMode);
        }
      }
      else if (giType == "civct")
      {
        // todo: add CIVCT here. should also check if apiBackend is vulkan
        // can use SetDebugVisMode when parsing DVM
        gzerr << "GI CI VCT is not supported" << std::endl;
      }
      else
      {
        gzerr << "GI method type [" << giType << "] is not supported."
              << std::endl;
      }
    }
  }

  this->dataPtr->renderUtil.SetEngineName(engineName);
#ifdef __APPLE__
  if (apiBackend.empty())
    apiBackend = "metal";
#endif
  this->dataPtr->renderUtil.SetApiBackend(apiBackend);
  this->dataPtr->renderUtil.SetEnableSensors(true,
      std::bind(&Sensors::CreateSensor, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->dataPtr->renderUtil.SetRemoveSensorCb(
      std::bind(&Sensors::RemoveSensor, this, std::placeholders::_1));
  this->dataPtr->renderUtil.SetEventManager(&_eventMgr);

  // parse sensor-specific data
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity != worldEntity)
  {
    // temperature used by thermal camera
    auto atmosphere = _ecm.Component<components::Atmosphere>(worldEntity);
    if (atmosphere)
    {
      auto atmosphereSdf = atmosphere->Data();
      this->dataPtr->ambientTemperature = atmosphereSdf.Temperature().Kelvin();
      this->dataPtr->ambientTemperatureGradient =
          atmosphereSdf.TemperatureGradient();
    }

    // Set render engine if specified from command line
    auto renderEngineServerComp =
      _ecm.Component<components::RenderEngineServerPlugin>(worldEntity);
    if (renderEngineServerComp && !renderEngineServerComp->Data().empty())
    {
      this->dataPtr->renderUtil.SetEngineName(renderEngineServerComp->Data());
    }

    // Set API backend if specified from command line
    auto renderEngineServerApiBackendComp =
      _ecm.Component<components::RenderEngineServerApiBackend>(worldEntity);
    if (renderEngineServerApiBackendComp &&
        !renderEngineServerApiBackendComp->Data().empty())
    {
      this->dataPtr->renderUtil.SetApiBackend(
        renderEngineServerApiBackendComp->Data());
    }

    // Set headless mode if specified from command line
    auto renderEngineServerHeadlessComp =
      _ecm.Component<components::RenderEngineServerHeadless>(worldEntity);
    if (renderEngineServerHeadlessComp)
    {
      this->dataPtr->renderUtil.SetHeadlessRendering(
        renderEngineServerHeadlessComp->Data());
    }
  }

  this->dataPtr->eventManager = &_eventMgr;

  this->dataPtr->stopConn = this->dataPtr->eventManager->Connect<events::Stop>(
      std::bind(&SensorsPrivate::Stop, this->dataPtr.get()));

  this->dataPtr->forceRenderConn = _eventMgr.Connect<events::ForceRender>(
      std::bind(&SensorsPrivate::ForceRender, this->dataPtr.get()));

  // Kick off worker thread
  this->dataPtr->Run();
}

//////////////////////////////////////////////////
void Sensors::Reset(const UpdateInfo &_info, EntityComponentManager &)
{
  GZ_PROFILE("Sensors::Reset");

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    gzdbg << "Resetting Sensors\n";

    {
      std::unique_lock<std::mutex> lock(this->dataPtr->sensorsMutex);
      this->dataPtr->activeSensors.clear();
      this->dataPtr->sensorsToUpdate.clear();
    }

    for (auto id : this->dataPtr->sensorIds)
    {
      sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(id);

      if (nullptr == s)
      {
        gzwarn << "Sensor removed before reset: " << id << "\n";
        continue;
      }

      s->SetNextDataUpdateTime(_info.simTime);
    }
    this->dataPtr->nextUpdateTime =  _info.simTime;
    std::unique_lock<std::mutex> lock2(this->dataPtr->renderUtilMutex);
    this->dataPtr->updateTime =  _info.simTime;
    this->dataPtr->updateTimeToApply =  _info.simTime;
    this->dataPtr->updateTimeApplied =  _info.simTime;
  }
}

//////////////////////////////////////////////////
void Sensors::Update(const UpdateInfo &_info,
                     EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sensors::Update");

#ifdef __APPLE__
  // On macOS the render engine must be initialised on the main thread.
  if (!this->dataPtr->initialized &&
      (_ecm.HasComponentType(components::BoundingBoxCamera::typeId) ||
       _ecm.HasComponentType(components::Camera::typeId) ||
       _ecm.HasComponentType(components::DepthCamera::typeId) ||
       _ecm.HasComponentType(components::GpuLidar::typeId) ||
       _ecm.HasComponentType(components::RgbdCamera::typeId) ||
       _ecm.HasComponentType(components::ThermalCamera::typeId) ||
       _ecm.HasComponentType(components::SegmentationCamera::typeId) ||
       _ecm.HasComponentType(components::WideAngleCamera::typeId)))
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
    gzdbg << "Initialization needed" << std::endl;
    this->dataPtr->renderUtil.Init();
    this->dataPtr->nextUpdateTime = _info.simTime;
  }
#endif

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sensors::PostUpdate");
  {
    if (!this->dataPtr->initialized &&
        (this->dataPtr->forceUpdate ||
         _ecm.HasComponentType(components::BoundingBoxCamera::typeId) ||
         _ecm.HasComponentType(components::Camera::typeId) ||
         _ecm.HasComponentType(components::DepthCamera::typeId) ||
         _ecm.HasComponentType(components::GpuLidar::typeId) ||
         _ecm.HasComponentType(components::RgbdCamera::typeId) ||
         _ecm.HasComponentType(components::ThermalCamera::typeId) ||
         _ecm.HasComponentType(components::SegmentationCamera::typeId) ||
         _ecm.HasComponentType(components::WideAngleCamera::typeId)))
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
      gzdbg << "Initialization needed" << std::endl;
      this->dataPtr->doInit = true;
      this->dataPtr->renderCv.notify_one();
    }
  }

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    {
      GZ_PROFILE("UpdateFromECM");
      // Make sure we do not override the state in renderUtil if there are
      // still ECM changes that still need to be propagated to the scene,
      // i.e. wait until renderUtil.Update(), has taken place in the
      // rendering thread
      std::unique_lock<std::mutex> lock(this->dataPtr->renderUtilMutex);
      this->dataPtr->updateTimeCv.wait(lock, [this]()
      {
        return !this->dataPtr->updateAvailable ||
               (this->dataPtr->updateTimeToApply ==
               this->dataPtr->updateTimeApplied);
      });

      this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
      this->dataPtr->updateTime = _info.simTime;
    }

    // check connections to render events
    // we will need to perform render updates if there are event subscribers
    // \todo(anyone) This currently forces scene tree updates at the sim update
    // rate which can be too frequent and causes a performance hit.
    // We should look into throttling render updates
    bool hasRenderConnections =
      (this->dataPtr->eventManager->ConnectionCount<events::PreRender>() > 0u ||
      this->dataPtr->eventManager->ConnectionCount<events::Render>() > 0u ||
      this->dataPtr->eventManager->ConnectionCount<events::PostRender>() > 0u);

    // if nextUpdateTime is max, it probably means there are previously
    // no active sensors or sensors with connections.
    // In this case, check if sensors have connections now. If so, we need to
    // set the nextUpdateTime
    if (this->dataPtr->nextUpdateTime ==
        std::chrono::steady_clock::duration::max() &&
        this->dataPtr->SensorsHaveConnections())
    {
      this->dataPtr->nextUpdateTime = this->dataPtr->NextUpdateTime(
          this->dataPtr->sensorsToUpdate, _info.simTime);
    }

    std::unordered_set<sensors::SensorId> sensorsWithPendingTriggers =
        this->dataPtr->SensorsWithPendingTrigger();

    // notify the render thread if updates are available
    if (hasRenderConnections ||
        this->dataPtr->nextUpdateTime <= _info.simTime ||
        this->dataPtr->renderUtil.PendingSensors() > 0 ||
        this->dataPtr->forceUpdate ||
        !sensorsWithPendingTriggers.empty())
    {
      if (this->dataPtr->disableOnDrainedBattery)
        this->dataPtr->UpdateBatteryState(_ecm);

      {
        std::unique_lock<std::mutex> cvLock(this->dataPtr->renderMutex);
        this->dataPtr->renderCv.wait(cvLock, [this] {
          return !this->dataPtr->running || !this->dataPtr->updateAvailable; });
      }

      if (!this->dataPtr->running)
      {
        return;
      }

      {
        std::unique_lock<std::mutex> lockSensors(this->dataPtr->sensorsMutex);
        this->dataPtr->activeSensors =
            std::move(this->dataPtr->sensorsToUpdate);
        // Add all sensors that have pending triggers.
        this->dataPtr->activeSensors.insert(sensorsWithPendingTriggers.begin(),
                                            sensorsWithPendingTriggers.end());
      }

      this->dataPtr->nextUpdateTime = this->dataPtr->NextUpdateTime(
          this->dataPtr->sensorsToUpdate, _info.simTime);

      // Force scene tree update if there are sensors to be created or
      // subscribes to the render events. This does not necessary force
      // sensors to update. Only active sensors will be updated
      this->dataPtr->forceUpdate =
          (this->dataPtr->renderUtil.PendingSensors() > 0) ||
          hasRenderConnections;

      {
        std::unique_lock<std::mutex> timeLock(this->dataPtr->renderUtilMutex);
        this->dataPtr->updateTimeToApply = this->dataPtr->updateTime;
      }

      {
        std::unique_lock<std::mutex> cvLock(this->dataPtr->renderMutex);
        this->dataPtr->updateAvailable = true;
        this->dataPtr->renderCv.notify_one();
      }
    }
  }
}

//////////////////////////////////////////////////
void SensorsPrivate::UpdateBatteryState(const EntityComponentManager &_ecm)
{
  // Battery state
  _ecm.Each<components::BatterySoC>(
      [&](const Entity & _entity, const components::BatterySoC *_bat)
      {
        bool hasCharge = _bat->Data() > 0;
        auto stateIt =
          this->modelBatteryState.find(_ecm.ParentEntity(_entity));
        if (stateIt != this->modelBatteryState.end())
        {
          // detect a change in battery charge state
          if (stateIt->second != hasCharge)
          {
            this->modelBatteryStateChanged[_ecm.ParentEntity(_entity)] =
                hasCharge;
          }
        }
        this->modelBatteryState[_ecm.ParentEntity(_entity)] = hasCharge;
        return true;
      });

  // disable sensor if parent model is out of battery or re-enable sensor
  // if battery is charging
  for (const auto & modelIt : this->modelBatteryStateChanged)
  {
    // check if sensor is part of this model
    for (const auto & sensorIt : this->entityToIdMap)
    {
      // parent link
      auto parentLinkComp =
          _ecm.Component<components::ParentEntity>(sensorIt.first);
      if (!parentLinkComp)
        continue;

      // parent model
      auto parentModelComp = _ecm.Component<components::ParentEntity>(
          parentLinkComp->Data());
      if (!parentModelComp)
        continue;

      // keep going up the tree in case sensor is in a nested model
      while (parentModelComp)
      {
        auto parentEnt = parentModelComp->Data();
        if (parentEnt == modelIt.first)
        {
          std::unique_lock<std::mutex> lock(this->sensorStateMutex);
          // sensor is part of model - update its active state
          this->sensorStateChanged[sensorIt.second] = modelIt.second;
          break;
        }
        parentModelComp = _ecm.Component<components::ParentEntity>(parentEnt);
      }
    }
  }
  this->modelBatteryStateChanged.clear();
}

//////////////////////////////////////////////////
std::string Sensors::CreateSensor(const Entity &_entity,
    const sdf::Sensor &_sdf, const std::string &_parentName)
{
  if (_sdf.Type() == sdf::SensorType::NONE)
  {
    gzerr << "Unable to create sensor. SDF sensor type is NONE." << std::endl;
    return std::string();
  }

  // Create within gz-sensors
  sensors::Sensor *sensor{nullptr};
  if (_sdf.Type() == sdf::SensorType::CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::CameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::DEPTH_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::DepthCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::GPU_LIDAR)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::GpuLidarSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::RGBD_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::RgbdCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::THERMAL_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::ThermalCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::BOUNDINGBOX_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::BoundingBoxCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::SEGMENTATION_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::SegmentationCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::WIDE_ANGLE_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      sensors::WideAngleCameraSensor>(_sdf);
  }

  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << _sdf.Name()
           << "]." << std::endl;
    return std::string();
  }

  // Store sensor ID
  auto sensorId = sensor->Id();
  this->dataPtr->entityToIdMap.insert({_entity, sensorId});
  this->dataPtr->sensorIds.insert(sensorId);

  // Set the scene so it can create the rendering sensor
  auto renderingSensor = dynamic_cast<sensors::RenderingSensor *>(sensor);
  renderingSensor->SetScene(this->dataPtr->scene);
  renderingSensor->SetParent(_parentName);
  renderingSensor->SetManualSceneUpdate(true);

  // Special case for stereo cameras
  auto cameraSensor = dynamic_cast<sensors::CameraSensor *>(sensor);
  if (nullptr != cameraSensor)
  {
    // Parent
    auto parent = cameraSensor->Parent();

    // If parent has other camera children, set the baseline.
    // For stereo pairs, the baseline for the left camera is zero, and for the
    // right camera it's the distance between them.
    // For more than 2 cameras, the first camera's baseline is zero and the
    // others have the distance between them.
    if (this->dataPtr->cameras.find(parent) !=
        this->dataPtr->cameras.end())
    {
      // TODO(anyone) This is safe because we're not removing sensors
      // First camera added to the parent link
      auto leftCamera = this->dataPtr->cameras[parent];
      auto rightCamera = cameraSensor;

      // If cameras have right / left topic, use that to decide which is which
      if (leftCamera->Topic().find("right") != std::string::npos &&
          rightCamera->Topic().find("left") != std::string::npos)
      {
        std::swap(rightCamera, leftCamera);
      }

      // Camera sensor's Y axis is orthogonal to the optical axis
      auto baseline = abs(rightCamera->Pose().Pos().Y() -
                          leftCamera->Pose().Pos().Y());
      rightCamera->SetBaseline(baseline);
    }
    else
    {
      this->dataPtr->cameras[parent] = cameraSensor;
    }
  }

  // Sensor-specific settings
  auto thermalSensor = dynamic_cast<sensors::ThermalCameraSensor *>(sensor);
  if (nullptr != thermalSensor)
  {
    thermalSensor->SetAmbientTemperature(this->dataPtr->ambientTemperature);

    // temperature gradient is in kelvin per meter - typically change in
    // temperature over change in altitude. However the implementation of
    // thermal sensor in gz-sensors varies temperature for all objects in its
    // view. So we will do an approximation based on camera view's vertical
    // distance.
    auto camSdf = _sdf.CameraSensor();
    double farClip = camSdf->FarClip();
    double angle = camSdf->HorizontalFov().Radian();
    double aspect = camSdf->ImageWidth() / camSdf->ImageHeight();
    double vfov = 2.0 * atan(tan(angle / 2.0) / aspect);
    double height = tan(vfov / 2.0) * farClip * 2.0;
    double tempRange =
        std::fabs(this->dataPtr->ambientTemperatureGradient * height);
    thermalSensor->SetAmbientTemperatureRange(tempRange);

    gzmsg << "Setting ambient temperature to "
           << this->dataPtr->ambientTemperature << " Kelvin and gradient to "
           << this->dataPtr->ambientTemperatureGradient << " K/m. "
           << "The resulting temperature range is: " << tempRange
           << " Kelvin." << std::endl;
  }

  return sensor->Name();
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration SensorsPrivate::NextUpdateTime(
    std::set<sensors::SensorId> &_sensorsToUpdate,
    const std::chrono::steady_clock::duration &_currentTime)
{
  _sensorsToUpdate.clear();
  std::chrono::steady_clock::duration minNextUpdateTime =
      std::chrono::steady_clock::duration::max();
  for (auto id : this->sensorIds)
  {
    sensors::Sensor *s = this->sensorManager.Sensor(id);

    if (nullptr == s)
    {
      continue;
    }

    auto rs = dynamic_cast<sensors::RenderingSensor *>(s);

    if (nullptr == rs)
    {
      continue;
    }

    if (!rs->HasConnections())
    {
      continue;
    }

    if (rs->IsTriggered())
    {
      continue;
    }

    std::chrono::steady_clock::duration time;
    // if sensor's next update tims is less or equal to current sim time then
    // it's in the process of being updated by the render loop
    // Set their next update time  to be current time + update period
    if (rs->NextDataUpdateTime() <= _currentTime)
    {
      time = rs->NextDataUpdateTime() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rs->UpdateRate()));
    }
    else
    {
      time = rs->NextDataUpdateTime();
    }

    if (time <= minNextUpdateTime)
    {
      _sensorsToUpdate.clear();
      minNextUpdateTime = time;
    }
    _sensorsToUpdate.insert(id);
  }
  return minNextUpdateTime;
}

//////////////////////////////////////////////////
bool SensorsPrivate::SensorsHaveConnections()
{
  for (auto id : this->sensorIds)
  {
    sensors::Sensor *s = this->sensorManager.Sensor(id);
    if (nullptr == s)
    {
      continue;
    }

    auto rs = dynamic_cast<sensors::RenderingSensor *>(s);

    if (nullptr == rs)
    {
      continue;
    }

    if (rs->HasConnections())
    {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
std::unordered_set<sensors::SensorId>
SensorsPrivate::SensorsWithPendingTrigger()
{
  std::unordered_set<sensors::SensorId> sensorsWithPendingTrigger;
  for (auto id : this->sensorIds)
  {
    sensors::Sensor *s = this->sensorManager.Sensor(id);
    if (nullptr == s)
    {
      continue;
    }

    if (s->HasPendingTrigger())
    {
      sensorsWithPendingTrigger.insert(id);
    }
  }
  return sensorsWithPendingTrigger;
}

GZ_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemReset,
  Sensors::ISystemUpdate,
  Sensors::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Sensors, "gz::sim::systems::Sensors")
