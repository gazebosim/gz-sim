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

#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>

#include <gz/rendering/Scene.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <gz/sensors/RgbdCameraSensor.hh>
#include <gz/sensors/ThermalCameraSensor.hh>
#include <gz/sensors/SegmentationCameraSensor.hh>
#include <gz/sensors/WideAngleCameraSensor.hh>
#include <gz/sensors/Manager.hh>

#include "gz/sim/components/Atmosphere.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/ParentEntity.hh"
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

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class.
class gz::sim::systems::SensorsPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief Main rendering interface
  public: RenderUtil renderUtil;

  /// \brief Unique set of sensor ids
  public: std::set<sensors::SensorId> sensorIds;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Temperature used by thermal camera. Defaults to temperature at
  /// sea level
  public: double ambientTemperature = 288.15;

  /// \brief Temperature gradient with respect to increasing altitude at sea
  /// level in units of K/m.
  public: double ambientTemperatureGradient = -0.0065;

  /// \brief Keep track of cameras, in case we need to handle stereo cameras.
  /// Key: Camera's parent scoped name
  /// Value: Pointer to camera
  public: std::map<std::string, sensors::CameraSensor *> cameras;

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
  public: bool updateAvailable { false };

  /// \brief Thread that rendering will occur in
  public: std::thread renderThread;

  /// \brief Mutex to protect rendering data
  public: std::mutex renderMutex;

  /// \brief Condition variable to signal rendering thread
  ///
  /// This variable is used to block/unblock operations in the rendering
  /// thread.  For a more detailed explanation on the flow refer to the
  /// documentation on RenderThread.
  public: std::condition_variable renderCv;

  /// \brief Connection to events::Stop event, used to stop thread
  public: gz::common::ConnectionPtr stopConn;

  /// \brief Update time for the next rendering iteration
  public: std::chrono::steady_clock::duration updateTime;

  /// \brief Sensors to include in the next rendering iteration
  public: std::vector<sensors::RenderingSensor *> activeSensors;

  /// \brief Mutex to protect sensorMask
  public: std::mutex sensorMaskMutex;

  /// \brief Mask sensor updates for sensors currently being rendered
  public: std::map<sensors::SensorId,
    std::chrono::steady_clock::duration> sensorMask;

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

  /// \brief Update battery state of sensors in model
  /// \param[in] _ecm Entity component manager
  public: void UpdateBatteryState(const EntityComponentManager &_ecm);

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
  public: std::map<sensors::SensorId, bool> sensorStateChanged;

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
    igndbg << "Waiting for init" << std::endl;
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
      igndbg << "Initializing render context" << std::endl;
      if (this->backgroundColor)
        this->renderUtil.SetBackgroundColor(*this->backgroundColor);
      if (this->ambientLight)
        this->renderUtil.SetAmbientLight(*this->ambientLight);
      this->renderUtil.Init();
      this->scene = this->renderUtil.Scene();
      this->scene->SetCameraPassCountPerGpuFlush(6u);
      this->initialized = true;
    }

    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
  igndbg << "Rendering Thread initialized" << std::endl;
}

//////////////////////////////////////////////////
void SensorsPrivate::RunOnce()
{
  std::unique_lock<std::mutex> lock(this->renderMutex);
  this->renderCv.wait(lock, [this]()
  {
    return !this->running || this->updateAvailable;
  });

  if (!this->running)
    return;

  if (!this->scene)
    return;

  IGN_PROFILE("SensorsPrivate::RunOnce");
  {
    IGN_PROFILE("Update");
    this->renderUtil.Update();
  }

  if (!this->activeSensors.empty())
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

    this->sensorMaskMutex.lock();
    // Check the active sensors against masked sensors.
    //
    // The internal state of a rendering sensor is not updated until the
    // rendering operation is complete, which can leave us in a position
    // where the sensor is falsely indicating that an update is needed.
    //
    // To prevent this, add sensors that are currently being rendered to
    // a mask. Sensors are removed from the mask when 90% of the update
    // delta has passed, which will allow rendering to proceed.
    for (const auto & sensor : this->activeSensors)
    {
      // 90% of update delta (1/UpdateRate());
      auto delta = std::chrono::duration_cast< std::chrono::milliseconds>(
        std::chrono::duration< double >(0.9 / sensor->UpdateRate()));
      this->sensorMask[sensor->Id()] = this->updateTime + delta;
    }
    this->sensorMaskMutex.unlock();

    {
      IGN_PROFILE("PreRender");
      this->eventManager->Emit<events::PreRender>();
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor
      this->scene->PreRender();
    }

    {
      // publish data
      IGN_PROFILE("RunOnce");
      this->sensorManager.RunOnce(this->updateTime);
    }

    {
      IGN_PROFILE("PostRender");
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor
      this->scene->PostRender();
      this->eventManager->Emit<events::PostRender>();
    }

    this->activeSensors.clear();
  }

  this->updateAvailable = false;
  lock.unlock();
  this->renderCv.notify_one();
}

//////////////////////////////////////////////////
void SensorsPrivate::RenderThread()
{
  IGN_PROFILE_THREAD_NAME("RenderThread");

  igndbg << "SensorsPrivate::RenderThread started" << std::endl;

  // We have to wait for rendering sensors to be available
  this->WaitForInit();

  while (this->running)
  {
    this->RunOnce();
  }

  // clean up before exiting
  for (const auto id : this->sensorIds)
    this->sensorManager.Remove(id);

  igndbg << "SensorsPrivate::RenderThread stopped" << std::endl;
}

//////////////////////////////////////////////////
void SensorsPrivate::Run()
{
  igndbg << "SensorsPrivate::Run" << std::endl;
  this->running = true;
  this->renderThread = std::thread(&SensorsPrivate::RenderThread, this);
}

//////////////////////////////////////////////////
void SensorsPrivate::Stop()
{
  igndbg << "SensorsPrivate::Stop" << std::endl;
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
void Sensors::RemoveSensor(const Entity &_entity)
{
  auto idIter = this->dataPtr->entityToIdMap.find(_entity);
  if (idIter != this->dataPtr->entityToIdMap.end())
  {
    // Remove from active sensors as well
    // Locking mutex to make sure the vector is not being changed while
    // the rendering thread is iterating over it
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->sensorMaskMutex);
      sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(idIter->second);
      auto rs = dynamic_cast<sensors::RenderingSensor *>(s);
      auto activeSensorIt = std::find(this->dataPtr->activeSensors.begin(),
          this->dataPtr->activeSensors.end(), rs);
      if (activeSensorIt != this->dataPtr->activeSensors.end())
      {
        this->dataPtr->activeSensors.erase(activeSensorIt);
      }
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

//////////////////////////////////////////////////
void Sensors::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  igndbg << "Configuring Sensors system" << std::endl;

  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

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

  this->dataPtr->renderUtil.SetEngineName(engineName);
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

  this->dataPtr->stopConn = _eventMgr.Connect<events::Stop>(
      std::bind(&SensorsPrivate::Stop, this->dataPtr.get()));

  // Kick off worker thread
  this->dataPtr->Run();
}

//////////////////////////////////////////////////
void Sensors::Update(const UpdateInfo &_info,
                     EntityComponentManager &_ecm)
{
  IGN_PROFILE("Sensors::Update");
  std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Sensors::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  {
    std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
    if (!this->dataPtr->initialized &&
        (_ecm.HasComponentType(components::Camera::typeId) ||
         _ecm.HasComponentType(components::DepthCamera::typeId) ||
         _ecm.HasComponentType(components::GpuLidar::typeId) ||
         _ecm.HasComponentType(components::RgbdCamera::typeId) ||
         _ecm.HasComponentType(components::ThermalCamera::typeId) ||
         _ecm.HasComponentType(components::SegmentationCamera::typeId) ||
         _ecm.HasComponentType(components::WideAngleCamera::typeId)))
    {
      igndbg << "Initialization needed" << std::endl;
      this->dataPtr->doInit = true;
      this->dataPtr->renderCv.notify_one();
    }
  }

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);

    auto time = math::durationToSecNsec(_info.simTime);
    auto t = math::secNsecToDuration(time.first, time.second);

    std::vector<sensors::RenderingSensor *> activeSensors;

    this->dataPtr->sensorMaskMutex.lock();
    for (auto id : this->dataPtr->sensorIds)
    {
      sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(id);
      auto rs = dynamic_cast<sensors::RenderingSensor *>(s);

      auto it = this->dataPtr->sensorMask.find(id);
      if (it != this->dataPtr->sensorMask.end())
      {
        if (it->second <= t)
        {
          this->dataPtr->sensorMask.erase(it);
        }
        else
        {
          continue;
        }
      }

      if (rs && rs->NextDataUpdateTime() <= t)
      {
        activeSensors.push_back(rs);
      }
    }
    this->dataPtr->sensorMaskMutex.unlock();

    if (!activeSensors.empty() ||
        this->dataPtr->renderUtil.PendingSensors() > 0)
    {
      if (this->dataPtr->disableOnDrainedBattery)
        this->dataPtr->UpdateBatteryState(_ecm);

      std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
      this->dataPtr->renderCv.wait(lock, [this] {
        return !this->dataPtr->running || !this->dataPtr->updateAvailable; });

      if (!this->dataPtr->running)
      {
        return;
      }

      this->dataPtr->activeSensors = std::move(activeSensors);
      this->dataPtr->updateTime = t;
      this->dataPtr->updateAvailable = true;
      this->dataPtr->renderCv.notify_one();
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
    ignerr << "Unable to create sensor. SDF sensor type is NONE." << std::endl;
    return std::string();
  }

  // Create within ign-sensors
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
    ignerr << "Failed to create sensor [" << _sdf.Name()
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
    // thermal sensor in ign-sensors varies temperature for all objects in its
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

    ignmsg << "Setting ambient temperature to "
           << this->dataPtr->ambientTemperature << " Kelvin and gradient to "
           << this->dataPtr->ambientTemperatureGradient << " K/m. "
           << "The resulting temperature range is: " << tempRange
           << " Kelvin." << std::endl;
  }

  return sensor->Name();
}

IGNITION_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemUpdate,
  Sensors::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Sensors, "gz::sim::systems::Sensors")
