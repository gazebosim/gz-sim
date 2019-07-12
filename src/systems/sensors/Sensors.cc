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

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/common/Time.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/RenderingSensor.hh>
#include <ignition/sensors/Manager.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "Sensors.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::SensorsPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief Main rendering interface
  public: RenderUtil renderUtil;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;
};

//////////////////////////////////////////////////
Sensors::Sensors() : System(), dataPtr(std::make_unique<SensorsPrivate>())
{
}

//////////////////////////////////////////////////
Sensors::~Sensors() = default;

//////////////////////////////////////////////////
void Sensors::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

  this->dataPtr->renderUtil.SetEngineName(engineName);
  this->dataPtr->renderUtil.SetEnableSensors(true,
      std::bind(&Sensors::CreateSensor, this,
      std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  // Only initialize if there are rendering sensors
  if (!this->dataPtr->initialized &&
      (_ecm.HasComponentType(components::Camera::typeId) ||
       _ecm.HasComponentType(components::DepthCamera::typeId) ||
       _ecm.HasComponentType(components::GpuLidar::typeId) ||
       _ecm.HasComponentType(components::RgbdCamera::typeId)))
  {
    this->dataPtr->renderUtil.Init();
    this->dataPtr->scene = this->dataPtr->renderUtil.Scene();
    this->dataPtr->initialized = true;
  }

  if (!this->dataPtr->initialized)
    return;

  this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);

  // update scene graph
  this->dataPtr->renderUtil.Update();

  // udate rendering sensors
  auto time = math::durationToSecNsec(_info.simTime);
  auto t = common::Time(time.first, time.second);

  // enable sensors if they need to be updated
  // std::vector<sensors::RenderingSensor *> activeSensors;
  std::vector<sensors::SensorId> sensorIds =
      this->dataPtr->sensorManager.Sensors();
  // for (auto id : sensorIds)
  // {
  //   sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(id);
  //   sensors::RenderingSensor *rs = dynamic_cast<sensors::RenderingSensor *>(s);
  //   if (rs && rs->NextUpdateTime() <= t)
  //   {
  //     std::cerr << "set enabled " << rs->Name() << std::endl;
  //     activeSensors.push_back(rs);
  //   }
  // }


  common::Time tn = common::Time::SystemTime();

  // global scene update - render one frame and update all active sensors
  bool update = false;

  for (auto id : sensorIds)
  {
    sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(id);
    sensors::RenderingSensor *rs = dynamic_cast<sensors::RenderingSensor *>(s);
    if (rs && rs->NextUpdateTime() <= t)
    {
      update = true;
      break;
    }
  }

  if (!update)
    return;

  this->dataPtr->scene->PreRender();

  // publish data
  this->dataPtr->sensorManager.RunOnce(t);

  this->dataPtr->scene->PostRender();

  common::Time dt = common::Time::SystemTime() - tn;
//  if (dt.Double() > 0.001)
//    std::cerr << dt.Double() << std::endl;

  // disable sensors after update
  // for (auto rs : activeSensors)
  // {
  //   std::cerr << "set disabled " << rs->Name() << std::endl;
  // }

}

//////////////////////////////////////////////////
std::string Sensors::CreateSensor(const sdf::Sensor &_sdf,
    const std::string &_parentName)
{
  if (_sdf.Type() == sdf::SensorType::NONE)
  {
    ignerr << "Unable to create sensor. SDF sensor type is NONE." << std::endl;
    return std::string();
  }

  // Create within ign-sensors
  auto sensorId = this->dataPtr->sensorManager.CreateSensor(_sdf);
  auto sensor = this->dataPtr->sensorManager.Sensor(sensorId);

  if (nullptr == sensor || sensors::NO_SENSOR == sensor->Id())
  {
    ignerr << "Failed to create sensor [" << _sdf.Name()
           << "]" << std::endl;
  }

  // Set the scene so it can create the rendering sensor
  auto renderingSensor =
      dynamic_cast<sensors::RenderingSensor *>(sensor);
  renderingSensor->SetScene(this->dataPtr->scene);
  renderingSensor->SetParent(_parentName);

  return sensor->Name();
}

IGNITION_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Sensors, "ignition::gazebo::systems::Sensors")
