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

#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/Manager.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

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
};

//////////////////////////////////////////////////
Sensors::Sensors() : System(), dataPtr(std::make_unique<SensorsPrivate>())
{
}

//////////////////////////////////////////////////
Sensors::~Sensors()
{
}

//////////////////////////////////////////////////
void Sensors::Configure(const EntityId &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Setup rendering
  auto engineName = _sdf->Get<std::string>("render_engine", "ogre").first;

  // TODO(anyone) Only do this if we do have rendering sensors
  auto *engine = ignition::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Failed to load engine [" << engineName << "]" << std::endl;
    return;
  }
  auto scene = engine->CreateScene("scene");

  // Create simulation runner sensor manager
  this->dataPtr->sensorManager.SetRenderingScene(scene);

  // Create cameras
  _ecm.Each<components::Camera>(
    [&](const EntityId &_entity,
        const components::Camera *_camera)->bool
    {

      auto cameraSensor = this->dataPtr->sensorManager.CreateSensor
          <sensors::CameraSensor>(_camera->Data());

      return true;
    });
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &/*_ecm*/)
{
  auto time = math::durationToSecNsec(_info.simTime);
  this->dataPtr->sensorManager.RunOnce(common::Time(time.first, time.second));
}

IGNITION_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemPostUpdate
)
