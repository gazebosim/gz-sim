/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ForceTorque.hh"

#include <unordered_map>
#include <utility>
#include <string>

#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/ForceTorqueSensor.hh>

#include "ignition/gazebo/components/ForceTorque.hh"
#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private ForceTorque data class.
class ignition::gazebo::systems::ForceTorquePrivate
{
  /// \brief A map of FT entity to its FT sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ignition::sensors::ForceTorqueSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  public: Entity worldEntity = kNullEntity;

  /// \brief Create FT sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateForceTorqueEntities(EntityComponentManager &_ecm);

  /// \brief Update FT sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove FT sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveForceTorqueEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
ForceTorque::ForceTorque() : System(), dataPtr(std::make_unique<ForceTorquePrivate>())
{
}

//////////////////////////////////////////////////
ForceTorque::~ForceTorque() = default;

//////////////////////////////////////////////////
void ForceTorque::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ForceTorque::PreUpdate");
  this->dataPtr->CreateForceTorqueEntities(_ecm);
}

//////////////////////////////////////////////////
void ForceTorque::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  IGN_PROFILE("ForceTorque::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          math::secNsecToDuration(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveForceTorqueEntities(_ecm);
}

//////////////////////////////////////////////////
void ForceTorquePrivate::CreateForceTorqueEntities(EntityComponentManager &_ecm)
{
  // Create FT Sensors
  _ecm.EachNew<components::ForceTorque>(
    [&](const Entity &_entity,
        const components::ForceTorque *_ft)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _ft->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/forcetorque";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::ForceTorqueSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::ForceTorqueSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create sensor [" << sensorScopedName << "]"
                 << std::endl;
          return true;
        }


      std::cout << "SENSOR CREATED" << std::endl;

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ForceTorquePrivate::Update(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("ForceTorquePrivate::Update");
  _ecm.Each<components::ForceTorque,
            components::JointForce>(
    [&](const Entity &_entity,
        const components::ForceTorque * /*_ft*/,
        const components::JointForce *_jointForce)->bool
      {

        std::cout << "VALUE: " << _jointForce->Data()[0] <<std::endl;
        // auto it = this->entitySensorMap.find(_entity);
        // if (it != this->entitySensorMap.end())
        // {
        //   // Set the IMU angular velocity (defined in imu's local frame)
        //   it->second->SetForce(ignition::math::Vector3d(_jointForce->Data()[0], _jointForce->Data()[1], _jointForce->Data()[2]));

        //   // // Set the IMU linear acceleration in the imu local frame
        //   // it->second->SetTorque(_torque);
        //  }
        // else
        // {
        //   ignerr << "Failed to update Force/Torque Sensor: " << _entity << ". "
        //          << "Entity not found." << std::endl;
        // }

        return true;
      });
}

//////////////////////////////////////////////////
void ForceTorquePrivate::RemoveForceTorqueEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("ForceTorquePrivate::RemoveForceTorqueEntities");
  _ecm.EachRemoved<components::ForceTorque>(
    [&](const Entity &_entity,
        const components::ForceTorque *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing FT sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(ForceTorque, System,
  ForceTorque::ISystemPreUpdate,
  ForceTorque::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(ForceTorque, "ignition::gazebo::systems::ForceTorque")
