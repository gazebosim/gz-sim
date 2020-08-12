/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/msgs/logical_camera_image.pb.h>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/LogicalCameraSensor.hh>

#include "ignition/gazebo/components/LogicalCamera.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "LogicalCamera.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private LogicalCamera data class.
class ignition::gazebo::systems::LogicalCameraPrivate
{
  /// \brief A map of logicalCamera entities
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::LogicalCameraSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create logicalCamera sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateLogicalCameraEntities(EntityComponentManager &_ecm);

  /// \brief Update logicalCamera sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateLogicalCameras(const EntityComponentManager &_ecm);

  /// \brief Remove logicalCamera sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveLogicalCameraEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
LogicalCamera::LogicalCamera() : System(),
    dataPtr(std::make_unique<LogicalCameraPrivate>())
{
}

//////////////////////////////////////////////////
LogicalCamera::~LogicalCamera() = default;

//////////////////////////////////////////////////
void LogicalCamera::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogicalCamera::PreUpdate");
  this->dataPtr->CreateLogicalCameraEntities(_ecm);
}

//////////////////////////////////////////////////
void LogicalCamera::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogicalCamera::PostUpdate");

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
    this->dataPtr->UpdateLogicalCameras(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update sensor
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          common::Time(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveLogicalCameraEntities(_ecm);
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::CreateLogicalCameraEntities(
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogicalCameraPrivate::CreateLogicalCameraEntities");
  // Create logicalCameras
  _ecm.EachNew<components::LogicalCamera, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::LogicalCamera *_logicalCamera,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        auto data = _logicalCamera->Data()->Clone();
        data->GetAttribute("name")->Set(sensorScopedName);
        // check topic
        if (!data->HasElement("topic"))
        {
          std::string topic = scopedName(_entity, _ecm) + "/logical_camera";
          data->GetElement("topic")->Set(topic);
        }
        std::unique_ptr<sensors::LogicalCameraSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::LogicalCameraSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create sensor [" << sensorScopedName << "]"
                 << std::endl;
          return true;
        }

        // set sensor parent
        std::string parentName = _ecm.Component<components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // set sensor world pose
        math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
        sensor->SetPose(sensorWorldPose);

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::UpdateLogicalCameras(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogicalCameraPrivate::UpdateLogicalCameras");
  std::map<std::string, math::Pose3d> modelPoses;

  _ecm.Each<components::Model, components::Name, components::Pose>(
      [&](const Entity &,
        const components::Model *,
        const components::Name *_name,
        const components::Pose *_pose)->bool
      {
        /// todo(anyone) We currently assume there are only top level models
        /// Update to retrieve world pose when nested models are supported.
        modelPoses[_name->Data()] = _pose->Data();
        return true;
      });


  _ecm.Each<components::LogicalCamera, components::WorldPose>(
    [&](const Entity &_entity,
        const components::LogicalCamera * /*_logicalCamera*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const math::Pose3d &worldPose = _worldPose->Data();
          it->second->SetPose(worldPose);
          it->second->SetModelPoses(std::move(modelPoses));
        }
        else
        {
          ignerr << "Failed to update logicalCamera: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::RemoveLogicalCameraEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogicalCameraPrivate::RemoveLogicalCameraEntities");
  _ecm.EachRemoved<components::LogicalCamera>(
    [&](const Entity &_entity,
        const components::LogicalCamera *)->bool
      {
        auto sensorIt = this->entitySensorMap.find(_entity);
        if (sensorIt == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing logicalCamera sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorIt);

        return true;
      });
}

IGNITION_ADD_PLUGIN(LogicalCamera, System,
  LogicalCamera::ISystemPreUpdate,
  LogicalCamera::ISystemPostUpdate
)


IGNITION_ADD_PLUGIN_ALIAS(LogicalCamera,
    "ignition::gazebo::systems::LogicalCamera")
