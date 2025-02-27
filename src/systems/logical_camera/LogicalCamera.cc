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

#include "LogicalCamera.hh"

#include <gz/msgs/logical_camera_image.pb.h>

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/LogicalCameraSensor.hh>

#include "gz/sim/components/LogicalCamera.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private LogicalCamera data class.
class gz::sim::systems::LogicalCameraPrivate
{
  /// \brief A map of logicalCamera entities
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::LogicalCameraSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _logicalCamera LogicalCamera component.
  /// \param[in] _parent Parent entity component.
  public: void AddLogicalCamera(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::LogicalCamera *_logicalCamera,
    const components::ParentEntity *_parent);

  /// \brief Create logicalCamera sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

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
  GZ_PROFILE("LogicalCamera::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void LogicalCamera::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LogicalCamera::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the LogicalCameraPrivate::UpdateLogicalCameras
    // function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->UpdateLogicalCameras(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update sensor
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveLogicalCameraEntities(_ecm);
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::AddLogicalCamera(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::LogicalCamera *_logicalCamera,
  const components::ParentEntity *_parent)
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
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // set sensor world pose
  math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
  sensor->SetPose(sensorWorldPose);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LogicalCameraPrivate::CreateLogicalCameraEntities");
  if (!this->initialized)
  {
    // Create logicalCameras
    _ecm.Each<components::LogicalCamera, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::LogicalCamera *_logicalCamera,
          const components::ParentEntity *_parent)->bool
        {
          this->AddLogicalCamera(_ecm, _entity, _logicalCamera, _parent);
          return true;
        });
    this->initialized = true;

  }
  else
  {
    // Create logicalCameras
    _ecm.EachNew<components::LogicalCamera, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::LogicalCamera *_logicalCamera,
          const components::ParentEntity *_parent)->bool
        {
          this->AddLogicalCamera(_ecm, _entity, _logicalCamera, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::UpdateLogicalCameras(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LogicalCameraPrivate::UpdateLogicalCameras");
  std::map<std::string, math::Pose3d> modelPoses;

  _ecm.Each<components::Model, components::Name>(
    [&](const Entity &_entity,
      const components::Model *,
      const components::Name *_name)->bool
    {
      modelPoses[_name->Data()] = worldPose(_entity, _ecm);
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
          // Make a copy of modelPoses s.t. SetModelPoses can take ownership
          auto modelPoses_ = modelPoses;
          it->second->SetModelPoses(std::move(modelPoses_));
        }
        else
        {
          gzerr << "Failed to update logicalCamera: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void LogicalCameraPrivate::RemoveLogicalCameraEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LogicalCameraPrivate::RemoveLogicalCameraEntities");
  _ecm.EachRemoved<components::LogicalCamera>(
    [&](const Entity &_entity,
        const components::LogicalCamera *)->bool
      {
        auto sensorIt = this->entitySensorMap.find(_entity);
        if (sensorIt == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing logicalCamera sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorIt);

        return true;
      });
}

GZ_ADD_PLUGIN(LogicalCamera, System,
  LogicalCamera::ISystemPreUpdate,
  LogicalCamera::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(LogicalCamera,
    "gz::sim::systems::LogicalCamera")
