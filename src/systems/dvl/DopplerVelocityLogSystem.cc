/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/common/VideoEncoder.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/rendering/Events.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/sensors/DopplerVelocityLog.hh>
#include <gz/sensors/Manager.hh>

#include "DopplerVelocityLogSystem.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

namespace requests
{

/// \brief A request for sensor creation.
struct CreateSensor
{
  sdf::Sensor sdf;
  gz::sim::Entity entity;
  gz::sim::Entity parent;
  std::string parentName;
};

/// \brief A request for sensor destruction.
struct DestroySensor
{
  gz::sim::Entity entity;
};

/// \brief A request for a world state update for sensors.
struct SetWorldState
{
  gz::sensors::WorldState worldState;
};

/// \brief A request for an environmental data update for sensors.
struct SetEnvironmentalData
{
  std::shared_ptr<gz::sensors::EnvironmentalData> environmentalData;
};

/// \brief Union request type.
using SomeRequest = std::variant<
  CreateSensor, DestroySensor,
  SetWorldState, SetEnvironmentalData>;

}  // namespace requests

// Private data class.
class DopplerVelocityLogSystem::Implementation
{
  /// \brief Callback invoked in the rendering thread before a rendering update
  public: void OnPreRender();

  /// \brief Callback invoked in the rendering thread during a rendering update
  public: void OnRender();

  /// \brief Callback invoked in the rendering thread after a rendering update
  public: void OnPostRender();

  /// \brief Callback invoked in the rendering thread before stopping
  public: void OnRenderTeardown();

  /// \brief Overload to handle sensor creation requests.
  public: void Handle(requests::CreateSensor _request);

  /// \brief Overload to handle sensor destruction requests.
  public: void Handle(requests::DestroySensor _request);

  /// \brief Overload to handle world state update requests.
  public: void Handle(requests::SetWorldState _request);

  /// \brief Overload to handle environment data update requests.
  public: void Handle(requests::SetEnvironmentalData _request);

  /// \brief Implementation for Configure() hook.
  public: void DoConfigure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr);

  /// \brief Implementation for PreUpdate() hook.
  public: void DoPreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm);

  /// \brief Implementation for Update() hook.
  public: void DoUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm);

  /// \brief Implementation for PostUpdate() hook.
  public: void DoPostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);

  /// \brief State of all entities in the world in simulation thread
  public: std::optional<gz::sensors::WorldState> latestWorldState;

  /// \brief State of all entities in the world in simulation thread
  public: std::shared_ptr<
    gz::sensors::EnvironmentalData> latestEnvironmentalData;

  /// \brief Connection to the post-render event.
  public: gz::common::ConnectionPtr preRenderConn;

  /// \brief Connection to the render event.
  public: gz::common::ConnectionPtr renderConn;

  /// \brief Connection to the post-render event.
  public: gz::common::ConnectionPtr postRenderConn;

  /// \brief Connection to the render teardown event.
  public: gz::common::ConnectionPtr renderTeardownConn;

  /// \brief Pointer to the event manager
  public: gz::sim::EventManager *eventMgr = nullptr;

  //// \brief Pointer to the rendering scene
  public: gz::rendering::ScenePtr scene;

  /// \brief Sensor managers
  public: gz::sensors::Manager sensorManager;

  /// \brief Entities of known sensors (in simulation thread)
  public: std::unordered_set<gz::sim::Entity> knownSensorEntities;

  /// \brief Sensor ID per sensor entity mapping in rendering thread
  public: std::unordered_map<
    gz::sim::Entity, gz::sensors::SensorId> sensorIdPerEntity;

  /// \brief IDs of sensors updated in the last rendering pass
  public: std::vector<gz::sensors::SensorId> updatedSensorIds;

  /// \brief Queue of requests from simulation thread to rendering thread
  public: std::vector<requests::SomeRequest> perStepRequests;

  /// \brief Mutex to synchronize access to queued requests
  public: std::mutex requestsMutex;

  /// \brief Requests queue, popped by the rendering thread.
  public: std::vector<requests::SomeRequest> queuedRequests;

  /// \brief Flag for pending (ie. queued) requests.
  public: std::atomic<bool> pendingRequests{false};

  /// \brief Flag for
  public: bool needsUpdate;

  /// \brief Current simulation time.
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Current simulation time.
  public: std::chrono::steady_clock::duration nextUpdateTime{
    std::chrono::steady_clock::duration::max()};
};

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::DoConfigure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &_eventMgr)
{
  this->preRenderConn =
      _eventMgr.Connect<gz::sim::events::PreRender>(
          std::bind(&DopplerVelocityLogSystem::Implementation::OnPreRender, this));

  this->renderConn =
      _eventMgr.Connect<gz::sim::events::Render>(
          std::bind(&DopplerVelocityLogSystem::Implementation::OnRender, this));

  this->postRenderConn =
      _eventMgr.Connect<gz::sim::events::PostRender>(
          std::bind(&DopplerVelocityLogSystem::Implementation::OnPostRender, this));

  this->renderTeardownConn =
      _eventMgr.Connect<gz::sim::events::RenderTeardown>(
          std::bind(&DopplerVelocityLogSystem::Implementation::OnRenderTeardown, this));

  this->eventMgr = &_eventMgr;
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::DoPreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::Environment>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Environment *_env) -> bool
    {
      if (_entity == gz::sim::worldEntity(_ecm))
      {
        this->perStepRequests.push_back(
          requests::SetEnvironmentalData{_env->Data()});
      }
      return true;
    });

  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent) -> bool
    {
      using namespace gz::sim;
      // Get sensor's scoped name without the world
      std::string sensorScopedName = removeParentScope(
          scopedName(_entity, _ecm, "::", false), "::");

      // Check sensor's type before proceeding
      sdf::Sensor sdf = _custom->Data();
      sdf::ElementPtr root = sdf.Element();
      if (!root->HasAttribute("gz:type"))
      {
        gzmsg << "No 'gz:type' attribute in custom sensor "
               << "[" << sensorScopedName << "]. Ignoring."
               << std::endl;
        return true;
      }
      auto type = root->Get<std::string>("gz:type");
      if (type != "dvl")
      {
        gzdbg << "Found custom sensor [" << sensorScopedName << "]"
               << " of '" << type << "' type. Ignoring." << std::endl;
        return true;
      }
      gzdbg << "Found custom sensor [" << sensorScopedName << "]"
             << " of '" << type << "' type!" << std::endl;

      sdf.SetName(sensorScopedName);

      if (sdf.Topic().empty())
      {
        // Default to scoped name as topic
        sdf.SetTopic(scopedName(_entity, _ecm) + "/dvl/velocity");
      }

      auto parentName =
          _ecm.Component<components::Name>(_parent->Data());

      enableComponent<components::WorldPose>(_ecm, _entity);
      enableComponent<components::WorldAngularVelocity>(_ecm, _entity);
      enableComponent<components::WorldLinearVelocity>(_ecm, _entity);

      this->perStepRequests.push_back(requests::CreateSensor{
          sdf, _entity, _parent->Data(), parentName->Data()});

      this->knownSensorEntities.insert(_entity);
      return true;
    });
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::DoPostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)
    {
      if (this->knownSensorEntities.count(_entity))
      {
        this->perStepRequests.push_back(
            requests::DestroySensor{_entity});
        this->knownSensorEntities.erase(_entity);
      }
      return true;
    });

  const auto [sec, nsec] =
      gz::math::durationToSecNsec(_info.simTime);
  this->simTime = gz::math::secNsecToDuration(sec, nsec);

  if (!this->perStepRequests.empty() || (
        !_info.paused && this->nextUpdateTime <= this->simTime))
  {
    requests::SetWorldState request;
    auto component = _ecm.Component<
      gz::sim::components::SphericalCoordinates
    >(gz::sim::worldEntity(_ecm));
    if (component)
    {
      request.worldState.origin = component->Data();
    }

    _ecm.Each<gz::sim::components::WorldPose,
              gz::sim::components::WorldLinearVelocity,
              gz::sim::components::WorldAngularVelocity>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::WorldPose *_pose,
          const gz::sim::components::WorldLinearVelocity *_linearVelocity,
          const gz::sim::components::WorldAngularVelocity *_angularVelocity)
      {
        auto & kinematicState = request.worldState.kinematics[_entity];
        kinematicState.pose = _pose->Data();
        kinematicState.linearVelocity = _linearVelocity->Data();
        kinematicState.angularVelocity = _angularVelocity->Data();
        return true;
      });;

    {
      std::lock_guard<std::mutex> lock(this->requestsMutex);
      this->queuedRequests.insert(
          this->queuedRequests.end(),
          std::make_move_iterator(this->perStepRequests.begin()),
          std::make_move_iterator(this->perStepRequests.end()));
      this->queuedRequests.push_back(std::move(request));
      this->perStepRequests.clear();
    }

    this->pendingRequests = true;

    this->eventMgr->Emit<gz::sim::events::ForceRender>();
  }
}

namespace
{

gz::rendering::VisualPtr findEntityVisual(
    gz::rendering::ScenePtr scene, gz::sim::Entity entity)
{
  for (unsigned int i = 0; i < scene->VisualCount(); ++i)
  {
    gz::rendering::VisualPtr visual = scene->VisualByIndex(i);
    if (visual->HasUserData("gazebo-entity"))
    {
      auto user_data = visual->UserData("gazebo-entity");
      if (entity == std::get<uint64_t>(user_data))
      {
        return visual;
      }
    }
  }
  return gz::rendering::VisualPtr();
}

}  // namespace

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::Handle(
    requests::CreateSensor _request)
{
  auto *sensor =
      this->sensorManager.CreateSensor<
          gz::sensors::DopplerVelocityLog>(_request.sdf);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor "
           << "[" << _request.sdf.Name() << "]"
           << std::endl;
    return;
  }

  sensor->SetEntity(_request.entity);
  sensor->SetParent(_request.parentName);

  // Set the scene so it can create the rendering sensor
  sensor->SetScene(this->scene);
  sensor->SetManualSceneUpdate(true);

  if (this->latestWorldState)
  {
    sensor->SetWorldState(*this->latestWorldState);
  }

  if (this->latestEnvironmentalData)
  {
    sensor->SetEnvironmentalData(*this->latestEnvironmentalData);
  }

  gz::rendering::VisualPtr parentVisual =
      findEntityVisual(this->scene, _request.parent);
  if (!parentVisual)
  {
    gzerr << "Failed to find parent visual for sensor "
           << "[" << _request.sdf.Name() << "]" << std::endl;
    if (!this->sensorManager.Remove(sensor->Id()))
    {
      gzerr << "Internal error, missing sensor "
             << "[" << _request.sdf.Name() << "]"
             << std::endl;
    }
    return;
  }
  for (auto renderingSensor : sensor->RenderingSensors())
  {
    parentVisual->AddChild(renderingSensor);
  }

  // Track sensor id for this sensor entity
  this->sensorIdPerEntity.insert({_request.entity, sensor->Id()});

  // Force (first) sensor update
  this->needsUpdate = true;
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::Handle(
    requests::DestroySensor _request)
{
  auto it = this->sensorIdPerEntity.find(_request.entity);
  if (it != this->sensorIdPerEntity.end())
  {
    auto *sensor = dynamic_cast<gz::sensors::DopplerVelocityLog *>(
        this->sensorManager.Sensor(it->second));
    if (sensor)
    {
      for (auto renderingSensor : sensor->RenderingSensors())
      {
        renderingSensor->RemoveParent();
      }
      this->sensorManager.Remove(it->second);
    }
    else
    {
      gzerr << "Internal error, missing DVL sensor for entity "
             << "[" << _request.entity << "]" << std::endl;
    }
    this->sensorIdPerEntity.erase(it);
  }
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::Handle(
    requests::SetWorldState _request)
{
  this->latestWorldState = std::move(_request.worldState);
  for (const auto& [_, sensorId] : this->sensorIdPerEntity)
  {
    auto *sensor = dynamic_cast<gz::sensors::DopplerVelocityLog *>(
        this->sensorManager.Sensor(sensorId));
    sensor->SetWorldState(*this->latestWorldState);
  }
  this->needsUpdate = true;
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::Handle(
    requests::SetEnvironmentalData _request)
{
  this->latestEnvironmentalData = std::move(_request.environmentalData);
  for (const auto& [_, sensorId] : this->sensorIdPerEntity)
  {
    auto *sensor = dynamic_cast<gz::sensors::DopplerVelocityLog *>(
        this->sensorManager.Sensor(sensorId));
    sensor->SetEnvironmentalData(*this->latestEnvironmentalData);
  }
  this->needsUpdate = true;
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::OnPreRender()
{
  GZ_PROFILE("DopplerVelocityLogSystem::Implementation::OnPreRender");
  if (!this->scene)
  {
    this->scene = gz::rendering::sceneFromFirstRenderEngine();
  }

  if (this->pendingRequests.exchange(false))
  {
    std::vector<requests::SomeRequest> requests;
    {
      std::lock_guard<std::mutex> lock(this->requestsMutex);
      requests.insert(requests.end(),
        std::make_move_iterator(this->queuedRequests.begin()),
        std::make_move_iterator(this->queuedRequests.end()));
      this->queuedRequests.clear();
    }
    for (auto &request : requests)
    {
      std::visit([this](auto & req) {
        this->Handle(std::move(req));
      }, request);
    }
  }
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::OnRender()
{
  GZ_PROFILE("DopplerVelocityLogSystem::Implementation::OnRender");
  if (!this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    return;
  }

  if (this->needsUpdate)
  {
    auto closestUpdateTime = std::chrono::steady_clock::duration::max();
    for (const auto & [_, sensorId] : this->sensorIdPerEntity)
    {
      gz::sensors::Sensor *sensor =
          this->sensorManager.Sensor(sensorId);

      constexpr bool kForce = true;
      if (sensor->Update(this->simTime, !kForce))
      {
        this->updatedSensorIds.push_back(sensorId);
      }

      closestUpdateTime = std::min(
          sensor->NextDataUpdateTime(), closestUpdateTime);
    }
    this->nextUpdateTime = closestUpdateTime;

    this->needsUpdate = false;
  }
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::OnPostRender()
{
  GZ_PROFILE("DopplerVelocityLogSystem::Implementation::OnPostRender");
  for (const auto & sensorId : this->updatedSensorIds)
  {
    auto *sensor =
        dynamic_cast<gz::sensors::DopplerVelocityLog *>(
            this->sensorManager.Sensor(sensorId));
    sensor->PostUpdate(this->simTime);
  }
  this->updatedSensorIds.clear();
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Implementation::OnRenderTeardown()
{
  GZ_PROFILE("DopplerVelocityLogSystem::Implementation::OnRenderTeardown");
  for (const auto & [entityId, sensorId] : this->sensorIdPerEntity)
  {
    auto *sensor = dynamic_cast<gz::sensors::DopplerVelocityLog *>(
        this->sensorManager.Sensor(sensorId));
    if (sensor)
    {
      for (auto renderingSensor : sensor->RenderingSensors())
      {
        renderingSensor->RemoveParent();
      }
      this->sensorManager.Remove(sensorId);
    }
    else
    {
      gzerr << "Internal error, missing DVL sensor for entity "
             << "[" << entityId << "]" << std::endl;
    }
  }
  this->sensorIdPerEntity.clear();
}

//////////////////////////////////////////////////
DopplerVelocityLogSystem::DopplerVelocityLogSystem() :
    dataPtr(new Implementation())
{
}

//////////////////////////////////////////////////
DopplerVelocityLogSystem::~DopplerVelocityLogSystem()
{
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
  GZ_PROFILE("DopplerVelocityLogSystem::Configure");
  this->dataPtr->DoConfigure(_entity, _sdf, _ecm, _eventMgr);
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("DopplerVelocityLogSystem::PreUpdate");
  this->dataPtr->DoPreUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void DopplerVelocityLogSystem::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("DopplerVelocityLogSystem::PostUpdate");
  this->dataPtr->DoPostUpdate(_info, _ecm);

}

GZ_ADD_PLUGIN(DopplerVelocityLogSystem,
  System,
  ISystemConfigure,
  ISystemPreUpdate,
  ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(DopplerVelocityLogSystem,
  "gz::sim:systems::DopplerVelocityLogSystem"
)

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
