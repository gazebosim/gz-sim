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

#include "SceneBroadcaster.hh"

#include <gz/msgs/camerasensor.pb.h>
#include <gz/msgs/distortion.pb.h>
#include <gz/msgs/geometry.pb.h>
#include <gz/msgs/imu_sensor.pb.h>
#include <gz/msgs/lidar_sensor.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/link.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/projector.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/sensor.pb.h>
#include <gz/msgs/sensor_noise.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/uint32_v.pb.h>
#include <gz/msgs/visual.pb.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <gz/common/Profiler.hh>
#include <gz/math/graph/Graph.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/AirSpeedSensor.hh"
#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/CastShadows.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/LaserRetro.hh"
#include "gz/sim/components/Lidar.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/LogicalCamera.hh"
#include "gz/sim/components/LogPlaybackStatistics.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParticleEmitter.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Projector.hh"
#include "gz/sim/components/RgbdCamera.hh"
#include "gz/sim/components/Scene.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"

#include <sdf/Camera.hh>
#include <sdf/Imu.hh>
#include <sdf/Lidar.hh>
#include <sdf/Noise.hh>
#include <sdf/Scene.hh>
#include <sdf/Sensor.hh>

using namespace std::chrono_literals;

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class.
class gz::sim::systems::SceneBroadcasterPrivate
{
  /// \brief Type alias for the graph used to represent the scene graph.
  public: using SceneGraphType = math::graph::DirectedGraph<
          std::shared_ptr<google::protobuf::Message>, bool>;

  /// \brief Setup Gazebo Transport services and publishers
  /// \param[in] _worldName Name of world.
  public: void SetupTransport(const std::string &_worldName);

  /// \brief Callback for scene info service.
  /// \param[out] _res Response containing the latest scene message.
  /// \return True if successful.
  public: bool SceneInfoService(msgs::Scene &_res);

  /// \brief Callback for scene graph service.
  /// \param[out] _res Response containing the the scene graph in DOT format.
  /// \return True if successful.
  public: bool SceneGraphService(msgs::StringMsg &_res);

  /// \brief Callback for state service.
  /// \param[out] _res Response containing the latest full state.
  /// \return True if successful.
  public: bool StateService(msgs::SerializedStepMap &_res);

  /// \brief Callback for state service - non blocking.
  /// \param[out] _res Response containing the last available full state.
  public: void StateAsyncService(const msgs::StringMsg &_req);

  /// \brief Updates the scene graph when entities are added
  /// \param[in] _manager The entity component manager
  public: void SceneGraphAddEntities(const EntityComponentManager &_manager);

  /// \brief Updates the scene graph when entities are removed
  /// \param[in] _manager The entity component manager
  public: void SceneGraphRemoveEntities(const EntityComponentManager &_manager);

  /// \brief Adds models to a msgs::Scene or msgs::Model object based on the
  /// contents of the scene graph
  /// \tparam T Either a msgs::Scene or msgs::Model
  /// \param[in] _msg Pointer to msg object to which the models will be added
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: template <typename T>
          static void AddModels(T *_msg, const Entity _entity,
                                const SceneGraphType &_graph);

  /// \brief Adds lights to a msgs::Scene or msgs::Link object based on the
  /// contents of the scene graph
  /// \tparam T Either a msgs::Scene or msgs::Link
  /// \param[in] _msg Pointer to msg object to which the lights will be added
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: template<typename T>
          static void AddLights(T *_msg, const Entity _entity,
                                const SceneGraphType &_graph);

  /// \brief Adds links to a msgs::Model object based on the contents of
  /// the scene graph
  /// \param[in] _msg Pointer to msg object to which the links will be added
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddLinks(msgs::Model *_msg, const Entity _entity,
                               const SceneGraphType &_graph);

  /// \brief Adds visuals to a msgs::Link object based on the contents of
  /// the scene graph
  /// \param[in] _msg Pointer to msg object to which the visuals will be added
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddVisuals(msgs::Link *_msg, const Entity _entity,
                                 const SceneGraphType &_graph);

  /// \brief Adds sensors to a msgs::Link object based on the contents of
  /// the scene graph
  /// \param[inout] _msg Pointer to msg object to which the sensors will be
  /// added.
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddSensors(msgs::Link *_msg, const Entity _entity,
                                 const SceneGraphType &_graph);

  /// \brief Adds particle emitters to a msgs::Link object based on the
  /// contents of the scene graph
  /// \param[inout] _msg Pointer to msg object to which the particle
  /// emitters will be added.
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddParticleEmitters(msgs::Link *_msg,
              const Entity _entity, const SceneGraphType &_graph);

  /// \brief Adds projectors to a msgs::Link object based on the
  /// contents of the scene graph
  /// \param[inout] _msg Pointer to msg object to which the projectors
  /// will be added.
  /// \param[in] _entity Parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddProjectors(msgs::Link *_msg,
              const Entity _entity, const SceneGraphType &_graph);

  /// \brief Recursively remove entities from the graph
  /// \param[in] _entity Entity
  /// \param[in/out] _graph Scene graph
  public: static void RemoveFromGraph(const Entity _entity,
                                      SceneGraphType &_graph);

  /// \brief Create and send out pose updates.
  /// \param[in] _info The update information
  /// \param[in] _manager The entity component manager
  public: void PoseUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_manager);

  /// \brief Transport node.
  public: std::unique_ptr<transport::Node> node{nullptr};

  /// \brief Pose publisher.
  public: transport::Node::Publisher posePub;

  /// \brief Dynamic pose publisher, for non-static model poses
  public: transport::Node::Publisher dyPosePub;

  /// \brief Rate at which to publish dynamic poses
  public: int dyPoseHertz{60};

  /// \brief Scene publisher
  public: transport::Node::Publisher scenePub;

  /// \brief Request publisher.
  /// This is used to request entities to be removed
  public: transport::Node::Publisher deletionPub;

  /// \brief State publisher
  public: transport::Node::Publisher statePub;

  /// \brief Graph containing latest information from entities.
  /// The data in each node is the message associated with that entity only.
  /// i.e, a model node only has a message about the model. It will not
  /// have any links, joints, etc. To create a the whole scene, one has to
  /// traverse the graph adding messages as necessary.
  public: SceneGraphType sceneGraph;

  /// \brief Keep the id of the world entity so we know how to traverse the
  /// graph.
  public: Entity worldEntity{kNullEntity};

  /// \brief Keep the name of the world entity so it's easy to create temporary
  /// scene graphs
  public: std::string worldName;

  /// \brief Protects scene graph.
  public: std::mutex graphMutex;

  /// \brief Protects stepMsg.
  public: std::mutex stateMutex;

  /// \brief Used to coordinate the state service response.
  public: std::condition_variable stateCv;

  /// \brief Filled on demand for the state service.
  public: msgs::SerializedStepMap stepMsg;

  /// \brief Last time the state was published.
  public: std::chrono::time_point<std::chrono::system_clock>
      lastStatePubTime{std::chrono::system_clock::now()};

  /// \brief Period to publish state while paused and running. The key for
  /// the map is used to store the publish period when paused and not
  /// paused. The not-paused (a.ka. running) period has a key=false and a
  /// default update rate of 60Hz. The paused period has a key=true and a
  /// default update rate of 30Hz.
  public: std::map<bool, std::chrono::duration<double, std::ratio<1, 1000>>>
      statePublishPeriod{
        {false, std::chrono::duration<double,
        std::ratio<1, 1000>>(1000/60.0)},
          {true,  std::chrono::duration<double,
            std::ratio<1, 1000>>(1000/30.0)}};

  /// \brief Flag used to indicate if the state service was called.
  public: bool stateServiceRequest{false};

  /// \brief A list of async state requests
  public: std::unordered_set<std::string> stateRequests;

  /// \brief Store SDF scene information so that it can be inserted into
  /// scene message.
  public: sdf::Scene sdfScene;

  /// \brief Flag used to indicate if periodic changes need to be published
  /// This is currently only used in playback mode.
  public: bool pubPeriodicChanges{false};

  /// \brief Stores a cache of components that are changed. (This prevents
  ///  dropping of periodic change components which may not be updated
  ///  frequently enough)
  public: std::unordered_map<ComponentTypeId,
    std::unordered_set<Entity>> changedComponents;
};


//////////////////////////////////////////////////
SceneBroadcaster::SceneBroadcaster()
  : System(), dataPtr(std::make_unique<SceneBroadcasterPrivate>())
{
}

//////////////////////////////////////////////////
void SceneBroadcaster::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> & _sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  // World
  const components::Name *name = _ecm.Component<components::Name>(_entity);
  if (name == nullptr)
  {
    gzerr << "World with id: " << _entity
           << " has no name. SceneBroadcaster cannot create transport topics\n";
    return;
  }

  this->dataPtr->worldEntity = _entity;
  this->dataPtr->worldName = name->Data();

  auto readHertz = _sdf->Get<int>("dynamic_pose_hertz", 60);
  this->dataPtr->dyPoseHertz = readHertz.first;

  auto stateHertz = _sdf->Get<double>("state_hertz", 60);
  if (stateHertz.first > 0.0)
  {
    this->dataPtr->statePublishPeriod[false] =
        std::chrono::duration<double, std::ratio<1, 1000>>(
            1000 / stateHertz.first);

    // Set the paused update rate to half of the running update rate.
    this->dataPtr->statePublishPeriod[true] =
        std::chrono::duration<double, std::ratio<1, 1000>>(1000/
            (stateHertz.first * 0.5));
  }
  else
  {
    using secs_double = std::chrono::duration<double, std::ratio<1>>;
    gzerr << "SceneBroadcaster state_hertz must be positive, using default ("
      << 1.0 / secs_double(this->dataPtr->statePublishPeriod[false]).count() <<
        "Hz)\n";
  }

  // Add to graph
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->graphMutex);
    this->dataPtr->sceneGraph.AddVertex(this->dataPtr->worldName, nullptr,
                                        this->dataPtr->worldEntity);
  }
}

//////////////////////////////////////////////////
void SceneBroadcaster::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_manager)
{
  GZ_PROFILE("SceneBroadcaster::PostUpdate");

  // Update scene graph with added entities before populating pose message
  if (_manager.HasNewEntities())
    this->dataPtr->SceneGraphAddEntities(_manager);

  // Store the Scene component data, which holds sdf::Scene so that we can
  // populate the scene info messages.
  auto sceneComp =
    _manager.Component<components::Scene>(this->dataPtr->worldEntity);
  if (sceneComp)
    this->dataPtr->sdfScene = sceneComp->Data();

  // Create and send pose update if transport connections exist.
  if (this->dataPtr->dyPosePub.HasConnections() ||
      this->dataPtr->posePub.HasConnections())
  {
    this->dataPtr->PoseUpdate(_info, _manager);
  }

  // call SceneGraphRemoveEntities at the end of this update cycle so that
  // removed entities are removed from the scene graph for the next update cycle
  this->dataPtr->SceneGraphRemoveEntities(_manager);

  // Iterate through entities and their changes to cache them.
  _manager.UpdatePeriodicChangeCache(this->dataPtr->changedComponents);

  // Publish state only if there are subscribers and
  // * throttle rate to 60 Hz
  // * also publish off-rate if there are change events:
  //     * new / erased entities
  //     * components with one-time changes
  //     * jump back in time
  // Throttle here instead of using transport::AdvertiseMessageOptions so that
  // we can skip the ECM serialization
  bool jumpBackInTime = _info.dt < std::chrono::steady_clock::duration::zero();
  bool changeEvent = _manager.HasEntitiesMarkedForRemoval() ||
    _manager.HasNewEntities() || _manager.HasOneTimeComponentChanges() ||
    jumpBackInTime || _manager.HasRemovedComponents();
  auto now = std::chrono::system_clock::now();
  bool itsPubTime = (now - this->dataPtr->lastStatePubTime >
       this->dataPtr->statePublishPeriod[_info.paused]);
  // check if we need to publish periodic changes in playback mode.
  bool pubChanges = this->dataPtr->pubPeriodicChanges &&
      _manager.HasPeriodicComponentChanges();
  auto shouldPublish = this->dataPtr->statePub.HasConnections() &&
       (changeEvent || itsPubTime || pubChanges);

  if (this->dataPtr->stateServiceRequest || shouldPublish)
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->stateMutex);
    this->dataPtr->stepMsg.Clear();

    set(this->dataPtr->stepMsg.mutable_stats(), _info);

    // Publish full state if it has been explicitly requested
    if (this->dataPtr->stateServiceRequest)
    {
      _manager.State(*this->dataPtr->stepMsg.mutable_state(), {}, {}, true);
    }
    // Publish the changed state if a change occurred to the ECS
    else if (changeEvent)
    {
      _manager.ChangedState(*this->dataPtr->stepMsg.mutable_state());
    }
    // Otherwise publish just periodic change components when running
    else if (!_info.paused)
    {
      GZ_PROFILE("SceneBroadcast::PostUpdate UpdateState");
      if (!_manager.HasPeriodicComponentChanges())
      {
        // log files may be recorded at lower rate than sim time step. So in
        // playback mode, the scene broadcaster may not see any periodic
        // changed states here since it no longer happens every iteration.
        // As the result, no state changes are published to be GUI, causing
        // visuals in the GUI scene to miss updates. The visuals are only
        // updated if by some timing coincidence that log playback updates
        // the ECM at the same iteration as when the scene broadcaster is going
        // to publish perioidc changes here.
        // To work around the issue, we force the scene broadcaster
        // to publish states at an offcycle iteration the next time it sees
        // periodic changes.
        auto playbackComp =
            _manager.Component<components::LogPlaybackStatistics>(
            this->dataPtr->worldEntity);
        if (playbackComp)
        {
          this->dataPtr->pubPeriodicChanges = true;
        }
        // this creates an empty state in the msg even there are no periodic
        // changed components - done to preseve existing behavior.
        // we may be able to remove this in the future and update tests
        this->dataPtr->stepMsg.mutable_state();
      }

      // Apply changes that were caught by the periodic state tracker and then
      // clear the change tracker.
      _manager.PeriodicStateFromCache(*this->dataPtr->stepMsg.mutable_state(),
        this->dataPtr->changedComponents);
      this->dataPtr->changedComponents.clear();
    }

    // Full state on demand
    if (this->dataPtr->stateServiceRequest)
    {
      this->dataPtr->stateServiceRequest = false;
      this->dataPtr->stateCv.notify_all();
    }

    // process async state requests
    if (!this->dataPtr->stateRequests.empty())
    {
      for (const auto &reqSrv : this->dataPtr->stateRequests)
      {
        this->dataPtr->node->Request(reqSrv, this->dataPtr->stepMsg);
      }
      this->dataPtr->stateRequests.clear();
    }

    // Poses periodically + change events
    // TODO(louise) Send changed state periodically instead, once it reflects
    // changed components
    if (shouldPublish)
    {
      GZ_PROFILE("SceneBroadcast::PostUpdate Publish State");
      this->dataPtr->statePub.Publish(this->dataPtr->stepMsg);
      this->dataPtr->lastStatePubTime = now;
    }
  }
}

//////////////////////////////////////////////////
void SceneBroadcaster::Reset(const UpdateInfo &_info,
                             EntityComponentManager &_manager)
{
  // Run Post Update so that GUI will be refreshed if reset is called while
  // simulation is paused.
  this->PostUpdate(_info, _manager);
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::PoseUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_manager)
{
  GZ_PROFILE("SceneBroadcast::PoseUpdate");

  msgs::Pose_V poseMsg, dyPoseMsg;
  bool dyPoseConnections = this->dyPosePub.HasConnections();
  bool poseConnections = this->posePub.HasConnections();

  // Models
  _manager.Each<components::Model, components::Name, components::Pose,
                components::Static>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::Pose *_poseComp,
          const components::Static *_staticComp) -> bool
      {
        if (poseConnections)
        {
          // Add to pose msg
          auto pose = poseMsg.add_pose();
          msgs::Set(pose, _poseComp->Data());
          pose->set_name(_nameComp->Data());
          pose->set_id(_entity);
        }

        if (dyPoseConnections && !_staticComp->Data())
        {
          // Add to dynamic pose msg
          auto dyPose = dyPoseMsg.add_pose();
          msgs::Set(dyPose, _poseComp->Data());
          dyPose->set_name(_nameComp->Data());
          dyPose->set_id(_entity);
        }
        return true;
      });

  // Links
  _manager.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
      [&](const Entity &_entity, const components::Link *,
          const components::Name *_nameComp,
          const components::Pose *_poseComp,
          const components::ParentEntity *_parentComp) -> bool
      {
        // Add to pose msg
        if (poseConnections)
        {
          auto pose = poseMsg.add_pose();
          msgs::Set(pose, _poseComp->Data());
          pose->set_name(_nameComp->Data());
          pose->set_id(_entity);
        }

        // Check whether parent model is static
        auto staticComp = _manager.Component<components::Static>(
          _parentComp->Data());
        if (dyPoseConnections && !staticComp->Data())
        {
          // Add to dynamic pose msg
          auto dyPose = dyPoseMsg.add_pose();
          msgs::Set(dyPose, _poseComp->Data());
          dyPose->set_name(_nameComp->Data());
          dyPose->set_id(_entity);
        }

        return true;
      });

  if (dyPoseConnections)
  {
    // Set the time stamp in the header
    dyPoseMsg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));

    this->dyPosePub.Publish(dyPoseMsg);
  }

  // Visuals
  if (poseConnections)
  {
    poseMsg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));

    _manager.Each<components::Visual, components::Name, components::Pose>(
      [&](const Entity &_entity, const components::Visual *,
          const components::Name *_nameComp,
          const components::Pose *_poseComp) -> bool
      {
        // Add to pose msg
        auto pose = poseMsg.add_pose();
        msgs::Set(pose, _poseComp->Data());
        pose->set_name(_nameComp->Data());
        pose->set_id(_entity);
        return true;
      });

    // Lights
    _manager.Each<components::Light, components::Name, components::Pose>(
        [&](const Entity &_entity, const components::Light *,
            const components::Name *_nameComp,
            const components::Pose *_poseComp) -> bool
        {
          // Add to pose msg
          auto pose = poseMsg.add_pose();
          msgs::Set(pose, _poseComp->Data());
          pose->set_name(_nameComp->Data());
          pose->set_id(_entity);
          return true;
        });

    this->posePub.Publish(poseMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SetupTransport(const std::string &_worldName)
{
  auto ns = transport::TopicUtils::AsValidTopic("/world/" + _worldName);
  if (ns.empty())
  {
    gzerr << "Failed to create valid namespace for world [" << _worldName
           << "]" << std::endl;
    return;
  }

  transport::NodeOptions opts;
  opts.SetNameSpace(ns);
  this->node = std::make_unique<transport::Node>(opts);

  // Scene info service
  std::string infoService{"scene/info"};

  this->node->Advertise(infoService, &SceneBroadcasterPrivate::SceneInfoService,
      this);

  gzmsg << "Serving scene information on [" << opts.NameSpace() << "/"
         << infoService << "]" << std::endl;

  // Scene graph service
  std::string graphService{"scene/graph"};

  this->node->Advertise(graphService,
      &SceneBroadcasterPrivate::SceneGraphService, this);

  gzmsg << "Serving graph information on [" << opts.NameSpace() << "/"
         << graphService << "]" << std::endl;

  // State service
  // Note: GuiRunner used to call this service but it is now using the async
  // version (state_async)
  std::string stateService{"state"};

  this->node->Advertise(stateService, &SceneBroadcasterPrivate::StateService,
      this);

  gzmsg << "Serving full state on [" << opts.NameSpace() << "/"
         << stateService << "]" << std::endl;

  // Async State service
  std::string stateAsyncService{"state_async"};

  this->node->Advertise(stateAsyncService,
      &SceneBroadcasterPrivate::StateAsyncService, this);

  gzmsg << "Serving full state (async) on [" << opts.NameSpace() << "/"
         << stateAsyncService << "]" << std::endl;

  // Scene info topic
  std::string sceneTopic{ns + "/scene/info"};

  this->scenePub = this->node->Advertise<msgs::Scene>(sceneTopic);

  gzmsg << "Publishing scene information on [" << sceneTopic
         << "]" << std::endl;

  // Entity deletion publisher
  std::string deletionTopic{ns + "/scene/deletion"};

  this->deletionPub =
      this->node->Advertise<msgs::UInt32_V>(deletionTopic);

  gzmsg << "Publishing entity deletions on [" << deletionTopic << "]"
         << std::endl;

  // State topic
  std::string stateTopic{ns + "/state"};

  this->statePub =
      this->node->Advertise<msgs::SerializedStepMap>(stateTopic);

  gzmsg << "Publishing state changes on [" << stateTopic << "]"
      << std::endl;

  // Pose info publisher
  std::string poseTopic{"pose/info"};

  transport::AdvertiseMessageOptions poseAdvertOpts;
  poseAdvertOpts.SetMsgsPerSec(60);
  this->posePub = this->node->Advertise<msgs::Pose_V>(poseTopic,
      poseAdvertOpts);

  gzmsg << "Publishing pose messages on [" << opts.NameSpace() << "/"
         << poseTopic << "]" << std::endl;

  // Dynamic pose info publisher
  std::string dyPoseTopic{"dynamic_pose/info"};

  transport::AdvertiseMessageOptions dyPoseAdvertOpts;
  dyPoseAdvertOpts.SetMsgsPerSec(this->dyPoseHertz);
  this->dyPosePub = this->node->Advertise<msgs::Pose_V>(dyPoseTopic,
      dyPoseAdvertOpts);

  gzmsg << "Publishing dynamic pose messages on [" << opts.NameSpace() << "/"
         << dyPoseTopic << "]" << std::endl;
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::SceneInfoService(msgs::Scene &_res)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);

  _res.Clear();

  // Populate scene message
  _res.CopyFrom(convert<msgs::Scene>(this->sdfScene));

  // Add models
  AddModels(&_res, this->worldEntity, this->sceneGraph);

  // Add lights
  AddLights(&_res, this->worldEntity, this->sceneGraph);

  return true;
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::StateAsyncService(
    const msgs::StringMsg &_req)
{
  std::unique_lock<std::mutex> lock(this->stateMutex);
  this->stateServiceRequest = true;
  this->stateRequests.insert(_req.data());
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::StateService(
    msgs::SerializedStepMap &_res)
{
  _res.Clear();

  // Lock and wait for an iteration to be run and fill the state
  std::unique_lock<std::mutex> lock(this->stateMutex);

  this->stateServiceRequest = true;
  auto success = this->stateCv.wait_for(lock, 5s, [&]
  {
    return this->stepMsg.has_state() && !this->stateServiceRequest;
  });

  if (success)
    _res.CopyFrom(this->stepMsg);
  else
    gzerr << "Timed out waiting for state" << std::endl;

  return success;
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::SceneGraphService(msgs::StringMsg &_res)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);

  _res.Clear();

  std::stringstream graphStr;
  graphStr << this->sceneGraph;

  _res.set_data(graphStr.str());

  return true;
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SceneGraphAddEntities(
    const EntityComponentManager &_manager)
{
  bool newEntity{false};

  // Populate a graph with latest information from all entities

  // Scene graph for new entities. This will be used later to create a scene msg
  // to publish.
  SceneGraphType newGraph;
  auto worldVertex = this->sceneGraph.VertexFromId(this->worldEntity);
  newGraph.AddVertex(worldVertex.Name(), worldVertex.Data(), worldVertex.Id());

  // Worlds: check this in case we're loading a world without models
  _manager.EachNew<components::World>(
      [&](const Entity &, const components::World *) -> bool
      {
        newEntity = true;
        return false;
      });

  // Models
  _manager.EachNew<components::Model, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto modelMsg = std::make_shared<msgs::Model>();
        modelMsg->set_id(_entity);
        modelMsg->set_name(_nameComp->Data());
        modelMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        newGraph.AddVertex(_nameComp->Data(), modelMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Links
  _manager.EachNew<components::Link, components::Name, components::ParentEntity,
                   components::Pose>(
      [&](const Entity &_entity, const components::Link *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto linkMsg = std::make_shared<msgs::Link>();
        linkMsg->set_id(_entity);
        linkMsg->set_name(_nameComp->Data());
        linkMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        newGraph.AddVertex(_nameComp->Data(), linkMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Visuals
  _manager.EachNew<components::Visual, components::Name,
                   components::ParentEntity,
                   components::CastShadows,
                   components::Pose>(
      [&](const Entity &_entity, const components::Visual *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::CastShadows *_castShadowsComp,
          const components::Pose *_poseComp) -> bool
      {
        auto visualMsg = std::make_shared<msgs::Visual>();
        visualMsg->set_id(_entity);
        visualMsg->set_parent_id(_parentComp->Data());
        visualMsg->set_name(_nameComp->Data());
        visualMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));
        visualMsg->set_cast_shadows(_castShadowsComp->Data());

        // Geometry is optional
        auto geometryComp = _manager.Component<components::Geometry>(_entity);
        if (geometryComp)
        {
          visualMsg->mutable_geometry()->CopyFrom(
              convert<msgs::Geometry>(geometryComp->Data()));
        }

        // Material is optional
        auto materialComp = _manager.Component<components::Material>(_entity);
        if (materialComp)
        {
          visualMsg->mutable_material()->CopyFrom(
              convert<msgs::Material>(materialComp->Data()));
        }

        // Add to graph
        newGraph.AddVertex(_nameComp->Data(), visualMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Lights
  _manager.EachNew<components::Light, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Light *_lightComp,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto lightMsg = std::make_shared<msgs::Light>();
        lightMsg->CopyFrom(convert<msgs::Light>(_lightComp->Data()));
        lightMsg->set_id(_entity);
        lightMsg->set_parent_id(_parentComp->Data());
        lightMsg->set_name(_nameComp->Data());
        lightMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        newGraph.AddVertex(_nameComp->Data(), lightMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);
        newEntity = true;
        return true;
      });

  // Sensors
  _manager.EachNew<components::Sensor, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Sensor *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto sensorMsg = std::make_shared<msgs::Sensor>();
        sensorMsg->set_id(_entity);
        sensorMsg->set_parent_id(_parentComp->Data());
        sensorMsg->set_name(_nameComp->Data());
        sensorMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        auto altimeterComp = _manager.Component<components::Altimeter>(_entity);
        if (altimeterComp)
        {
          sensorMsg->set_type("altimeter");
        }
        auto airPressureComp = _manager.Component<
          components::AirPressureSensor>(_entity);
        if (airPressureComp)
        {
          sensorMsg->set_type("air_pressure");
        }
        auto airSpeedComp = _manager.Component<
          components::AirSpeedSensor>(_entity);
        if (airSpeedComp)
        {
          sensorMsg->set_type("air_speed");
        }
        auto cameraComp = _manager.Component<components::Camera>(_entity);
        if (cameraComp)
        {
          sensorMsg->set_type("camera");
          msgs::CameraSensor * cameraSensorMsg = sensorMsg->mutable_camera();
          const auto * camera = cameraComp->Data().CameraSensor();
          if (camera)
          {
            cameraSensorMsg->set_horizontal_fov(
              camera->HorizontalFov().Radian());
            cameraSensorMsg->mutable_image_size()->set_x(camera->ImageWidth());
            cameraSensorMsg->mutable_image_size()->set_y(camera->ImageHeight());
            cameraSensorMsg->set_image_format(camera->PixelFormatStr());
            cameraSensorMsg->set_near_clip(camera->NearClip());
            cameraSensorMsg->set_far_clip(camera->FarClip());
            msgs::Distortion *distortionMsg =
              cameraSensorMsg->mutable_distortion();
            if (distortionMsg)
            {
              distortionMsg->mutable_center()->set_x(
                camera->DistortionCenter().X());
              distortionMsg->mutable_center()->set_y(
                camera->DistortionCenter().Y());
              distortionMsg->set_k1(camera->DistortionK1());
              distortionMsg->set_k2(camera->DistortionK2());
              distortionMsg->set_k3(camera->DistortionK3());
              distortionMsg->set_p1(camera->DistortionP1());
              distortionMsg->set_p2(camera->DistortionP2());
            }
          }
        }
        auto contactSensorComp = _manager.Component<
          components::ContactSensor>(_entity);
        if (contactSensorComp)
        {
          sensorMsg->set_type("contact_sensor");
        }
        auto depthCameraSensorComp = _manager.Component<
          components::DepthCamera>(_entity);
        if (depthCameraSensorComp)
        {
          sensorMsg->set_type("depth_camera");
        }
        auto gpuLidarComp = _manager.Component<components::GpuLidar>(_entity);
        if (gpuLidarComp)
        {
          sensorMsg->set_type("gpu_lidar");
          msgs::LidarSensor * lidarSensorMsg = sensorMsg->mutable_lidar();
          const auto * lidar = gpuLidarComp->Data().LidarSensor();

          if (lidar && lidarSensorMsg)
          {
            lidarSensorMsg->set_horizontal_samples(
              lidar->HorizontalScanSamples());
            lidarSensorMsg->set_horizontal_resolution(
              lidar->HorizontalScanResolution());
            lidarSensorMsg->set_horizontal_min_angle(
              lidar->HorizontalScanMinAngle().Radian());
            lidarSensorMsg->set_horizontal_max_angle(
              lidar->HorizontalScanMaxAngle().Radian());
            lidarSensorMsg->set_vertical_samples(lidar->VerticalScanSamples());
            lidarSensorMsg->set_vertical_resolution(
              lidar->VerticalScanResolution());
            lidarSensorMsg->set_vertical_min_angle(
              lidar->VerticalScanMinAngle().Radian());
            lidarSensorMsg->set_vertical_max_angle(
              lidar->VerticalScanMaxAngle().Radian());
            lidarSensorMsg->set_range_min(lidar->RangeMin());
            lidarSensorMsg->set_range_max(lidar->RangeMax());
            lidarSensorMsg->set_range_resolution(lidar->RangeResolution());
            msgs::SensorNoise *sensorNoise = lidarSensorMsg->mutable_noise();
            if (sensorNoise)
            {
              const auto noise = lidar->LidarNoise();
              switch(noise.Type())
              {
                case sdf::NoiseType::GAUSSIAN:
                  sensorNoise->set_type(msgs::SensorNoise::GAUSSIAN);
                  break;
                case sdf::NoiseType::GAUSSIAN_QUANTIZED:
                  sensorNoise->set_type(msgs::SensorNoise::GAUSSIAN_QUANTIZED);
                  break;
                default:
                  sensorNoise->set_type(msgs::SensorNoise::NONE);
              }
              sensorNoise->set_mean(noise.Mean());
              sensorNoise->set_stddev(noise.StdDev());
              sensorNoise->set_bias_mean(noise.BiasMean());
              sensorNoise->set_bias_stddev(noise.BiasStdDev());
              sensorNoise->set_precision(noise.Precision());
              sensorNoise->set_dynamic_bias_stddev(noise.DynamicBiasStdDev());
              sensorNoise->set_dynamic_bias_correlation_time(
                noise.DynamicBiasCorrelationTime());
            }
          }
        }
        auto imuComp = _manager.Component<components::Imu>(_entity);
        if (imuComp)
        {
          sensorMsg->set_type("imu");
          msgs::IMUSensor * imuMsg = sensorMsg->mutable_imu();
          const auto * imu = imuComp->Data().ImuSensor();

          set(
              imuMsg->mutable_linear_acceleration()->mutable_x_noise(),
              imu->LinearAccelerationXNoise());
          set(
              imuMsg->mutable_linear_acceleration()->mutable_y_noise(),
              imu->LinearAccelerationYNoise());
          set(
              imuMsg->mutable_linear_acceleration()->mutable_z_noise(),
              imu->LinearAccelerationZNoise());
          set(
              imuMsg->mutable_angular_velocity()->mutable_x_noise(),
              imu->AngularVelocityXNoise());
          set(
              imuMsg->mutable_angular_velocity()->mutable_y_noise(),
              imu->AngularVelocityYNoise());
          set(
              imuMsg->mutable_angular_velocity()->mutable_z_noise(),
              imu->AngularVelocityZNoise());
        }
        auto laserRetroComp = _manager.Component<
          components::LaserRetro>(_entity);
        if (laserRetroComp)
        {
          sensorMsg->set_type("laser_retro");
        }
        auto lidarComp = _manager.Component<components::Lidar>(_entity);
        if (lidarComp)
        {
          sensorMsg->set_type("lidar");
        }
        auto logicalCamera = _manager.Component<
          components::LogicalCamera>(_entity);
        if (logicalCamera)
        {
          sensorMsg->set_type("logical_camera");
        }
        auto rgbdCameraComp = _manager.Component<
          components::RgbdCamera>(_entity);
        if (rgbdCameraComp)
        {
          sensorMsg->set_type("rgbd_camera");
        }
        auto thermalCameraComp = _manager.Component<
          components::ThermalCamera>(_entity);
        if (thermalCameraComp)
        {
          sensorMsg->set_type("thermal_camera");
        }

        // Add to graph
        newGraph.AddVertex(_nameComp->Data(), sensorMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);
        newEntity = true;
        return true;
      });

  // Particle emitters
  _manager.EachNew<components::ParticleEmitter, components::ParentEntity,
    components::Pose>(
      [&](const Entity &_entity,
          const components::ParticleEmitter *_emitterComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto emitterMsg = std::make_shared<msgs::ParticleEmitter>();
        emitterMsg->CopyFrom(_emitterComp->Data());
        emitterMsg->set_id(_entity);
        emitterMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        newGraph.AddVertex(emitterMsg->name(), emitterMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);
        newEntity = true;
        return true;
      });

  // Projectors
  _manager.EachNew<components::Projector, components::ParentEntity,
    components::Pose>(
      [&](const Entity &_entity,
          const components::Projector *_projectorComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto projectorMsg = std::make_shared<msgs::Projector>();
        projectorMsg->CopyFrom(
            convert<msgs::Projector>(_projectorComp->Data()));
        // \todo(anyone) add id field to projector msg
        // projectorMsg->set_id(_entity);
        projectorMsg->mutable_pose()->CopyFrom(
            msgs::Convert(_poseComp->Data()));

        // Add to graph
        newGraph.AddVertex(projectorMsg->name(), projectorMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);
        newEntity = true;
        return true;
      });

  // Update the whole scene graph from the new graph
  {
    std::lock_guard<std::mutex> lock(this->graphMutex);
    for (const auto &[id, vert] : newGraph.Vertices())
    {
      // Add the vertex only if it's not already in the graph
      if (!this->sceneGraph.VertexFromId(id).Valid())
        this->sceneGraph.AddVertex(vert.get().Name(), vert.get().Data(), id);
    }
    for (const auto &[id, edge] : newGraph.Edges())
    {
      // Add the edge only if it's not already in the graph
      if (!this->sceneGraph.EdgeFromVertices(edge.get().Vertices().first,
            edge.get().Vertices().second).Valid())
      {
        this->sceneGraph.AddEdge(edge.get().Vertices(), edge.get().Data());
      }
    }
  }

  if (newEntity)
  {
    // Only offer scene services once the message has been populated at least
    // once
    if (!this->node)
      this->SetupTransport(this->worldName);

    msgs::Scene sceneMsg;
    // Populate scene message
    sceneMsg.CopyFrom(convert<msgs::Scene>(this->sdfScene));

    AddModels(&sceneMsg, this->worldEntity, newGraph);

    // Add lights
    AddLights(&sceneMsg, this->worldEntity, newGraph);
    this->scenePub.Publish(sceneMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SceneGraphRemoveEntities(
    const EntityComponentManager &_manager)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);
  // Handle Removed Entities
  std::vector<Entity> removedEntities;

  // Scene a deleted model deletes all its child entities, we don't have to
  // handle links. We assume here that links are not deleted by themselves.
  // TODO(anyone) Handle case where other entities can be deleted without the
  // parent model being deleted.
  // Models
  _manager.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *) -> bool
      {
        removedEntities.push_back(_entity);
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph);
        return true;
      });

  // Lights
  _manager.EachRemoved<components::Light>(
      [&](const Entity &_entity, const components::Light *) -> bool
      {
        removedEntities.push_back(_entity);
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph);
        return true;
      });

  if (!removedEntities.empty())
  {
    // Send the list of deleted entities
    msgs::UInt32_V deletionMsg;

    for (const auto &entity : removedEntities)
    {
      deletionMsg.mutable_data()->Add(entity);
    }
    this->deletionPub.Publish(deletionMsg);
  }
}

//////////////////////////////////////////////////
/// \tparam T Either a msgs::Scene or msgs::Model
template<typename T>
void SceneBroadcasterPrivate::AddModels(T *_msg, const Entity _entity,
                                        const SceneGraphType &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto modelMsg = std::dynamic_pointer_cast<msgs::Model>(
        vertex.second.get().Data());
    if (!modelMsg)
      continue;

    auto msgOut = _msg->add_model();
    msgOut->CopyFrom(*modelMsg);

    // Nested models
    AddModels(msgOut, vertex.first, _graph);

    // Links
    AddLinks(msgOut, vertex.first, _graph);
  }
}

//////////////////////////////////////////////////
template<typename T>
void SceneBroadcasterPrivate::AddLights(T *_msg, const Entity _entity,
                                        const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto lightMsg = std::dynamic_pointer_cast<msgs::Light>(
        vertex.second.get().Data());
    if (!lightMsg)
      continue;

    _msg->add_light()->CopyFrom(*lightMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddVisuals(msgs::Link *_msg, const Entity _entity,
                                         const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto visualMsg = std::dynamic_pointer_cast<msgs::Visual>(
        vertex.second.get().Data());
    if (!visualMsg)
      continue;

    _msg->add_visual()->CopyFrom(*visualMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddSensors(msgs::Link *_msg, const Entity _entity,
    const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto sensorMsg = std::dynamic_pointer_cast<msgs::Sensor>(
        vertex.second.get().Data());
    if (!sensorMsg)
      continue;

    _msg->add_sensor()->CopyFrom(*sensorMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddParticleEmitters(msgs::Link *_msg,
    const Entity _entity, const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto emitterMsg = std::dynamic_pointer_cast<msgs::ParticleEmitter>(
        vertex.second.get().Data());
    if (!emitterMsg)
      continue;

    _msg->add_particle_emitter()->CopyFrom(*emitterMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddProjectors(msgs::Link *_msg,
    const Entity _entity, const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto projectorMsg = std::dynamic_pointer_cast<msgs::Projector>(
        vertex.second.get().Data());
    if (!projectorMsg)
      continue;

    _msg->add_projector()->CopyFrom(*projectorMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddLinks(msgs::Model *_msg, const Entity _entity,
                                       const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    auto linkMsg = std::dynamic_pointer_cast<msgs::Link>(
        vertex.second.get().Data());
    if (!linkMsg)
      continue;

    auto msgOut = _msg->add_link();
    msgOut->CopyFrom(*linkMsg);

    // Visuals
    AddVisuals(msgOut, vertex.second.get().Id(), _graph);

    // Lights
    AddLights(msgOut, vertex.second.get().Id(), _graph);

    // Sensors
    AddSensors(msgOut, vertex.second.get().Id(), _graph);

    // Particle emitters
    AddParticleEmitters(msgOut, vertex.second.get().Id(), _graph);

    // Projectors
    AddProjectors(msgOut, vertex.second.get().Id(), _graph);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::RemoveFromGraph(const Entity _entity,
                                              SceneGraphType &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_entity))
  {
    RemoveFromGraph(vertex.first, _graph);
  }
  _graph.RemoveVertex(_entity);
}


GZ_ADD_PLUGIN(SceneBroadcaster,
                    gz::sim::System,
                    SceneBroadcaster::ISystemConfigure,
                    SceneBroadcaster::ISystemPostUpdate,
                    SceneBroadcaster::ISystemReset)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(SceneBroadcaster,
                          "gz::sim::systems::SceneBroadcaster")
