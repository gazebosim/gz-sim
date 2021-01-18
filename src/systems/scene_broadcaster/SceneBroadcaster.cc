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

#include <ignition/msgs/scene.pb.h>

#include <chrono>
#include <condition_variable>
#include <string>
#include <unordered_set>

#include <ignition/common/Profiler.hh>
#include <ignition/math/graph/Graph.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace std::chrono_literals;

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::SceneBroadcasterPrivate
{
  /// \brief Type alias for the graph used to represent the scene graph.
  public: using SceneGraphType = math::graph::DirectedGraph<
          std::shared_ptr<google::protobuf::Message>, bool>;

  /// \brief Setup Ignition transport services and publishers
  /// \param[in] _worldName Name of world.
  public: void SetupTransport(const std::string &_worldName);

  /// \brief Callback for scene info service.
  /// \param[out] _res Response containing the latest scene message.
  /// \return True if successful.
  public: bool SceneInfoService(ignition::msgs::Scene &_res);

  /// \brief Callback for scene graph service.
  /// \param[out] _res Response containing the the scene graph in DOT format.
  /// \return True if successful.
  public: bool SceneGraphService(ignition::msgs::StringMsg &_res);

  /// \brief Callback for state service.
  /// \param[out] _res Response containing the latest full state.
  /// \return True if successful.
  public: bool StateService(ignition::msgs::SerializedStepMap &_res);

  /// \brief Callback for state service - non blocking.
  /// \param[out] _res Response containing the last available full state.
  public: void StateAsyncService(const ignition::msgs::StringMsg &_req);

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

  /// \brief Period to publish state, defaults to 60 Hz.
  public: std::chrono::duration<int64_t, std::ratio<1, 1000>>
      statePublishPeriod{std::chrono::milliseconds(1000/60)};

  /// \brief Flag used to indicate if the state service was called.
  public: bool stateServiceRequest{false};

  /// \brief A list of async state requests
  public: std::unordered_set<std::string> stateRequests;
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
    ignerr << "World with id: " << _entity
           << " has no name. SceneBroadcaster cannot create transport topics\n";
    return;
  }

  this->dataPtr->worldEntity = _entity;
  this->dataPtr->worldName = name->Data();

  auto readHertz = _sdf->Get<int>("dynamic_pose_hertz", 60);
  this->dataPtr->dyPoseHertz = readHertz.first;

  auto stateHerz = _sdf->Get<int>("state_hertz", 60);
  this->dataPtr->statePublishPeriod =
      std::chrono::duration<int64_t, std::ratio<1, 1000>>(
      std::chrono::milliseconds(1000/stateHerz.first));

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
  IGN_PROFILE("SceneBroadcaster::PostUpdate");

  // Update scene graph with added entities before populating pose message
  if (_manager.HasNewEntities())
    this->dataPtr->SceneGraphAddEntities(_manager);

  // Populate pose message
  // TODO(louise) Get <scene> from SDF

  // Create and send pose update if transport connections exist.
  if (this->dataPtr->dyPosePub.HasConnections() ||
      this->dataPtr->posePub.HasConnections())
  {
    this->dataPtr->PoseUpdate(_info, _manager);
  }

  // call SceneGraphRemoveEntities at the end of this update cycle so that
  // removed entities are removed from the scene graph for the next update cycle
  this->dataPtr->SceneGraphRemoveEntities(_manager);

  // Publish state only if there are subscribers and
  // * throttle rate to 60 Hz
  // * also publish off-rate if there are change events:
  //     * new / erased entities
  //     * components with one-time changes
  //     * jump back in time
  // Throttle here instead of using transport::AdvertiseMessageOptions so that
  // we can skip the ECM serialization
  bool jumpBackInTime = _info.dt < std::chrono::steady_clock::duration::zero();
  auto now = std::chrono::system_clock::now();
  bool changeEvent = _manager.HasEntitiesMarkedForRemoval() ||
        _manager.HasNewEntities() || _manager.HasOneTimeComponentChanges() ||
        jumpBackInTime;
  bool itsPubTime = now - this->dataPtr->lastStatePubTime >
       this->dataPtr->statePublishPeriod;
  auto shouldPublish = this->dataPtr->statePub.HasConnections() &&
       (changeEvent || itsPubTime);

  if (this->dataPtr->stateServiceRequest || shouldPublish)
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->stateMutex);
    this->dataPtr->stepMsg.Clear();

    set(this->dataPtr->stepMsg.mutable_stats(), _info);

    // Publish full state if there are change events
    if (changeEvent || this->dataPtr->stateServiceRequest)
    {
      _manager.State(*this->dataPtr->stepMsg.mutable_state(), {}, {}, true);
    }
    // Otherwise publish just periodic change components
    else
    {
      IGN_PROFILE("SceneBroadcast::PostUpdate UpdateState");
      auto periodicComponents = _manager.ComponentTypesWithPeriodicChanges();
      _manager.State(*this->dataPtr->stepMsg.mutable_state(),
          {}, periodicComponents);
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
      IGN_PROFILE("SceneBroadcast::PostUpdate Publish State");
      this->dataPtr->statePub.Publish(this->dataPtr->stepMsg);
      this->dataPtr->lastStatePubTime = now;
    }
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::PoseUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_manager)
{
  IGN_PROFILE("SceneBroadcast::PoseUpdate");

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
  transport::NodeOptions opts;
  opts.SetNameSpace("/world/" + _worldName);
  this->node = std::make_unique<transport::Node>(opts);

  // Scene info service
  std::string infoService{"scene/info"};

  this->node->Advertise(infoService, &SceneBroadcasterPrivate::SceneInfoService,
      this);

  ignmsg << "Serving scene information on [" << opts.NameSpace() << "/"
         << infoService << "]" << std::endl;

  // Scene graph service
  std::string graphService{"scene/graph"};

  this->node->Advertise(graphService,
      &SceneBroadcasterPrivate::SceneGraphService, this);

  ignmsg << "Serving graph information on [" << opts.NameSpace() << "/"
         << graphService << "]" << std::endl;

  // State service
  // Note: GuiRunner used to call this service but it is now using the async
  // version (state_async)
  std::string stateService{"state"};

  this->node->Advertise(stateService, &SceneBroadcasterPrivate::StateService,
      this);

  ignmsg << "Serving full state on [" << opts.NameSpace() << "/"
         << stateService << "]" << std::endl;

  // Async State service
  std::string stateAsyncService{"state_async"};

  this->node->Advertise(stateAsyncService,
      &SceneBroadcasterPrivate::StateAsyncService, this);

  ignmsg << "Serving full state (async) on [" << opts.NameSpace() << "/"
         << stateAsyncService << "]" << std::endl;

  // Scene info topic
  std::string sceneTopic{"/world/" + _worldName + "/scene/info"};

  this->scenePub = this->node->Advertise<ignition::msgs::Scene>(sceneTopic);

  ignmsg << "Publishing scene information on [" << sceneTopic
         << "]" << std::endl;

  // Entity deletion publisher
  std::string deletionTopic{"/world/" + _worldName + "/scene/deletion"};

  this->deletionPub =
      this->node->Advertise<ignition::msgs::UInt32_V>(deletionTopic);

  ignmsg << "Publishing entity deletions on [" << deletionTopic << "]"
         << std::endl;

  // State topic
  std::string stateTopic{"/world/" + _worldName + "/state"};

  this->statePub =
      this->node->Advertise<ignition::msgs::SerializedStepMap>(stateTopic);

  ignmsg << "Publishing state changes on [" << stateTopic << "]"
      << std::endl;

  // Pose info publisher
  std::string poseTopic{"pose/info"};

  transport::AdvertiseMessageOptions poseAdvertOpts;
  poseAdvertOpts.SetMsgsPerSec(60);
  this->posePub = this->node->Advertise<msgs::Pose_V>(poseTopic,
      poseAdvertOpts);

  ignmsg << "Publishing pose messages on [" << opts.NameSpace() << "/"
         << poseTopic << "]" << std::endl;

  // Dynamic pose info publisher
  std::string dyPoseTopic{"dynamic_pose/info"};

  transport::AdvertiseMessageOptions dyPoseAdvertOpts;
  dyPoseAdvertOpts.SetMsgsPerSec(this->dyPoseHertz);
  this->dyPosePub = this->node->Advertise<msgs::Pose_V>(dyPoseTopic,
      dyPoseAdvertOpts);

  ignmsg << "Publishing dynamic pose messages on [" << opts.NameSpace() << "/"
         << dyPoseTopic << "]" << std::endl;
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::SceneInfoService(ignition::msgs::Scene &_res)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);

  _res.Clear();

  // Populate scene message

  // Add models
  AddModels(&_res, this->worldEntity, this->sceneGraph);

  // Add lights
  AddLights(&_res, this->worldEntity, this->sceneGraph);

  return true;
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::StateAsyncService(
    const ignition::msgs::StringMsg &_req)
{
  std::unique_lock<std::mutex> lock(this->stateMutex);
  this->stateServiceRequest = true;
  this->stateRequests.insert(_req.data());
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::StateService(
    ignition::msgs::SerializedStepMap &_res)
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
    ignerr << "Timed out waiting for state" << std::endl;

  return success;
}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::SceneGraphService(ignition::msgs::StringMsg &_res)
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


IGNITION_ADD_PLUGIN(SceneBroadcaster,
                    ignition::gazebo::System,
                    SceneBroadcaster::ISystemConfigure,
                    SceneBroadcaster::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(SceneBroadcaster,
                          "ignition::gazebo::systems::SceneBroadcaster")
