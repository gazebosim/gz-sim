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

#include <ignition/msgs/scene.pb.h>
#include <ignition/math/graph/Graph.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/systems/SceneBroadcaster.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
template<typename T>
void AddLights(T *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<
        std::shared_ptr<google::protobuf::Message>, bool> &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto lightMsg = std::dynamic_pointer_cast<msgs::Light>(
        vertex.second.get().Data());
    if (!lightMsg)
      continue;

    _msg->add_light()->CopyFrom(*lightMsg.get());
  }
}

//////////////////////////////////////////////////
void AddVisuals(msgs::Link *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<
        std::shared_ptr<google::protobuf::Message>, bool> &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto visualMsg = std::dynamic_pointer_cast<msgs::Visual>(
        vertex.second.get().Data());
    if (!visualMsg)
      continue;

    _msg->add_visual()->CopyFrom(*visualMsg.get());
  }
}

//////////////////////////////////////////////////
void AddLinks(msgs::Model *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<
        std::shared_ptr<google::protobuf::Message>, bool> &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto linkMsg = std::dynamic_pointer_cast<msgs::Link>(
        vertex.second.get().Data());
    if (!linkMsg)
      continue;

    // Visuals
    AddVisuals(linkMsg.get(), vertex.second.get().Id(), _graph);

    // Lights
    AddLights(linkMsg.get(), vertex.second.get().Id(), _graph);

    _msg->add_link()->CopyFrom(*linkMsg.get());
  }
}

//////////////////////////////////////////////////
/// \tparam T Either a msgs::Scene or msgs::Model
template<typename T>
void AddModels(T *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<
        std::shared_ptr<google::protobuf::Message>, bool> &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto modelMsg = std::dynamic_pointer_cast<msgs::Model>(
        vertex.second.get().Data());
    if (!modelMsg)
      continue;

    std::cout << "Adding model: " << modelMsg->name() << std::endl;
    // Nested models
    AddModels(modelMsg.get(), vertex.second.get().Id(), _graph);

    // Links
    AddLinks(modelMsg.get(), vertex.second.get().Id(), _graph);

    _msg->add_model()->CopyFrom(*modelMsg.get());
  }
}

//////////////////////////////////////////////////
/// \brief Recursively remove entities from the graph
/// \param[in] _id Id of entity
/// \param[in] _graph Scene graph
/// \param[out] _erasedEntities All erased entities
void RemoveFromGraph(
    const EntityId _id,
    const math::graph::DirectedGraph<std::shared_ptr<google::protobuf::Message>,
                                     bool> &_graph,
    std::vector<EntityId> &_erasedEntities)
{
  _erasedEntities.push_back(_id);
  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    RemoveFromGraph(vertex.first, _graph, _erasedEntities);
  }
}

// Private data class.
class ignition::gazebo::systems::SceneBroadcasterPrivate
{
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

  /// \brief Updates the scene graph when entities are added/removed
  /// \param[in] _manager The entity component manager
  public: void UpdateSceneGraph(const EntityComponentManager &_manager);

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Pose publisher.
  public: transport::Node::Publisher posePub;

  /// \brief Scene publisher
  public: transport::Node::Publisher scenePub;

  /// \brief Request publisher.
  /// This is used to request entities to be removed
  public: transport::Node::Publisher requestPub;

  /// \brief Graph containing latest information from entities.
  public: math::graph::DirectedGraph<
      std::shared_ptr<google::protobuf::Message>, bool> sceneGraph;

  /// \brief Keep the id of the world entity so we know how to traverse the
  /// graph.
  public: EntityId worldId{kNullEntity};

  /// \brief Protects scene graph.
  public: std::mutex graphMutex;
};

//////////////////////////////////////////////////
SceneBroadcaster::SceneBroadcaster()
  : System(), dataPtr(std::make_unique<SceneBroadcasterPrivate>())
{
  this->dataPtr->sceneGraph = math::graph::DirectedGraph<
      std::shared_ptr<google::protobuf::Message>, bool>();
}

//////////////////////////////////////////////////
SceneBroadcaster::~SceneBroadcaster()
{
}

//////////////////////////////////////////////////
void SceneBroadcaster::Configure(
    const EntityId &_id, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm, EventManager &)
{
  // World
  this->dataPtr->worldId = _id;
  auto name = _ecm.Component<components::Name>(_id);
  this->dataPtr->SetupTransport(name->Data());

  // Add to graph
  std::lock_guard<std::mutex> lock(this->dataPtr->graphMutex);
  this->dataPtr->sceneGraph.AddVertex(name->Data(), nullptr, _id);
}

//////////////////////////////////////////////////
void SceneBroadcaster::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &_manager)
{
  this->dataPtr->UpdateSceneGraph(_manager);

  // Populate pose message
  // TODO(louise) Get <scene> from SDF
  // TODO(louise) Fill message header

  // Populate a graph with latest information from all entities
  // TODO(louise) once we know what entities are added/deleted process only
  // those. For now, recreating graph at every iteration.

  msgs::Pose_V poseMsg;
  // Levels
  _manager.Each<components::Level, components::Name, components::Pose>(
      [&](const EntityId &_entity, const components::Level *,
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

  // Models
  _manager.Each<components::Model, components::Name, components::Pose>(
      [&](const EntityId &_entity, const components::Model *,
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

  // Links
  _manager.Each<components::Link, components::Name, components::Pose>(
      [&](const EntityId &_entity, const components::Link *,
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

  // Visuals
  _manager.Each<components::Visual, components::Name, components::Pose>(
      [&](const EntityId &_entity, const components::Visual *,
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
      [&](const EntityId &_entity, const components::Light *,
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

  this->dataPtr->posePub.Publish(poseMsg);
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SetupTransport(const std::string &_worldName)
{
  // Scene info service
  std::string infoService{"/world/" + _worldName + "/scene/info"};

  this->node.Advertise(infoService, &SceneBroadcasterPrivate::SceneInfoService,
      this);

  ignmsg << "Serving scene information on [" << infoService << "]" << std::endl;

  // Scene info service
  std::string sceneTopic{"/world/" + _worldName + "/scene"};

  this->scenePub = this->node.Advertise<ignition::msgs::Scene>(sceneTopic);

  ignmsg << "Serving scene information on [" << infoService << "]" << std::endl;
  // Scene Requests
  std::string requestTopic{"/world/" + _worldName + "/scene/request"};

  this->requestPub =
      this->node.Advertise<ignition::msgs::Request>(requestTopic);

  ignmsg << "Publishing scene managements requests on [" << requestTopic << "]"
         << std::endl;
  // Scene graph service
  std::string graphService{"/world/" + _worldName + "/scene/graph"};

  this->node.Advertise(graphService,
      &SceneBroadcasterPrivate::SceneGraphService, this);

  ignmsg << "Serving scene graph on [" << graphService << "]" << std::endl;

  // Pose info publisher
  std::string topic{"/world/" + _worldName + "/pose/info"};

  transport::AdvertiseMessageOptions advertOpts;
  advertOpts.SetMsgsPerSec(60);
  this->posePub = this->node.Advertise<msgs::Pose_V>(topic, advertOpts);

  ignmsg << "Publishing pose messages on [" << topic << "]" << std::endl;

}

//////////////////////////////////////////////////
bool SceneBroadcasterPrivate::SceneInfoService(ignition::msgs::Scene &_res)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);

  _res.Clear();

  // Populate scene message

  // Add models
  AddModels(&_res, this->worldId, this->sceneGraph);

  // Add lights
  AddLights(&_res, this->worldId, this->sceneGraph);

  return true;
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

void SceneBroadcasterPrivate::UpdateSceneGraph(
    const EntityComponentManager &_manager)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);

  // TODO(louise) Get <scene> from SDF
  // TODO(louise) Fill message header

  // Populate a graph with latest information from all entities
  // TODO(louise) once we know what entities are added/deleted process only
  // those. For now, recreating graph at every iteration.

  bool newEntity{false};

  // Levels
  _manager.EachNew<components::Level, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const EntityId &_entity, const components::Level *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        // use a model for level visualization. May need to change later
        auto levelMsg = std::make_shared<msgs::Model>();
        levelMsg->set_id(_entity);
        levelMsg->set_name(_nameComp->Data());
        levelMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        this->sceneGraph.AddVertex(_nameComp->Data(), levelMsg, _entity);
        this->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Models
  _manager.EachNew<components::Model, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const EntityId &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto modelMsg = std::make_shared<msgs::Model>();
        modelMsg->set_id(_entity);
        modelMsg->set_name(_nameComp->Data());
        modelMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        this->sceneGraph.AddVertex(_nameComp->Data(), modelMsg, _entity);
        this->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Links
  _manager.EachNew<components::Link, components::Name, components::ParentEntity,
                   components::Pose>(
      [&](const EntityId &_entity, const components::Link *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto linkMsg = std::make_shared<msgs::Link>();
        linkMsg->set_id(_entity);
        linkMsg->set_name(_nameComp->Data());
        linkMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        this->sceneGraph.AddVertex(_nameComp->Data(), linkMsg, _entity);
        this->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Visuals
  _manager.EachNew<components::Visual, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const EntityId &_entity, const components::Visual *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto visualMsg = std::make_shared<msgs::Visual>();
        visualMsg->set_id(_entity);
        visualMsg->set_parent_id(_parentComp->Data());
        visualMsg->set_name(_nameComp->Data());
        visualMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Geometry is optional
        auto geometryComp = _manager.Component<components::Geometry>(_entity);
        if (geometryComp)
        {
          visualMsg->mutable_geometry()->CopyFrom(
              Convert<msgs::Geometry>(geometryComp->Data()));
        }

        // Material is optional
        auto materialComp = _manager.Component<components::Material>(_entity);
        if (materialComp)
        {
          visualMsg->mutable_material()->CopyFrom(
              Convert<msgs::Material>(materialComp->Data()));
        }

        // Add to graph
        this->sceneGraph.AddVertex(_nameComp->Data(), visualMsg, _entity);
        this->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        newEntity = true;
        return true;
      });

  // Lights
  _manager.EachNew<components::Light, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const EntityId &_entity, const components::Light *_lightComp,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        auto lightMsg = std::make_shared<msgs::Light>();
        lightMsg->CopyFrom(Convert<msgs::Light>(_lightComp->Data()));
        lightMsg->set_id(_entity);
        lightMsg->set_parent_id(_parentComp->Data());
        lightMsg->set_name(_nameComp->Data());
        lightMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));

        // Add to graph
        this->sceneGraph.AddVertex(_nameComp->Data(), lightMsg, _entity);
        this->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);
        newEntity = true;
        return true;
      });



  // Handle Erased Entities
  std::vector<EntityId> erasedEntities;

  // Scene a deleted model deletes all its child entities, we don't have to
  // handle links. We assume here that links are not deleted by themselves.
  // TODO(anyone) Handle case where other entities can be deleted without the
  // parent model being deleted.
  // Models
  _manager.EachErased<components::Model>(
      [&](const EntityId &_entity, const components::Model *) -> bool
      {
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph, erasedEntities);
        return true;
      });

  // Lights
  _manager.EachErased<components::Light>(
      [&](const EntityId &_entity, const components::Light *) -> bool
      {
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph, erasedEntities);
        return true;
      });

  if (newEntity)
  {
    // TODO(addisu) only add the new entities
    msgs::Scene sceneMsg;

    AddModels(&sceneMsg, this->worldId, this->sceneGraph);

    // Add lights
    AddLights(&sceneMsg, this->worldId, this->sceneGraph);
    this->scenePub.Publish(sceneMsg);
  }
  // For each erased entity, we send a delete message
  for (const auto &entity : erasedEntities)
  {
    std::cout << "SceneBroadcaster: Delete " << entity << std::endl;
    msgs::Request req;
    req.set_request("delete_entity");
    req.set_id(entity);
    this->requestPub.Publish(req);
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::SceneBroadcaster,
                    ignition::gazebo::System,
                    SceneBroadcaster::ISystemConfigure,
                    SceneBroadcaster::ISystemPostUpdate)
