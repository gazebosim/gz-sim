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
  /// \param[in] _id Id of the parent entity in the graph
  /// \param[in] _graph Scene graph
  public: template <typename T>
          static void AddModels(T *_msg, const EntityId _id,
                                const SceneGraphType &_graph);

  /// \brief Adds lights to a msgs::Scene or msgs::Link object based on the
  /// contents of the scene graph
  /// \tparam T Either a msgs::Scene or msgs::Link
  /// \param[in] _msg Pointer to msg object to which the lights will be added
  /// \param[in] _id Id of the parent entity in the graph
  /// \param[in] _graph Scene graph
  public: template<typename T>
          static void AddLights(T *_msg, const EntityId _id,
                                const SceneGraphType &_graph);

  /// \brief Adds links to a msgs::Model object based on the contents of
  /// the scene graph
  /// \param[in] _msg Pointer to msg object to which the links will be added
  /// \param[in] _id Id of the parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddLinks(msgs::Model *_msg, const EntityId _id,
                               const SceneGraphType &_graph);

  /// \brief Adds visuals to a msgs::Link object based on the contents of
  /// the scene graph
  /// \param[in] _msg Pointer to msg object to which the visuals will be added
  /// \param[in] _id Id of the parent entity in the graph
  /// \param[in] _graph Scene graph
  public: static void AddVisuals(msgs::Link *_msg, const EntityId _id,
                                 const SceneGraphType &_graph);

  /// \brief Recursively remove entities from the graph
  /// \param[in] _id Id of entity
  /// \param[in/out] _graph Scene graph
  public: static void RemoveFromGraph(const EntityId _id,
                                      SceneGraphType &_graph);

  /// \brief Transport node.
  public: std::unique_ptr<transport::Node> node{nullptr};

  /// \brief Pose publisher.
  public: transport::Node::Publisher posePub;

  /// \brief Scene publisher
  public: transport::Node::Publisher scenePub;

  /// \brief Request publisher.
  /// This is used to request entities to be removed
  public: transport::Node::Publisher deletionPub;

  /// \brief Graph containing latest information from entities.
  /// The data in each node is the message associated with that entity only.
  /// i.e, a model node only has a message about the model. It will not
  /// have any links, joints, etc. To create a the whole scene, one has to
  /// traverse the graph adding messages as necessary.
  public: SceneGraphType sceneGraph;

  /// \brief Keep the id of the world entity so we know how to traverse the
  /// graph.
  public: EntityId worldId{kNullEntity};

  /// \brief Keep the name of the world entity so it's easy to create temporary
  /// scene graphs
  public: std::string worldName;

  /// \brief Protects scene graph.
  public: std::mutex graphMutex;
};

//////////////////////////////////////////////////
SceneBroadcaster::SceneBroadcaster()
  : System(), dataPtr(std::make_unique<SceneBroadcasterPrivate>())
{
}

//////////////////////////////////////////////////
void SceneBroadcaster::Configure(
    const EntityId &_id, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm, EventManager &)
{
  // World
  auto name = _ecm.Component<components::Name>(_id);
  if (name == nullptr)
  {
    ignerr << "World with id: " << _id
           << " has no name. SceneBroadcaster cannot create transport topics\n";
    return;
  }

  this->dataPtr->worldId = _id;
  this->dataPtr->worldName = name->Data();

  this->dataPtr->SetupTransport(this->dataPtr->worldName);

  // Add to graph
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->graphMutex);
    this->dataPtr->sceneGraph.AddVertex(this->dataPtr->worldName, nullptr,
                                        this->dataPtr->worldId);
  }
}

//////////////////////////////////////////////////
void SceneBroadcaster::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &_manager)
{
  // Update scene graph with added entities before populating pose message
  this->dataPtr->SceneGraphAddEntities(_manager);

  // Populate pose message
  // TODO(louise) Get <scene> from SDF
  // TODO(louise) Fill message header

  msgs::Pose_V poseMsg;

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

  // call SceneGraphRemoveEntities at the end of this update cycle so that
  // erased entities are removed from the scene graph for the next update cycle
  this->dataPtr->SceneGraphRemoveEntities(_manager);
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

  // Scene info topic
  std::string sceneTopic{"/world/" + _worldName + "/scene/info"};

  this->scenePub = this->node->Advertise<ignition::msgs::Scene>(sceneTopic);

  ignmsg << "Serving scene information on [" << sceneTopic << "]" << std::endl;

  // Entity deletion publisher
  std::string deletionTopic{"/world/" + _worldName + "/scene/deletion"};

  this->deletionPub =
      this->node->Advertise<ignition::msgs::UInt32_V>(deletionTopic);

  ignmsg << "Publishing entity deletions on [" << deletionTopic << "]"
         << std::endl;

  // Pose info publisher
  std::string topic{"pose/info"};

  transport::AdvertiseMessageOptions advertOpts;
  advertOpts.SetMsgsPerSec(60);
  this->posePub = this->node->Advertise<msgs::Pose_V>(topic, advertOpts);

  ignmsg << "Publishing pose messages on [" << opts.NameSpace() << "/" << topic
         << "]" << std::endl;
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

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SceneGraphAddEntities(
    const EntityComponentManager &_manager)
{
  bool newEntity{false};

  // Populate a graph with latest information from all entities

  // Scene graph for new entities. This will be used later to create a scene msg
  // to publish.
  SceneGraphType newGraph;
  auto worldVertex = this->sceneGraph.VertexFromId(this->worldId);
  newGraph.AddVertex(worldVertex.Name(), worldVertex.Data(), worldVertex.Id());

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
        newGraph.AddVertex(_nameComp->Data(), modelMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

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
        newGraph.AddVertex(_nameComp->Data(), linkMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

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
        newGraph.AddVertex(_nameComp->Data(), visualMsg, _entity);
        newGraph.AddEdge({_parentComp->Data(), _entity}, true);

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
      if (!this->sceneGraph.VertexFromId(id).Valid())
        this->sceneGraph.AddVertex(vert.get().Name(), vert.get().Data(), id);
    }
    for (const auto &[id, edge] : newGraph.Edges())
    {
      if (!this->sceneGraph.EdgeFromId(edge.get().Id()).Valid())
        this->sceneGraph.AddEdge(edge.get().Vertices(), edge.get().Data());
    }
  }

  if (newEntity)
  {
    msgs::Scene sceneMsg;

    AddModels(&sceneMsg, this->worldId, newGraph);

    // Add lights
    AddLights(&sceneMsg, this->worldId, newGraph);
    this->scenePub.Publish(sceneMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::SceneGraphRemoveEntities(
    const EntityComponentManager &_manager)
{
  std::lock_guard<std::mutex> lock(this->graphMutex);
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
        erasedEntities.push_back(_entity);
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph);
        return true;
      });

  // Lights
  _manager.EachErased<components::Light>(
      [&](const EntityId &_entity, const components::Light *) -> bool
      {
        erasedEntities.push_back(_entity);
        // Remove from graph
        RemoveFromGraph(_entity, this->sceneGraph);
        return true;
      });

  if (!erasedEntities.empty())
  {
    // Send the list of deleted entities
    msgs::UInt32_V deletionMsg;

    for (const auto &entity : erasedEntities)
    {
      deletionMsg.mutable_data()->Add(entity);
    }
    this->deletionPub.Publish(deletionMsg);
  }
}

//////////////////////////////////////////////////
/// \tparam T Either a msgs::Scene or msgs::Model
template<typename T>
void SceneBroadcasterPrivate::AddModels(T *_msg, const EntityId _id,
                                        const SceneGraphType &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_id))
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
void SceneBroadcasterPrivate::AddLights(T *_msg, const EntityId _id,
                                        const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto lightMsg = std::dynamic_pointer_cast<msgs::Light>(
        vertex.second.get().Data());
    if (!lightMsg)
      continue;

    _msg->add_light()->CopyFrom(*lightMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddVisuals(msgs::Link *_msg, const EntityId _id,
                                         const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto visualMsg = std::dynamic_pointer_cast<msgs::Visual>(
        vertex.second.get().Data());
    if (!visualMsg)
      continue;

    _msg->add_visual()->CopyFrom(*visualMsg);
  }
}

//////////////////////////////////////////////////
void SceneBroadcasterPrivate::AddLinks(msgs::Model *_msg, const EntityId _id,
                                       const SceneGraphType &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
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
void SceneBroadcasterPrivate::RemoveFromGraph(const EntityId _id,
                                              SceneGraphType &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    RemoveFromGraph(vertex.first, _graph);
  }
  _graph.RemoveVertex(_id);
}


IGNITION_ADD_PLUGIN(ignition::gazebo::systems::SceneBroadcaster,
                    ignition::gazebo::System,
                    SceneBroadcaster::ISystemConfigure,
                    SceneBroadcaster::ISystemPostUpdate)
