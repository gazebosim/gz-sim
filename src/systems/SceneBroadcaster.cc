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

//////////////////////////////////////////////////
template<typename T>
void AddLights(T _msg,
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
void AddVisuals(msgs::LinkSharedPtr _msg,
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
void AddLinks(msgs::ModelSharedPtr _msg,
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
    AddVisuals(linkMsg, vertex.second.get().Id(), _graph);

    // Lights
    AddLights(linkMsg, vertex.second.get().Id(), _graph);

    _msg->add_link()->CopyFrom(*linkMsg.get());
  }
}

//////////////////////////////////////////////////
template<typename T>
void AddModels(T _msg,
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

    // Nested models
    AddModels(modelMsg, vertex.second.get().Id(), _graph);

    // Links
    AddLinks(modelMsg, vertex.second.get().Id(), _graph);

    _msg->add_model()->CopyFrom(*modelMsg.get());
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

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Pose publisher.
  public: transport::Node::Publisher posePub;

  /// \brief Graph containing latest information from entities.
  public: math::graph::DirectedGraph<
      std::shared_ptr<google::protobuf::Message>, bool> sceneGraph;

  /// \brief Keep the id of the world entity so we know how to traverse the
  /// graph.
  public: EntityId worldId;

  /// \brief Protects scene graph.
  public: std::mutex graphMutex;
};

//////////////////////////////////////////////////
SceneBroadcaster::SceneBroadcaster()
  : System(), dataPtr(std::make_unique<SceneBroadcasterPrivate>())
{
}

//////////////////////////////////////////////////
SceneBroadcaster::~SceneBroadcaster()
{
}

//////////////////////////////////////////////////
void SceneBroadcaster::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &_manager)
{
  // Populate pose message
  msgs::Pose_V poseMsg;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->graphMutex);

    // TODO(louise) Get <scene> from SDF
    // TODO(louise) Fill message header

    // Populate a graph with latest information from all entities
    // TODO(louise) once we know what entities are added/deleted process only
    // those. For now, recreating graph at every iteration.
    this->dataPtr->sceneGraph = math::graph::DirectedGraph<
        std::shared_ptr<google::protobuf::Message>, bool>();

    // World
    // \todo(anyone) It would be convenient to have the following functions:
    // * _manager.Has<components::World, components::Name>: to tell whether
    // there
    //   is an entity which has a given set of components.
    // * _manager.EntityCount<components::World, components::Name>: returns the
    //  number of entities which have all the given components.
    this->dataPtr->worldId = kNullEntity;
    _manager.Each<components::World,
                  components::Name>(
      [&](const EntityId &_entity,
          const components::World */*_worldComp*/,
          const components::Name *_nameComp)->bool
      {
        if (kNullEntity != this->dataPtr->worldId)
        {
          ignerr << "Internal error, more than one world found." << std::endl;
          return true;
        }
        this->dataPtr->worldId = _entity;

        if (!this->dataPtr->posePub)
        {
          this->dataPtr->SetupTransport(_nameComp->Data());
        }

        // Add to graph
        this->dataPtr->sceneGraph.AddVertex(
            _nameComp->Data(), nullptr, _entity);
        return true;
      });

    if (kNullEntity == this->dataPtr->worldId)
    {
      ignerr << "Failed to find world entity" << std::endl;
      return;
    }

    // Models
    _manager.Each<components::Model,
                  components::Name,
                  components::ParentEntity,
                  components::Pose>(
      [&](const EntityId &_entity,
          const components::Model */*_modelComp*/,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp)->bool
      {
        auto modelMsg = std::make_shared<msgs::Model>();
        modelMsg->set_id(_entity);
        modelMsg->set_name(_nameComp->Data());
        modelMsg->mutable_pose()->CopyFrom(msgs::Convert(
            _poseComp->Data()));

        // Add to graph
        this->dataPtr->sceneGraph.AddVertex(
            _nameComp->Data(), modelMsg, _entity);
        this->dataPtr->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        // Add to pose msg
        auto pose = poseMsg.add_pose();
        msgs::Set(pose, _poseComp->Data());
        pose->set_name(_nameComp->Data());
        pose->set_id(_entity);
        return true;
      });

    // Links
    _manager.Each<components::Link,
                  components::Name,
                  components::ParentEntity,
                  components::Pose>(
      [&](const EntityId &_entity,
          const components::Link */*_linkComp*/,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp)->bool
      {
        auto linkMsg = std::make_shared<msgs::Link>();
        linkMsg->set_id(_entity);
        linkMsg->set_name(_nameComp->Data());
        linkMsg->mutable_pose()->CopyFrom(msgs::Convert(
            _poseComp->Data()));

        // Add to graph
        this->dataPtr->sceneGraph.AddVertex(
            _nameComp->Data(), linkMsg, _entity);
        this->dataPtr->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        // Add to pose msg
        auto pose = poseMsg.add_pose();
        msgs::Set(pose, _poseComp->Data());
        pose->set_name(_nameComp->Data());
        pose->set_id(_entity);
        return true;
      });

    // Visuals
    _manager.Each<components::Visual,
                  components::Name,
                  components::ParentEntity,
                  components::Pose>(
      [&](const EntityId &_entity,
          const components::Visual */*_visualComp*/,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp)->bool
      {
        auto visualMsg = std::make_shared<msgs::Visual>();
        visualMsg->set_id(_entity);
        visualMsg->set_parent_id(_parentComp->Data());
        visualMsg->set_name(_nameComp->Data());
        visualMsg->mutable_pose()->CopyFrom(msgs::Convert(
            _poseComp->Data()));

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
        this->dataPtr->sceneGraph.AddVertex(
            _nameComp->Data(), visualMsg, _entity);
        this->dataPtr->sceneGraph.AddEdge(
            {_parentComp->Data(), _entity}, true);

        // Add to pose msg
        auto pose = poseMsg.add_pose();
        msgs::Set(pose, _poseComp->Data());
        pose->set_name(_nameComp->Data());
        pose->set_id(_entity);
        return true;
      });

    // Lights
    _manager.Each<components::Light,
                  components::Name,
                  components::ParentEntity,
                  components::Pose>(
      [&](const EntityId &_entity,
          const components::Light *_lightComp,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp)->bool
      {
        auto lightMsg = std::make_shared<msgs::Light>();
        lightMsg->CopyFrom(Convert<msgs::Light>(_lightComp->Data()));
        lightMsg->set_id(_entity);
        lightMsg->set_parent_id(_parentComp->Data());
        lightMsg->set_name(_nameComp->Data());
        lightMsg->mutable_pose()->CopyFrom(msgs::Convert(
            _poseComp->Data()));

        // Add to graph
        this->dataPtr->sceneGraph.AddVertex(
            _nameComp->Data(), lightMsg, _entity);
        this->dataPtr->sceneGraph.AddEdge({_parentComp->Data(), _entity}, true);

        // Add to pose msg
        auto pose = poseMsg.add_pose();
        msgs::Set(pose, _poseComp->Data());
        pose->set_name(_nameComp->Data());
        pose->set_id(_entity);
        return true;
      });
  }
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

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::SceneBroadcaster,
                    ignition::gazebo::System,
                    SceneBroadcaster::ISystemPostUpdate)
