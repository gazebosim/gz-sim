/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include "Scene3DNew.hh"
#include <ignition/common/Console.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/ParentEntity.hh"

#include <iostream>

namespace ignition
{
namespace gui
{
namespace plugins
{
  class Scene3DNewPrivate
  {
    public: bool initialized = false;

    /// \brief New models to be created. The elements in the tuple are:
    /// [0] entity id, [1], SDF DOM, [2] parent entity id, [3] sim iteration
    public: std::vector<
      std::tuple<ignition::gazebo::Entity, sdf::Model, ignition::gazebo::Entity,
        uint64_t>> newModels;

    /// \brief A map of entity ids and pose updates.
    public: std::unordered_map<ignition::gazebo::Entity, math::Pose3d> entityPoses;

    /// \brief Keep the id of the world entity so we know how to traverse the
    /// graph.
    public: ignition::gazebo::Entity worldEntity{ignition::gazebo::kNullEntity};

    /// \brief Type alias for the graph used to represent the scene graph.
    public: using SceneGraphType = math::graph::DirectedGraph<
            std::shared_ptr<google::protobuf::Message>, bool>;
    /// \brief Graph containing latest information from entities.
    /// The data in each node is the message associated with that entity only.
    /// i.e, a model node only has a message about the model. It will not
    /// have any links, joints, etc. To create a the whole scene, one has to
    /// traverse the graph adding messages as necessary.
    public: SceneGraphType sceneGraph;

    /// \brief Keep the name of the world entity so it's easy to create temporary
    /// scene graphs
    public: std::string worldName;

    //////////////////////////////////////////////////
    /// \tparam T Either a msgs::Scene or msgs::Model
    template<typename T>
    void AddModels(T *_msg, const ignition::gazebo::Entity _entity,
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
    void AddLights(T *_msg, const ignition::gazebo::Entity _entity,
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
    void AddVisuals(msgs::Link *_msg, const ignition::gazebo::Entity _entity,
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
    void AddLinks(msgs::Model *_msg, const ignition::gazebo::Entity _entity,
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
    void RemoveFromGraph(const ignition::gazebo::Entity _entity,
                         SceneGraphType &_graph)
    {
      for (const auto &vertex : _graph.AdjacentsFrom(_entity))
      {
        RemoveFromGraph(vertex.first, _graph);
      }
      _graph.RemoveVertex(_entity);
    }
  };
}
}
}

using namespace ignition;
using namespace gui;
using namespace plugins;

Scene3DNew::~Scene3DNew()
{
}

//////////////////////////////////////////
void Scene3DNew::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  ignerr << "Scene3DNew LoadConfig" << std::endl;

  if (this->title.empty())
    this->title = "Scene3D new!";

  this->dataPtr->SetPluginItem(this->PluginItem());

  if (_pluginElem)
  {
    auto elem = _pluginElem->FirstChildElement("engine");
    if (nullptr != elem && nullptr != elem->GetText())
    {
      this->dataPtr->SetEngineName(elem->GetText());
      // renderWindow->SetEngineName();
      // there is a problem with displaying ogre2 render textures that are in
      // sRGB format. Workaround for now is to apply gamma correction manually.
      // There maybe a better way to solve the problem by making OpenGL calls..
      if (elem->GetText() == std::string("ogre2"))
        this->PluginItem()->setProperty("gammaCorrect", true);

      elem = _pluginElem->FirstChildElement("scene");
      if (nullptr != elem && nullptr != elem->GetText())
        this->dataPtr->SetSceneName(elem->GetText());
    }

    elem = _pluginElem->FirstChildElement("ambient_light");
    if (nullptr != elem && nullptr != elem->GetText())
    {
      math::Color ambient;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> ambient;
      this->dataPtr->SetAmbientLight(ambient);
    }

    elem = _pluginElem->FirstChildElement("background_color");
    if (nullptr != elem && nullptr != elem->GetText())
    {
      math::Color bgColor;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> bgColor;
      this->dataPtr->SetBackgroundColor(bgColor);
    }

    elem = _pluginElem->FirstChildElement("camera_pose");
    if (nullptr != elem && nullptr != elem->GetText())
    {
      math::Pose3d pose;
      std::stringstream poseStr;
      poseStr << std::string(elem->GetText());
      poseStr >> pose;
      this->dataPtr->SetCameraPose(pose);
    }

    elem = _pluginElem->FirstChildElement("camera_follow");
    if (nullptr != elem)
    {
      if (auto gainElem = elem->FirstChildElement("p_gain"))
      {
        double gain;
        std::stringstream gainStr;
        gainStr << std::string(gainElem->GetText());
        gainStr >> gain;
        if (gain >= 0 && gain <= 1.0)
          this->dataPtr->SetFollowPGain(gain);
        else
          ignerr << "Camera follow p gain outside of range [0, 1]" << std::endl;
      }

      if (auto targetElem = elem->FirstChildElement("target"))
      {
        std::stringstream targetStr;
        targetStr << std::string(targetElem->GetText());
        this->dataPtr->SetFollowTarget(targetStr.str(), true);
      }

      if (auto worldFrameElem = elem->FirstChildElement("world_frame"))
      {
        std::string worldFrameStr =
            common::lowercase(worldFrameElem->GetText());
        if (worldFrameStr == "true" || worldFrameStr == "1")
          this->dataPtr->SetFollowWorldFrame(true);
        else if (worldFrameStr == "false" || worldFrameStr == "0")
          this->dataPtr->SetFollowWorldFrame(false);
        else
        {
          ignerr << "Faild to parse <world_frame> value: " << worldFrameStr
                 << std::endl;
        }
      }

      if (auto offsetElem = elem->FirstChildElement("offset"))
      {
        math::Vector3d offset;
        std::stringstream offsetStr;
        offsetStr << std::string(offsetElem->GetText());
        offsetStr >> offset;
        this->dataPtr->SetFollowOffset(offset);
      }
    }

    elem = _pluginElem->FirstChildElement("fullscreen");
    if (elem != nullptr)
    {
      auto fullscreen = false;
      elem->QueryBoolText(&fullscreen);
      this->dataPtr->SetFullScreen(fullscreen);
    }

    elem = _pluginElem->FirstChildElement("visibility_mask");
    if (elem != nullptr)
    {
      uint32_t visibilityMask = 0xFFFFFFFFu;
      std::stringstream visibilityMaskStr;
      visibilityMaskStr << std::string(elem->GetText());
      bool isHex = common::lowercase(
          visibilityMaskStr.str()).compare(0, 2, "0x") == 0;
      if (isHex)
        visibilityMaskStr >> std::hex >> visibilityMask;
      else
        visibilityMaskStr >> visibilityMask;
      this->dataPtr->SetVisibilityMask(visibilityMask);
    }
  }
}

//////////////////////////////////////////////////
void Scene3DNew::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->IsSceneAvailable())
  {
    return;
  }

  this->dataPtr->Update();

  if (!this->dataInternalPtr->initialized)
  {
    bool newEntity{false};

    // Populate a graph with latest information from all entities

    // Scene graph for new entities. This will be used later to create a scene msg
    // to publish.
    Scene3DNewPrivate::SceneGraphType newGraph;

    // ignerr << "Initializing..." << std::endl;
    // Get all the new worlds
    // TODO(anyone) Only one scene is supported for now
    // extend the sensor system to support mutliple scenes in the future
    _ecm.Each<ignition::gazebo::components::World,
      ignition::gazebo::components::Scene,
      ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity & _entity,
          const ignition::gazebo::components::World *,
          const ignition::gazebo::components::Scene *_scene,
          const ignition::gazebo::components::Name *_name)->bool
        {
          this->dataInternalPtr->worldEntity = _entity;
          this->dataInternalPtr->worldName = _name->Data();
          const sdf::Scene &sceneSdf = _scene->Data();
          this->dataPtr->SetAmbientLight(sceneSdf.Ambient()); //585
          this->dataPtr->SetBackgroundColor(sceneSdf.Background());
          if (sceneSdf.Grid()){
            this->dataPtr->SetShowGrid(true);
          }
          if (sceneSdf.Sky())
          {
            this->dataPtr->SetSkyEnabled(true);
          }
          this->dataInternalPtr->initialized = true;

          newEntity = true;
          newGraph.AddVertex(this->dataInternalPtr->worldName, nullptr,
                             this->dataInternalPtr->worldEntity);

          return true;
        });
    _ecm.Each<ignition::gazebo::components::Model,
      ignition::gazebo::components::Name, ignition::gazebo::components::Pose,
      ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *_name,
            const ignition::gazebo::components::Pose *_pose,
            const ignition::gazebo::components::ParentEntity *_parent)->bool
        {
          auto modelMsg = std::make_shared<msgs::Model>();
          modelMsg->set_id(_entity);
          modelMsg->set_name(_name->Data());
          modelMsg->mutable_pose()->CopyFrom(ignition::msgs::Convert(_pose->Data()));

          // Add to graph
          newGraph.AddVertex(_name->Data(), modelMsg, _entity);
          newGraph.AddEdge({_parent->Data(), _entity}, true);

          newEntity = true;
          return true;
        });

    // Links
    _ecm.Each<ignition::gazebo::components::Link,
      ignition::gazebo::components::Name, ignition::gazebo::components::ParentEntity,
      ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Link *,
            const ignition::gazebo::components::Name *_nameComp,
            const ignition::gazebo::components::ParentEntity *_parentComp,
            const ignition::gazebo::components::Pose *_poseComp) -> bool
        {
          auto linkMsg = std::make_shared<msgs::Link>();
          linkMsg->set_id(_entity);
          linkMsg->set_name(_nameComp->Data());
          linkMsg->mutable_pose()->CopyFrom(ignition::msgs::Convert(_poseComp->Data()));

          // Add to graph
          newGraph.AddVertex(_nameComp->Data(), linkMsg, _entity);
          newGraph.AddEdge({_parentComp->Data(), _entity}, true);

          newEntity = true;
          return true;
        });

    // Visuals
    _ecm.Each<ignition::gazebo::components::Visual,
                     ignition::gazebo::components::Name,
                     ignition::gazebo::components::ParentEntity,
                     ignition::gazebo::components::CastShadows,
                     ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Visual *,
            const ignition::gazebo::components::Name *_nameComp,
            const ignition::gazebo::components::ParentEntity *_parentComp,
            const ignition::gazebo::components::CastShadows *_castShadowsComp,
            const ignition::gazebo::components::Pose *_poseComp) -> bool
        {
          auto visualMsg = std::make_shared<msgs::Visual>();
          visualMsg->set_id(_entity);
          visualMsg->set_parent_id(_parentComp->Data());
          visualMsg->set_name(_nameComp->Data());
          visualMsg->mutable_pose()->CopyFrom(ignition::msgs::Convert(_poseComp->Data()));
          visualMsg->set_cast_shadows(_castShadowsComp->Data());

          // Geometry is optional
          auto geometryComp = _ecm.Component<ignition::gazebo::components::Geometry>(_entity);
          if (geometryComp)
          {
            visualMsg->mutable_geometry()->CopyFrom(
                ignition::gazebo::convert<ignition::msgs::Geometry>(geometryComp->Data()));
          }

          // Material is optional
          auto materialComp = _ecm.Component<ignition::gazebo::components::Material>(_entity);
          if (materialComp)
          {
            visualMsg->mutable_material()->CopyFrom(
                ignition::gazebo::convert<ignition::msgs::Material>(materialComp->Data()));
          }

          // Add to graph
          newGraph.AddVertex(_nameComp->Data(), visualMsg, _entity);
          newGraph.AddEdge({_parentComp->Data(), _entity}, true);

          newEntity = true;
          return true;
        });

    // Lights
    _ecm.Each<ignition::gazebo::components::Light,
                 ignition::gazebo::components::Name,
                 ignition::gazebo::components::ParentEntity,
                 ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Light *_lightComp,
            const ignition::gazebo::components::Name *_nameComp,
            const ignition::gazebo::components::ParentEntity *_parentComp,
            const ignition::gazebo::components::Pose *_poseComp) -> bool
        {
          auto lightMsg = std::make_shared<msgs::Light>();
          lightMsg->CopyFrom(ignition::gazebo::convert<msgs::Light>(_lightComp->Data()));
          lightMsg->set_id(_entity);
          lightMsg->set_parent_id(_parentComp->Data());
          lightMsg->set_name(_nameComp->Data());
          lightMsg->mutable_pose()->CopyFrom(ignition::msgs::Convert(_poseComp->Data()));

          // Add to graph
          newGraph.AddVertex(_nameComp->Data(), lightMsg, _entity);
          newGraph.AddEdge({_parentComp->Data(), _entity}, true);
          newEntity = true;
          return true;
        });

    if (newEntity)
    {
      msgs::Scene sceneMsg;
      this->dataInternalPtr->AddModels(
        &sceneMsg, this->dataInternalPtr->worldEntity, newGraph);
      this->dataInternalPtr->AddLights(
        &sceneMsg, this->dataInternalPtr->worldEntity, newGraph);

      // ignerr << "worldEntity " << this->dataInternalPtr->worldEntity << std::endl;
      // ignerr << "SetScene " << sceneMsg.model_size() << std::endl;
      // ignerr << "newGraph " << newGraph.Vertices().size() << std::endl;
      this->dataPtr->SetScene(sceneMsg);
    }
  }
  else
  {
    // update entity poses
    _ecm.Each<ignition::gazebo::components::Model,
              ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Model *,
          const ignition::gazebo::components::Pose *_pose)->bool
        {
          this->dataInternalPtr->entityPoses[_entity] = _pose->Data();
          return true;
        });

    _ecm.Each<ignition::gazebo::components::Link,
              ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Link *,
          const ignition::gazebo::components::Pose *_pose)->bool
        {
          this->dataInternalPtr->entityPoses[_entity] = _pose->Data();
          return true;
        });

    // visuals
    _ecm.Each<ignition::gazebo::components::Visual,
              ignition::gazebo::components::Pose >(
        [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Visual *,
          const ignition::gazebo::components::Pose *_pose)->bool
        {
          this->dataInternalPtr->entityPoses[_entity] = _pose->Data();
          return true;
        });
    this->dataPtr->UpdatePoses(this->dataInternalPtr->entityPoses);
  }
}

//////////////////////////////////////////
Scene3DNew::Scene3DNew() : GuiSystem(),
    dataPtr(new Scene3DInterface()), dataInternalPtr(new Scene3DNewPrivate())
{
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::plugins::Scene3DNew,
                    ignition::gui::Plugin)
