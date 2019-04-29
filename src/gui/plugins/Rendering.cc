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

#include <map>
#include <vector>

#include <sdf/sdf.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/gziface/SceneManager.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "Rendering.hh"

using namespace ignition;
using namespace gazebo;

// Private data class.
class ignition::gazebo::RenderUtilPrivate
{
  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void CreateRenderingEntities(const EntityComponentManager &_ecm);

  /// \brief Remove rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void RemoveRenderingEntities(const EntityComponentManager &_ecm);

  /// \brief Update rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void UpdateRenderingEntities(const EntityComponentManager &_ecm);

  /// \brief Name of rendering engine
  public: std::string engineName = "ogre2";

  /// \brief Name of scene
  public: std::string sceneName = "scene";

  /// \brief Initial Camera pose
  public: math::Pose3d cameraPose = math::Pose3d(0, 0, 2, 0, 0.4, 0);

  /// \brief Scene background color
  public: math::Color backgroundColor = math::Color::Black;

  /// \brief Ambient color
  public: math::Color ambientLight = math::Color(0.3, 0.3, 0.3, 1.0);

  /// \brief Scene manager
  public: rendering::gziface::SceneManager sceneManager;

  /// \brief Pointer to rendering engine.
  public: ignition::rendering::RenderEngine *engine{nullptr};

  /// \brief Map of Gazebo entities to their respective IDs within ign-sensors.
  /// Note that both of these are different from node's ID in ign-rendering.
  public: std::map<Entity, uint64_t> entityToSensorId;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Flag to indicate if the current GL context should be used
  public: bool useCurrentGLContext = false;

  public: std::vector<sdf::Scene> newScenes;
  public: std::vector<std::tuple<uint64_t, sdf::Model, uint64_t>> newModels;
  public: std::vector<std::tuple<uint64_t, sdf::Link, uint64_t>> newLinks;
  public: std::vector<std::tuple<uint64_t, sdf::Visual, uint64_t>> newVisuals;
  public: std::vector<std::tuple<uint64_t, sdf::Light, uint64_t>> newLights;
  public: std::vector<std::tuple<uint64_t, std::string, uint64_t>>
      newSensors;
  public: std::set<uint64_t> removeEntities;
  public: std::map<uint64_t, math::Pose3d> entityPoses;
  public: std::mutex updateMutex;

  public: bool enableSensors = false;
  public: std::function<unsigned int(uint64_t, sdf::ElementPtr, std::string)>
      createSensorCb;
};

//////////////////////////////////////////////////
RenderUtil::RenderUtil() : dataPtr(std::make_unique<RenderUtilPrivate>())
{
}

//////////////////////////////////////////////////
RenderUtil::~RenderUtil() = default;


//////////////////////////////////////////////////
rendering::ScenePtr RenderUtil::Scene() const
{
  return this->dataPtr->scene;
}

//////////////////////////////////////////////////
void RenderUtil::UpdateFromECM(const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
  this->dataPtr->CreateRenderingEntities(_ecm);
  this->dataPtr->UpdateRenderingEntities(_ecm);
  this->dataPtr->RemoveRenderingEntities(_ecm);
}

//////////////////////////////////////////////////
void RenderUtil::Update()
{
  if (!this->dataPtr->initialized)
    return;

  if (!this->dataPtr->scene)
    return;

  this->dataPtr->updateMutex.lock();
  auto newScenes = std::move(this->dataPtr->newScenes);
  auto newModels = std::move(this->dataPtr->newModels);
  auto newLinks = std::move(this->dataPtr->newLinks);
  auto newVisuals = std::move(this->dataPtr->newVisuals);
  auto newLights = std::move(this->dataPtr->newLights);
  auto removeEntities = std::move(this->dataPtr->removeEntities);
  auto entityPoses = std::move(this->dataPtr->entityPoses);

  this->dataPtr->newScenes.clear();
  this->dataPtr->newModels.clear();
  this->dataPtr->newLinks.clear();
  this->dataPtr->newVisuals.clear();
  this->dataPtr->newLights.clear();
  this->dataPtr->removeEntities.clear();
  this->dataPtr->entityPoses.clear();

  std::vector<std::tuple<uint64_t, std::string, uint64_t>> newSensors;
  if (this->dataPtr->enableSensors)
  {
    // todo(anyone) switch to use std::move once sensors have been updated
    // to use sdf DOM
    // std::copy(this->dataPtr->newSensors.begin(),
    //     this->dataPtr->newSensors.end(),
    //     std::back_inserter(newSensors));
    newSensors = std::move(this->dataPtr->newSensors);
    this->dataPtr->newSensors.clear();
  }

  this->dataPtr->updateMutex.unlock();

  // scene - only one scene is supported for now
  // extend the sensor system to support mutliple scenes in the future
  for (auto &scene : newScenes)
  {
    this->dataPtr->scene->SetAmbientLight(scene.Ambient());
    this->dataPtr->scene->SetBackgroundColor(scene.Background());
    // only one scene so break
    break;
  }

  // create new entities
  for (auto &model : newModels)
  {
    std::cerr << "creating model " << std::get<0>(model) << std::endl;
    sdf::Model m = std::get<1>(model);
    this->dataPtr->sceneManager.CreateModel(
        std::get<0>(model), std::get<1>(model), std::get<2>(model));
  }

  for (auto &link : newLinks)
  {
    this->dataPtr->sceneManager.CreateLink(
        std::get<0>(link), std::get<1>(link), std::get<2>(link));
  }

  for (auto &visual : newVisuals)
  {
    this->dataPtr->sceneManager.CreateVisual(
        std::get<0>(visual), std::get<1>(visual), std::get<2>(visual));
  }

  for (auto &light : newLights)
  {
    std::cerr << "creating light " << std::get<0>(light) << std::endl;
    this->dataPtr->sceneManager.CreateLight(
        std::get<0>(light), std::get<1>(light), std::get<2>(light));
  }

  if (this->dataPtr->enableSensors && this->dataPtr->createSensorCb)
  {
    for (auto &sensor : newSensors)
    {
       uint64_t entity = std::get<0>(sensor);
       std::string dataStr = std::get<1>(sensor);
       uint64_t parent = std::get<2>(sensor);

       static sdf::SDFPtr sdfParsed;
       if (!sdfParsed)
       {
         sdfParsed.reset(new sdf::SDF());
         sdf::init(sdfParsed);
       }
       std::stringstream stream;
       stream << "<?xml version='1.0'?>"
              << "<sdf version='1.6'>"
              << dataStr
              << "</sdf>";

       if (!sdf::readString(stream.str(), sdfParsed))
       {
         ignerr << "Error parsing sensors sdf" << std::endl;
         continue;
       }
       sdf::ElementPtr data = sdfParsed->Root()->GetElement("sensor");

       // two sensors with the same name cause conflicts. We'll need to use scoped names
       // TODO(anyone) do this in ign-sensors?
       auto parentNode = this->dataPtr->sceneManager.NodeById(parent);
       if (!parentNode)
       {
         ignerr << "Failed to create sensor for entity [" << entity
                << "]. Parent not found." << std::endl;
         continue;
       }

       std::string scopedName = parentNode->Name() + "::"
           + data->Get<std::string>("name");
       data->GetAttribute("name")->Set(scopedName);

       unsigned int renderingId =
           this->dataPtr->createSensorCb(entity, data, parentNode->Name());
       // // Create within ign-sensors
       // auto sensor =
       //     this->sensorManager.CreateSensor<sensors::CameraSensor>(data);
       // if (nullptr == sensor || sensors::NO_SENSOR == sensor->Id())
       // {
       //   ignerr << "Failed to create sensor [" << scopedName << "]"
       //          << std::endl;
       // }
       // // Set the scene so it can create the rendering camera
       // sensor->SetScene(this->scene);

       // Add to the system's scene manager
       if (!this->dataPtr->sceneManager.AddSensor(entity, renderingId, parent))
       {
         ignerr << "Failed to create sensor [" << scopedName << "]"
                << std::endl;
       }
       // else
       // {
       //   this->entityToSensorId[_entity] = sensor->Id();
       //   sensor->SetParent(parent->Name());
       //   sensor->SetScene(this->scene);
       // }
    }
  }


  // remove existing entities
  for (auto &entity: removeEntities)
  {
    this->dataPtr->sceneManager.RemoveEntity(entity);
  }


  // update entities' pose
  for (auto &pose : entityPoses)
  {
    auto node = this->dataPtr->sceneManager.NodeById(pose.first);
    if (node)
      node->SetLocalPose(pose.second);
  }
}

//////////////////////////////////////////////////
void RenderUtilPrivate::CreateRenderingEntities(const EntityComponentManager &_ecm)
{
  // Get all the new worlds
  // TODO(anyone) Only one scene is supported for now
  // extend the sensor system to support mutliple scenes in the future
  _ecm.EachNew<components::World, components::Scene>(
      [&](const Entity & _entity,
        const components::World * /* _world */ ,
        const components::Scene *_scene)->bool
      {
        this->sceneManager.SetWorldId(_entity);
        std::cerr << "each new world / scene  " << _entity << std::endl;
        const sdf::Scene &sceneSdf = _scene->Data();
        this->newScenes.push_back(sceneSdf);
        // this->scene->SetAmbientLight(sceneSdf.Ambient());
        // this->scene->SetBackgroundColor(sceneSdf.Background());
        return true;
      });


  _ecm.EachNew<components::Model, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        sdf::Model model;
        model.SetName(_name->Data());
        model.SetPose(_pose->Data());
        // this->sceneManager.CreateModel(_entity, model, _parent->Data());
        this->newModels.push_back(
            std::make_tuple(_entity, model, _parent->Data()));
        return true;
      });

  _ecm.EachNew<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {
        sdf::Link link;
        link.SetName(_name->Data());
        link.SetPose(_pose->Data());
        // this->sceneManager.CreateLink(_entity, link, _parent->Data());
        this->newLinks.push_back(
            std::make_tuple(_entity, link, _parent->Data()));
        return true;
      });

  // visuals
  _ecm.EachNew<components::Visual, components::Name, components::Pose,
            components::Geometry, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Visual *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::Geometry *_geom,
          const components::ParentEntity *_parent)->bool
      {
        sdf::Visual visual;
        visual.SetName(_name->Data());
        visual.SetPose(_pose->Data());
        visual.SetGeom(_geom->Data());

        // Optional components
        auto material = _ecm.Component<components::Material>(_entity);
        if (material != nullptr)
        {
          visual.SetMaterial(material->Data());
        }

        // this->sceneManager.CreateVisual(_entity, visual, _parent->Data());
        this->newVisuals.push_back(
            std::make_tuple(_entity, visual, _parent->Data()));
        return true;
      });

  // lights
  _ecm.EachNew<components::Light, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Light *_light,
          const components::ParentEntity *_parent) -> bool
      {
        //this->sceneManager.CreateLight(_entity, _light->Data(), _parent->Data());
        this->newLights.push_back(
            std::make_tuple(_entity, _light->Data(), _parent->Data()));
        return true;
      });

  if (this->enableSensors)
  {
    // Create cameras
    _ecm.EachNew<components::Camera, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Camera *_camera,
          const components::ParentEntity *_parent)->bool
        {
          this->newSensors.push_back(
              std::make_tuple(_entity, _camera->Data()->ToString(""),
              _parent->Data()));
          return true;
        });

    // Create depth cameras
    _ecm.EachNew<components::DepthCamera, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::DepthCamera *_depthCamera,
          const components::ParentEntity *_parent)->bool
        {
          this->newSensors.push_back(
              std::make_tuple(_entity, _depthCamera->Data()->ToString(""),
              _parent->Data()));
          return true;
        });


    // Create gpu lidar
    _ecm.EachNew<components::GpuLidar, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::GpuLidar *_gpuLidar,
          const components::ParentEntity *_parent)->bool
        {
          this->newSensors.push_back(
              std::make_tuple(_entity, _gpuLidar->Data()->ToString(""),
               _parent->Data()));
          return true;
        });
  }
}

//////////////////////////////////////////////////
void RenderUtilPrivate::UpdateRenderingEntities(const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Model, components::Pose>(
      [&](const Entity &_entity,
        const components::Model *,
        const components::Pose *_pose)->bool
      {
        // auto node = this->sceneManager.NodeById(_entity);
        // if (node)
        // {
        //   node->SetLocalPose(_pose->Data());
        // }
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  _ecm.Each<components::Link, components::Pose>(
      [&](const Entity &_entity,
        const components::Link *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  // visuals
  _ecm.Each<components::Visual, components::Pose >(
      [&](const Entity &_entity,
        const components::Visual *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  // lights
  _ecm.Each<components::Light, components::Pose>(
      [&](const Entity &_entity,
        const components::Light *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  // Update cameras
  _ecm.Each<components::Camera, components::Pose>(
    [&](const Entity &_entity,
        const components::Camera *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  // Update depth cameras
  _ecm.Each<components::DepthCamera, components::Pose>(
    [&](const Entity &_entity,
        const components::DepthCamera *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });

  // Update gpu_lidar
  _ecm.Each<components::GpuLidar, components::Pose>(
    [&](const Entity &_entity,
        const components::GpuLidar *,
        const components::Pose *_pose)->bool
      {
        this->entityPoses[_entity] = _pose->Data();
        return true;
      });
}

//////////////////////////////////////////////////
void RenderUtilPrivate::RemoveRenderingEntities(const EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });

  _ecm.EachRemoved<components::Link>(
      [&](const Entity &_entity, const components::Link *)->bool
      {
        removeEntities.insert(_entity);
        return true;
      });

  // visuals
  _ecm.EachRemoved<components::Visual>(
      [&](const Entity &_entity, const components::Visual *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });

  // lights
  _ecm.EachRemoved<components::Light>(
      [&](const Entity &_entity, const components::Light *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });

  // cameras
  _ecm.EachRemoved<components::Camera>(
    [&](const Entity &_entity, const components::Camera *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });

  // depth cameras
  _ecm.EachRemoved<components::DepthCamera>(
    [&](const Entity &_entity, const components::DepthCamera *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });

  // gpu_lidars
  _ecm.EachRemoved<components::GpuLidar>(
    [&](const Entity &_entity, const components::GpuLidar *)->bool
      {
        // this->sceneManager.RemoveEntity(_entity);
        removeEntities.insert(_entity);
        return true;
      });
}

/////////////////////////////////////////////////
void RenderUtil::Init()
{
  std::map<std::string, std::string> params;
  if (this->dataPtr->useCurrentGLContext)
    params["useCurrentGLContext"] = "1";
  this->dataPtr->engine = rendering::engine(this->dataPtr->engineName, params);
  if (!this->dataPtr->engine)
  {
    ignerr << "Engine [" << this->dataPtr->engineName << "] is not supported"
           << std::endl;
    return;
  }

  // Scene
  this->dataPtr->scene =
      this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  if (!this->dataPtr->scene)
  {
    igndbg << "Create scene [" << this->dataPtr->sceneName << "]" << std::endl;
    this->dataPtr->scene =
        this->dataPtr->engine->CreateScene(this->dataPtr->sceneName);
    this->dataPtr->scene->SetAmbientLight(this->dataPtr->ambientLight);
    this->dataPtr->scene->SetBackgroundColor(this->dataPtr->backgroundColor);
  }
  this->dataPtr->sceneManager.SetScene(this->dataPtr->scene);
  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void RenderUtil::SetBackgroundColor(const math::Color &_color)
{
  this->dataPtr->backgroundColor = _color;
}

/////////////////////////////////////////////////
void RenderUtil::SetAmbientLight(const math::Color &_ambient)
{
  this->dataPtr->ambientLight  = _ambient;
}

/////////////////////////////////////////////////
void RenderUtil::SetEngineName(const std::string &_name)
{
  this->dataPtr->engineName = _name;
}

/////////////////////////////////////////////////
std::string RenderUtil::EngineName() const
{
  return this->dataPtr->engineName;
}

/////////////////////////////////////////////////
void RenderUtil::SetSceneName(const std::string &_name)
{
  this->dataPtr->sceneName = _name;
}

/////////////////////////////////////////////////
std::string RenderUtil::SceneName() const
{
  return this->dataPtr->sceneName;
}

/////////////////////////////////////////////////
void RenderUtil::SetCameraPose(const math::Pose3d &_pose)
{
  this->dataPtr->cameraPose = _pose;
}

/////////////////////////////////////////////////
math::Pose3d RenderUtil::CameraPose() const
{
  return this->dataPtr->cameraPose;
}

/////////////////////////////////////////////////
void RenderUtil::SetUseCurrentGLContext(bool _enable)
{
  this->dataPtr->useCurrentGLContext = _enable;
}

/////////////////////////////////////////////////
void RenderUtil::SetEnableSensors(bool _enable,
    const std::function<unsigned int(uint64_t, sdf::ElementPtr, std::string)>
    &_createSensorCb)
{
  this->dataPtr->enableSensors = _enable;
  this->dataPtr->createSensorCb = _createSensorCb;
}
