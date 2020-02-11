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

#include <sdf/Element.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/parser.hh>
#include <sdf/Scene.hh>
#include <sdf/SDFImpl.hh>
#include <sdf/Visual.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/rendering.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CastShadows.hh"
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
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"

#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;

// Private data class.
class ignition::gazebo::RenderUtilPrivate
{
  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void CreateRenderingEntities(const EntityComponentManager &_ecm,
      const UpdateInfo &_info);

  /// \brief Remove rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void RemoveRenderingEntities(const EntityComponentManager &_ecm,
      const UpdateInfo &_info);

  /// \brief Update rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void UpdateRenderingEntities(const EntityComponentManager &_ecm);

  /// \brief Name of rendering engine
  public: std::string engineName = "ogre2";

  /// \brief Name of scene
  public: std::string sceneName = "scene";

  /// \brief Scene background color
  public: math::Color backgroundColor = math::Color::Black;

  /// \brief Ambient color
  public: math::Color ambientLight = math::Color(1.0, 1.0, 1.0, 1.0);

  /// \brief Scene manager
  public: SceneManager sceneManager;

  /// \brief Pointer to rendering engine.
  public: ignition::rendering::RenderEngine *engine{nullptr};

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Flag to indicate if the current GL context should be used
  public: bool useCurrentGLContext = false;

  /// \brief New scenes to be created
  public: std::vector<sdf::Scene> newScenes;

  /// \brief New models to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id, [3] sim iteration
  public: std::vector<std::tuple<Entity, sdf::Model, Entity, uint64_t>>
      newModels;

  /// \brief New links to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Link, Entity>> newLinks;

  /// \brief New visuals to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Visual, Entity>> newVisuals;

  /// \brief New lights to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Light, Entity>> newLights;

  /// \brief New sensors to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Sensor, Entity>>
      newSensors;

  /// \brief Map of ids of entites to be removed and sim iteration when the
  /// remove request is received
  public: std::map<Entity, uint64_t> removeEntities;

  /// \brief A map of entity ids and pose updates.
  public: std::map<Entity, math::Pose3d> entityPoses;

  /// \brief Mutex to protect updates
  public: std::mutex updateMutex;

  //// \brief Flag to indicate whether to create sensors
  public: bool enableSensors = false;

  /// \brief Callback function for creating sensors.
  /// The function args are: entity id, sensor sdf, and parent name.
  /// The function returns the id of the rendering sensor created.
  public: std::function<
      std::string(const sdf::Sensor &, const std::string &)>
      createSensorCb;

  /// \brief Map of currently selected entities mapping entity id to node id
  public: std::map<Entity, uint64_t> selectedEntities;

  /// \brief Map of original emissive colors for nodes currently highlighted.
  public: std::map<std::string, math::Color> originalEmissive;

  /// \brief Whether the transform gizmo is being dragged.
  public: bool transformActive{false};

  /// \brief Highlight a node and all its children.
  /// \param[in] _node Node to be highlighted
  /// TODO(anyone) On future versions, use a bounding box instead
  public: void HighlightNode(const rendering::NodePtr &_node);

  /// \brief Restore a highlighted node to normal.
  /// \param[in] _node Node to be restored.
  /// TODO(anyone) On future versions, use a bounding box instead
  public: void LowlightNode(const rendering::NodePtr &_node);
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
  IGN_PROFILE("RenderUtil::UpdateFromECM");
  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
  this->dataPtr->CreateRenderingEntities(_ecm, _info);
  if (_info.dt != std::chrono::steady_clock::duration::zero())
    this->dataPtr->UpdateRenderingEntities(_ecm);
  this->dataPtr->RemoveRenderingEntities(_ecm, _info);
}

//////////////////////////////////////////////////
int RenderUtil::PendingSensors() const
{
  if (!this->dataPtr->initialized)
    return -1;

  if (!this->dataPtr->scene)
    return -1;

  this->dataPtr->updateMutex.lock();
  int nSensors = this->dataPtr->newSensors.size();
  this->dataPtr->updateMutex.unlock();
  return nSensors;
}

//////////////////////////////////////////////////
void RenderUtil::Update()
{
  IGN_PROFILE("RenderUtil::Update");
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

  std::vector<std::tuple<Entity, sdf::Sensor, Entity>> newSensors;
  if (this->dataPtr->enableSensors)
  {
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
    if (scene.Grid() && !this->dataPtr->enableSensors)
      this->ShowGrid();
    // only one scene so break
    break;
  }

  // remove existing entities
  // \todo(anyone) Remove sensors
  {
    IGN_PROFILE("RenderUtil::Update Remove");
    for (auto &entity : removeEntities)
    {
      auto node = this->dataPtr->sceneManager.NodeById(entity.first);
      if (!this->dataPtr->selectedEntities.empty() &&
          this->dataPtr->selectedEntities.find(entity.first) !=
          this->dataPtr->selectedEntities.end())
      {
        this->dataPtr->selectedEntities.erase(entity.first);
        this->dataPtr->sceneManager.RemoveEntity(entity.first);
      }
    }
  }

  // create new entities
  {
    IGN_PROFILE("RenderUtil::Update Create");
    for (auto &model : newModels)
    {
      uint64_t iteration = std::get<3>(model);
      Entity entityId = std::get<0>(model);
      // since entites to be created and removed are queued, we need
      // to check their creation timestamp to make sure we do not create a new
      // entity when there is also a remove request with a more recent
      // timestamp
      // \todo(anyone) add test to check scene entities are properly added
      // and removed.
      auto removeIt = removeEntities.find(entityId);
      if (removeIt != removeEntities.end())
      {
        uint64_t removeIteration = removeIt->second;
        if (iteration < removeIteration)
          continue;
      }
      this->dataPtr->sceneManager.CreateModel(
          entityId, std::get<1>(model), std::get<2>(model));
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
      this->dataPtr->sceneManager.CreateLight(
          std::get<0>(light), std::get<1>(light), std::get<2>(light));
    }

    if (this->dataPtr->enableSensors && this->dataPtr->createSensorCb)
    {
      for (auto &sensor : newSensors)
      {
         Entity entity = std::get<0>(sensor);
         sdf::Sensor dataSdf = std::get<1>(sensor);
         Entity parent = std::get<2>(sensor);

         // two sensors with the same name cause conflicts. We'll need to use
         // scoped names
         // TODO(anyone) do this in ign-sensors?
         auto parentNode = this->dataPtr->sceneManager.NodeById(parent);
         if (!parentNode)
         {
           ignerr << "Failed to create sensor with name[" << dataSdf.Name()
                  << "] for entity [" << entity
                  << "]. Parent not found with ID[" << parent << "]."
                  << std::endl;
           continue;
         }

         std::string sensorName =
             this->dataPtr->createSensorCb(dataSdf, parentNode->Name());
         // Add to the system's scene manager
         if (!this->dataPtr->sceneManager.AddSensor(entity, sensorName, parent))
         {
           ignerr << "Failed to create sensor [" << sensorName << "]"
                  << std::endl;
         }
      }
    }
  }

  // update entities' pose
  {
    IGN_PROFILE("RenderUtil::Update Poses");
    for (auto &pose : entityPoses)
    {
      auto node = this->dataPtr->sceneManager.NodeById(pose.first);
      if (!node)
        continue;

      // Don't move entity being manipulated (last selected)
      // TODO(anyone) Check top level visual instead of parent
      Entity entityId = this->EntityFromNode(node->Parent());
      if (this->dataPtr->transformActive &&
          (pose.first == (*this->dataPtr->selectedEntities.rbegin()).first ||
          entityId == (*this->dataPtr->selectedEntities.rbegin()).first))
      {
        continue;
      }

      node->SetLocalPose(pose.second);
    }
  }
}

//////////////////////////////////////////////////
void RenderUtilPrivate::CreateRenderingEntities(
    const EntityComponentManager &_ecm, const UpdateInfo &_info)
{
  IGN_PROFILE("RenderUtilPrivate::CreateRenderingEntities");
  auto addNewSensor = [&_ecm, this](Entity _entity, const sdf::Sensor &_sdfData,
                                    Entity _parent,
                                    const std::string &_topicSuffix)
  {
    sdf::Sensor sdfDataCopy(_sdfData);
    std::string sensorScopedName =
        removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
    sdfDataCopy.SetName(sensorScopedName);
    // check topic
    if (sdfDataCopy.Topic().empty())
    {
      sdfDataCopy.SetTopic(scopedName(_entity, _ecm) + _topicSuffix);
    }
    this->newSensors.push_back(
        std::make_tuple(_entity, std::move(sdfDataCopy), _parent));
  };

  const std::string cameraSuffix{"/image"};
  const std::string depthCameraSuffix{"/depth_image"};
  const std::string rgbdCameraSuffix{""};
  const std::string gpuLidarSuffix{"/scan"};

  // Treat all pre-existent entities as new at startup
  // TODO(anyone) refactor Each and EachNew below to reduce duplicate code
  if (!this->initialized)
  {
    // Get all the new worlds
    // TODO(anyone) Only one scene is supported for now
    // extend the sensor system to support mutliple scenes in the future
    _ecm.Each<components::World, components::Scene>(
        [&](const Entity & _entity,
          const components::World *,
          const components::Scene *_scene)->bool
        {
          this->sceneManager.SetWorldId(_entity);
          const sdf::Scene &sceneSdf = _scene->Data();
          this->newScenes.push_back(sceneSdf);
          return true;
        });


    _ecm.Each<components::Model, components::Name, components::Pose,
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
          this->newModels.push_back(
              std::make_tuple(_entity, model, _parent->Data(),
              _info.iterations));
          return true;
        });

    _ecm.Each<components::Link, components::Name, components::Pose,
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
          this->newLinks.push_back(
              std::make_tuple(_entity, link, _parent->Data()));
          return true;
        });

    // visuals
    _ecm.Each<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Geometry *_geom,
            const components::CastShadows *_castShadows,
            const components::ParentEntity *_parent)->bool
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetPose(_pose->Data());
          visual.SetGeom(_geom->Data());
          visual.SetCastShadows(_castShadows->Data());

          // Optional components
          auto material = _ecm.Component<components::Material>(_entity);
          if (material != nullptr)
          {
            visual.SetMaterial(material->Data());
          }

          this->newVisuals.push_back(
              std::make_tuple(_entity, visual, _parent->Data()));
          return true;
        });

    // lights
    _ecm.Each<components::Light, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Light *_light,
            const components::ParentEntity *_parent) -> bool
        {
          this->newLights.push_back(
              std::make_tuple(_entity, _light->Data(), _parent->Data()));
          return true;
        });

    if (this->enableSensors)
    {
      // Create cameras
      _ecm.Each<components::Camera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Camera *_camera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _camera->Data(), _parent->Data(),
                         cameraSuffix);
            return true;
          });

      // Create depth cameras
      _ecm.Each<components::DepthCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::DepthCamera *_depthCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _depthCamera->Data(), _parent->Data(),
                         depthCameraSuffix);
            return true;
          });

      // Create rgbd cameras
      _ecm.Each<components::RgbdCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::RgbdCamera *_rgbdCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _rgbdCamera->Data(), _parent->Data(),
                         rgbdCameraSuffix);
            return true;
          });

      // Create gpu lidar
      _ecm.Each<components::GpuLidar, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::GpuLidar *_gpuLidar,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _gpuLidar->Data(), _parent->Data(),
                         gpuLidarSuffix);
            return true;
          });
    }
    this->initialized = true;
  }
  else
  {
    // Get all the new worlds
    // TODO(anyone) Only one scene is supported for now
    // extend the sensor system to support mutliple scenes in the future
    _ecm.EachNew<components::World, components::Scene>(
        [&](const Entity & _entity,
          const components::World *,
          const components::Scene *_scene)->bool
        {
          this->sceneManager.SetWorldId(_entity);
          const sdf::Scene &sceneSdf = _scene->Data();
          this->newScenes.push_back(sceneSdf);
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
          this->newModels.push_back(
              std::make_tuple(_entity, model, _parent->Data(),
              _info.iterations));
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
          this->newLinks.push_back(
              std::make_tuple(_entity, link, _parent->Data()));
          return true;
        });

    // visuals
    _ecm.EachNew<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Geometry *_geom,
            const components::CastShadows *_castShadows,
            const components::ParentEntity *_parent)->bool
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetPose(_pose->Data());
          visual.SetGeom(_geom->Data());
          visual.SetCastShadows(_castShadows->Data());

          // Optional components
          auto material = _ecm.Component<components::Material>(_entity);
          if (material != nullptr)
          {
            visual.SetMaterial(material->Data());
          }

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
            addNewSensor(_entity, _camera->Data(), _parent->Data(),
                         cameraSuffix);
            return true;
          });

      // Create depth cameras
      _ecm.EachNew<components::DepthCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::DepthCamera *_depthCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _depthCamera->Data(), _parent->Data(),
                         depthCameraSuffix);
            return true;
          });

      // Create RGBD cameras
      _ecm.EachNew<components::RgbdCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::RgbdCamera *_rgbdCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _rgbdCamera->Data(), _parent->Data(),
                         rgbdCameraSuffix);
            return true;
          });

      // Create gpu lidar
      _ecm.EachNew<components::GpuLidar, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::GpuLidar *_gpuLidar,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _gpuLidar->Data(), _parent->Data(),
                         gpuLidarSuffix);
            return true;
          });
    }
  }
}

//////////////////////////////////////////////////
void RenderUtilPrivate::UpdateRenderingEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("RenderUtilPrivate::UpdateRenderingEntities");
  _ecm.Each<components::Model, components::Pose>(
      [&](const Entity &_entity,
        const components::Model *,
        const components::Pose *_pose)->bool
      {
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

  // Update RGBD cameras
  _ecm.Each<components::RgbdCamera, components::Pose>(
      [&](const Entity &_entity,
        const components::RgbdCamera *,
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
void RenderUtilPrivate::RemoveRenderingEntities(
    const EntityComponentManager &_ecm, const UpdateInfo &_info)
{
  IGN_PROFILE("RenderUtilPrivate::RemoveRenderingEntities");
  _ecm.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  _ecm.EachRemoved<components::Link>(
      [&](const Entity &_entity, const components::Link *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // visuals
  _ecm.EachRemoved<components::Visual>(
      [&](const Entity &_entity, const components::Visual *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // lights
  _ecm.EachRemoved<components::Light>(
      [&](const Entity &_entity, const components::Light *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // cameras
  _ecm.EachRemoved<components::Camera>(
    [&](const Entity &_entity, const components::Camera *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // depth cameras
  _ecm.EachRemoved<components::DepthCamera>(
    [&](const Entity &_entity, const components::DepthCamera *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // rgbd cameras
  _ecm.EachRemoved<components::RgbdCamera>(
    [&](const Entity &_entity, const components::RgbdCamera *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // gpu_lidars
  _ecm.EachRemoved<components::GpuLidar>(
    [&](const Entity &_entity, const components::GpuLidar *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
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
void RenderUtil::ShowGrid()
{
  rendering::VisualPtr root = this->dataPtr->scene->RootVisual();

  // create gray material
  rendering::MaterialPtr gray = this->dataPtr->scene->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

  // create grid visual
  rendering::VisualPtr visual = this->dataPtr->scene->CreateVisual();
  rendering::GridPtr gridGeom = this->dataPtr->scene->CreateGrid();
  if (!gridGeom)
  {
    ignwarn << "Failed to create grid for scene ["
      << this->dataPtr->scene->Name() << "] on engine ["
        << this->dataPtr->scene->Engine()->Name() << "]"
          << std::endl;
    return;
  }
  gridGeom->SetCellCount(20);
  gridGeom->SetCellLength(1);
  gridGeom->SetVerticalCellCount(0);
  visual->AddGeometry(gridGeom);
  visual->SetLocalPosition(0, 0, 0.015);
  visual->SetMaterial(gray);
  root->AddChild(visual);
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
void RenderUtil::SetUseCurrentGLContext(bool _enable)
{
  this->dataPtr->useCurrentGLContext = _enable;
}

/////////////////////////////////////////////////
void RenderUtil::SetEnableSensors(bool _enable,
    std::function<std::string(const sdf::Sensor &, const std::string &)>
    _createSensorCb)
{
  this->dataPtr->enableSensors = _enable;
  this->dataPtr->createSensorCb = std::move(_createSensorCb);
}

/////////////////////////////////////////////////
SceneManager &RenderUtil::SceneManager()
{
  return this->dataPtr->sceneManager;
}

/////////////////////////////////////////////////
Entity RenderUtil::EntityFromNode(const rendering::NodePtr &_node)
{
  return this->dataPtr->sceneManager.EntityFromNode(_node);
}

/////////////////////////////////////////////////
void RenderUtil::SetSelectedEntity(rendering::NodePtr _node)
{
  if (!_node)
    return;

  Entity entityId = this->EntityFromNode(_node);

  if (entityId == kNullEntity)
    return;

  this->dataPtr->selectedEntities.insert(
      std::pair<Entity, uint64_t>(entityId, _node->Id()));
  this->dataPtr->HighlightNode(_node);
}

/////////////////////////////////////////////////
void RenderUtil::DeselectAllEntities()
{
  for (const auto &entity : this->dataPtr->selectedEntities)
  {
    auto node = this->dataPtr->sceneManager.NodeById(entity.first);
    this->dataPtr->LowlightNode(node);
  }
  this->dataPtr->selectedEntities.clear();
  this->dataPtr->originalEmissive.clear();
}

/////////////////////////////////////////////////
rendering::NodePtr RenderUtil::SelectedEntity() const
{
  // Return most recently selected node
  auto node = this->dataPtr->sceneManager.NodeById(
      (*(this->dataPtr->selectedEntities.rbegin())).first);
  return node;
}

/////////////////////////////////////////////////
std::map<Entity, Entity> RenderUtil::SelectedEntities() const
{
  return this->dataPtr->selectedEntities;
}

/////////////////////////////////////////////////
void RenderUtil::SetTransformActive(bool _active)
{
  this->dataPtr->transformActive = _active;
}

////////////////////////////////////////////////
void RenderUtilPrivate::HighlightNode(const rendering::NodePtr &_node)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->HighlightNode(child);
  }

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (nullptr == vis)
    return;

  // Visual material
  auto visMat = vis->Material();
  if (nullptr != visMat)
  {
    // If the entity isn't already highlighted, highlight it
    if (this->originalEmissive.find(vis->Name()) ==
        this->originalEmissive.end())
    {
      this->originalEmissive[vis->Name()] = visMat->Emissive();
      visMat->SetEmissive(visMat->Emissive() + math::Color(0.5, 0.5, 0.5));
      vis->SetMaterial(visMat);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    // If the entity isn't already highlighted, highlight it
    if (this->originalEmissive.find(geom->Name()) ==
        this->originalEmissive.end())
    {
      this->originalEmissive[geom->Name()] = geomMat->Emissive();
      geomMat->SetEmissive(geomMat->Emissive() + math::Color(0.5, 0.5, 0.5));
      geom->SetMaterial(geomMat);
    }
  }
}

////////////////////////////////////////////////
void RenderUtilPrivate::LowlightNode(const rendering::NodePtr &_node)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->LowlightNode(child);
  }

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (nullptr == vis)
    return;

  // Visual material
  auto visMat = vis->Material();
  if (nullptr != visMat)
  {
    auto visEmissive = this->originalEmissive.find(vis->Name());
    if (visEmissive != this->originalEmissive.end())
    {
      visMat->SetEmissive(visEmissive->second);
      vis->SetMaterial(visMat);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    auto geomEmissive = this->originalEmissive.find(geom->Name());
    if (geomEmissive != this->originalEmissive.end())
    {
      geomMat->SetEmissive(geomEmissive->second);
      geom->SetMaterial(geomMat);
    }
  }
}
