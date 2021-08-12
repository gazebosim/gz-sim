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
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

#include <sdf/Actor.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/parser.hh>
#include <sdf/Scene.hh>
#include <sdf/SDFImpl.hh>
#include <sdf/Visual.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/common/Skeleton.hh>
#include <ignition/common/SkeletonAnimation.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/msgs/Utility.hh>

#include <ignition/rendering.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/LaserRetro.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LightCmd.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParticleEmitter.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/TemperatureRange.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/Transparency.hh"
#include "ignition/gazebo/components/Visibility.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"
#include "ignition/gazebo/rendering/MarkerManager.hh"

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

  /// \brief Total time elapsed in simulation. This will not increase while
  /// paused.
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Name of rendering engine
  public: std::string engineName = "ogre2";

  /// \brief Name of scene
  public: std::string sceneName = "scene";

  //// \brief True to enable sky in the scene
  public: bool skyEnabled = false;

  /// \brief Scene background color
  public: math::Color backgroundColor = math::Color::Black;

  /// \brief Ambient color
  public: math::Color ambientLight = math::Color(1.0, 1.0, 1.0, 1.0);

  /// \brief Scene manager
  public: SceneManager sceneManager;

  /// \brief Marker manager
  public: MarkerManager markerManager;

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

  /// \brief New actors to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Actor, Entity>> newActors;

  /// \brief New lights to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Light, Entity>> newLights;

  /// \brief A map of entity light ids and light visuals
  public: std::map<Entity, Entity> matchLightWithVisuals;

  /// \brief New sensors to be created. The elements in the tuple are:
  /// [0] entity id, [1], SDF DOM, [2] parent entity id
  public: std::vector<std::tuple<Entity, sdf::Sensor, Entity>>
      newSensors;

  /// \brief New particle emitter to be created. The elements in the tuple are:
  /// [0] entity id, [1], particle emitter, [2] parent entity id
  public: std::vector<std::tuple<Entity, msgs::ParticleEmitter, Entity>>
      newParticleEmitters;

  /// \brief New particle emitter commands to be requested.
  /// The map key and value are: entity id of the particle emitter to
  /// update, and particle emitter msg
  public: std::unordered_map<Entity, msgs::ParticleEmitter>
      newParticleEmittersCmds;

  /// \brief A list of entities with particle emitter cmds to remove
  public: std::vector<Entity> particleCmdsToRemove;

  /// \brief Map of ids of entites to be removed and sim iteration when the
  /// remove request is received
  public: std::unordered_map<Entity, uint64_t> removeEntities;

  /// \brief A map of entity ids and pose updates.
  public: std::unordered_map<Entity, math::Pose3d> entityPoses;

  /// \brief A map of entity ids and light updates.
  public: std::unordered_map<Entity, msgs::Light> entityLights;

  /// \brief A map of entity ids and light updates.
  public: std::vector<Entity> entityLightsCmdToDelete;

  /// \brief A map of entity ids and actor transforms.
  public: std::map<Entity, std::map<std::string, math::Matrix4d>>
                          actorTransforms;

  /// \brief A map of entity ids and temperature data.
  /// The value of this map (tuple) represents either a single (uniform)
  /// temperature, or a heat signature with a min/max temperature. If the string
  /// in the tuple is empty, then this entity has a uniform temperature across
  /// its surface, and this uniform temperature is stored in the first float of
  /// the tuple (the second float and string are unused for uniform temperature
  /// entities). If the string in the tuple is not empty, then the string
  /// represents the entity's heat signature (a path to a heat signature texture
  /// file), and the floats represent the min/max temperatures of the heat
  /// signature, respectively.
  ///
  /// All temperatures are in Kelvin.
  public: std::map<Entity, std::tuple<float, float, std::string>> entityTemp;

  /// \brief A map of entity ids and wire boxes
  public: std::unordered_map<Entity, ignition::rendering::WireBoxPtr> wireBoxes;

  /// \brief A map of entity ids and trajectory pose updates.
  public: std::unordered_map<Entity, math::Pose3d> trajectoryPoses;

  /// \brief A map of entity ids and actor animation info.
  public: std::unordered_map<Entity, AnimationUpdateData> actorAnimationData;

  /// \brief True to update skeletons manually using bone poses
  /// (see actorTransforms). False to let render engine update animation
  /// based on sim time.
  /// \todo(anyone) Let this be turned on from a component
  public: bool actorManualSkeletonUpdate = false;

  /// \brief Mutex to protect updates
  public: std::mutex updateMutex;

  //// \brief Flag to indicate whether to create sensors
  public: bool enableSensors = false;

  /// \brief A set containing all the entities with attached rendering sensors
  public: std::unordered_set<Entity> sensorEntities;

  /// \brief Callback function for creating sensors.
  /// The function args are: entity id, sensor sdf, and parent name.
  /// The function returns the id of the rendering sensor created.
  public: std::function<std::string(const gazebo::Entity &, const sdf::Sensor &,
          const std::string &)> createSensorCb;

  /// \brief Light equality comparison function.
  public: std::function<bool(const sdf::Light &, const sdf::Light &)>
          lightEql { [](const sdf::Light &_a, const sdf::Light &_b)
            {
             return
                _a.Type() == _b.Type() &&
                _a.Name() == _b.Name() &&
                _a.Diffuse() == _b.Diffuse() &&
                _a.Specular() == _b.Specular() &&
                math::equal(
                  _a.AttenuationRange(), _b.AttenuationRange(), 1e-6) &&
               math::equal(
                 _a.LinearAttenuationFactor(),
                 _b.LinearAttenuationFactor(),
                 1e-6) &&
               math::equal(
                 _a.ConstantAttenuationFactor(),
                 _b.ConstantAttenuationFactor(),
                 1e-6) &&
               math::equal(
                 _a.QuadraticAttenuationFactor(),
                 _b.QuadraticAttenuationFactor(),
                 1e-6) &&
               _a.CastShadows() == _b.CastShadows() &&
               _a.Direction() == _b.Direction() &&
               _a.SpotInnerAngle() == _b.SpotInnerAngle() &&
               _a.SpotOuterAngle() == _b.SpotOuterAngle() &&
               math::equal(_a.SpotFalloff(), _b.SpotFalloff(), 1e-6);
            }};

  /// \brief Callback function for removing sensors.
  /// The function arg is the entity id
  public: std::function<void(const gazebo::Entity &)> removeSensorCb;

  /// \brief Currently selected entities, organized by order of selection.
  public: std::vector<Entity> selectedEntities;

  /// \brief Map of original emissive colors for nodes currently highlighted.
  public: std::map<std::string, math::Color> originalEmissive;

  /// \brief Whether the transform gizmo is being dragged.
  public: bool transformActive{false};

  /// \brief Highlight a node and all its children.
  /// \param[in] _node Node to be highlighted
  public: void HighlightNode(const rendering::NodePtr &_node);

  /// \brief Restore a highlighted node to normal.
  /// \param[in] _node Node to be restored.
  public: void LowlightNode(const rendering::NodePtr &_node);

  /// \brief New collisions to be created
  public: std::vector<Entity> newCollisions;

  /// \brief Finds the links (collision parent) that are used to create child
  /// collision visuals in RenderUtil::Update
  /// \param[in] _ecm The entity-component manager
  public: void FindCollisionLinks(const EntityComponentManager &_ecm);

  /// \brief A list of links used to create new collision visuals
  public: std::vector<Entity> newCollisionLinks;

  /// \brief A map of collision entity ids and their SDF DOM
  public: std::map<Entity, sdf::Collision> entityCollisions;

  /// \brief A map of model entities and their corresponding children links
  public: std::map<Entity, std::vector<Entity>> modelToLinkEntities;

  /// \brief A map of link entities and their corresponding children collisions
  public: std::map<Entity, std::vector<Entity>> linkToCollisionEntities;

  /// \brief A map of created collision entities and if they are currently
  /// visible
  public: std::map<Entity, bool> viewingCollisions;

  /// \brief A map of model entities and their corresponding children models
  public: std::map<Entity, std::vector<Entity>> modelToModelEntities;

  /// \brief A map of entity id to thermal camera sensor configuration
  /// properties. The elements in the tuple are:
  /// <resolution, temperature range (min, max)>
  public: std::unordered_map<Entity,
      std::tuple<double, components::TemperatureRangeInfo>> thermalCameraData;

  /// \brief A helper function that removes the sensor associated with an
  /// entity, if an associated sensor exists. This should be called in
  /// RenderUtil::Update.
  /// \param[in] _entity The entity that should be checked for an associated
  /// sensor.
  public: void RemoveSensor(const Entity _entity);

  /// \brief A helper function that removes the bounding box associated with an
  /// entity, if an associated bounding box exists. This should be called in
  /// RenderUtil::Update.
  /// \param[in] _entity The entity that should be checked for an associated
  /// bounding box.
  public: void RemoveBoundingBox(const Entity _entity);

  /// \brief A helper function for updating lights. This should be called in
  /// RenderUtil::Update.
  /// \param[in] _entityLights A map of entity IDs to their light updates.
  public: void UpdateLights(
              const std::unordered_map<Entity, msgs::Light> &_entityLights);

  /// \brief A helper function for updating the thermal camera. This should be
  /// called in RenderUtil::Update.
  /// \param[in] _thermalCamData The thermal camera data that needs to be
  /// updated.
  /// \sa thermalCameraData
  public: void UpdateThermalCamera(const std::unordered_map<Entity,
    std::tuple<double, components::TemperatureRangeInfo>> &_thermalCamData);

  /// \brief Helper function for updating animation. This should be called in
  /// RenderUtil::Update.
  /// \param[in] _actorAnimationData A map of entities to their animation update
  /// data.
  /// \param[in] _entityPoses A map of entity ids and pose updates.
  /// \param[in] _trajectoryPoses A map of entity ids and trajectory
  /// pose updates.
  /// \sa actorManualSkeletonUpdate
  public: void UpdateAnimation(const std::unordered_map<Entity,
              AnimationUpdateData> &_actorAnimationData,
              const std::unordered_map<Entity, math::Pose3d> &_entityPoses,
              const std::unordered_map<Entity, math::Pose3d> &_trajectoryPoses);
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
void RenderUtil::UpdateECM(const UpdateInfo &/*_info*/,
                           EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);

  // Remove the commands from the entity
  // these are commands from the last iteration. We want to make sure all
  // systems have a chance to process them first before they are removed.
  for (const auto &entity : this->dataPtr->particleCmdsToRemove)
    _ecm.RemoveComponent<components::ParticleEmitterCmd>(entity);
  this->dataPtr->particleCmdsToRemove.clear();

  // particle emitters commands
  _ecm.Each<components::ParticleEmitterCmd>(
      [&](const Entity &_entity,
          const components::ParticleEmitterCmd *_emitterCmd) -> bool
      {
        // store emitter properties and update them in rendering thread
        this->dataPtr->newParticleEmittersCmds[_entity] =
        _emitterCmd->Data();

        // update pose comp here
        if (_emitterCmd->Data().has_pose())
        {
          auto poseComp = _ecm.Component<components::Pose>(_entity);
          if (poseComp)
            poseComp->Data() = msgs::Convert(_emitterCmd->Data().pose());
        }
        // Store the entity ids to clear outside of the `Each` loop.
        this->dataPtr->particleCmdsToRemove.push_back(_entity);

        return true;
      });

  // Update lights
  auto olderEntitiesLightsCmdToDelete =
    std::move(this->dataPtr->entityLightsCmdToDelete);
  this->dataPtr->entityLightsCmdToDelete.clear();

  _ecm.Each<components::LightCmd>(
      [&](const Entity &_entity,
          const components::LightCmd * _lightCmd) -> bool
      {
        this->dataPtr->entityLights[_entity] = _lightCmd->Data();
        this->dataPtr->entityLightsCmdToDelete.push_back(_entity);

        auto lightComp = _ecm.Component<components::Light>(_entity);
        if (lightComp)
        {
          sdf::Light sdfLight = convert<sdf::Light>(_lightCmd->Data());
          auto state = lightComp->SetData(sdfLight,
              this->dataPtr->lightEql) ?
              ComponentState::OneTimeChange :
              ComponentState::NoChange;
          _ecm.SetChanged(_entity, components::Light::typeId, state);
        }
        return true;
      });

  for (const auto entity : olderEntitiesLightsCmdToDelete)
  {
    _ecm.RemoveComponent<components::LightCmd>(entity);
  }

  // Update thermal cameras
  _ecm.Each<components::ThermalCamera>(
      [&](const Entity &_entity,
        const components::ThermalCamera *)->bool
      {
        // set properties from thermal sensor plugin
        // Set defaults to invaid values so we know they have not been set.
        // set UpdateECM(). We check for valid values first before setting
        // these thermal camera properties..
        double resolution = 0.0;
        components::TemperatureRangeInfo range;
        range.min = std::numeric_limits<double>::max();
        range.max = 0;

        // resolution
        auto resolutionComp =
          _ecm.Component<components::TemperatureLinearResolution>(_entity);
        if (resolutionComp != nullptr)
        {
          resolution = resolutionComp->Data();
          _ecm.RemoveComponent<components::TemperatureLinearResolution>(
              _entity);
        }

        // min / max temp
        auto tempRangeComp =
          _ecm.Component<components::TemperatureRange>(_entity);
        if (tempRangeComp != nullptr)
        {
          range = tempRangeComp->Data();
          _ecm.RemoveComponent<components::TemperatureRange>(_entity);
        }

        if (resolutionComp || tempRangeComp)
        {
          this->dataPtr->thermalCameraData[_entity] =
              std::make_tuple(resolution, range);
        }
        return true;
      });
}

//////////////////////////////////////////////////
void RenderUtil::UpdateFromECM(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  IGN_PROFILE("RenderUtil::UpdateFromECM");
  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
  this->dataPtr->simTime = _info.simTime;

  this->dataPtr->CreateRenderingEntities(_ecm, _info);
  this->dataPtr->UpdateRenderingEntities(_ecm);
  this->dataPtr->RemoveRenderingEntities(_ecm, _info);
  this->dataPtr->markerManager.SetSimTime(_info.simTime);
  this->dataPtr->FindCollisionLinks(_ecm);
}

//////////////////////////////////////////////////
void RenderUtilPrivate::FindCollisionLinks(const EntityComponentManager &_ecm)
{
  if (this->newCollisions.empty())
    return;

  for (const auto &entity : this->newCollisions)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}))
    {
      std::stack<Entity> modelStack;
      modelStack.push(entity);

      std::vector<Entity> childLinks, childModels;
      while (!modelStack.empty())
      {
        Entity model = modelStack.top();
        modelStack.pop();

        childLinks = _ecm.EntitiesByComponents(components::ParentEntity(model),
                                               components::Link());
        links.insert(links.end(),
                     childLinks.begin(),
                     childLinks.end());

        childModels =
            _ecm.EntitiesByComponents(components::ParentEntity(model),
                                      components::Model());
        for (const auto &childModel : childModels)
        {
            modelStack.push(childModel);
        }
      }
    }
    else if (_ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links.push_back(entity);
    }
    else
    {
      ignerr << "Entity [" << entity
             << "] for viewing collision must be a model or link"
             << std::endl;
      continue;
    }

    this->newCollisionLinks.insert(this->newCollisionLinks.end(),
        links.begin(),
        links.end());
  }
  this->newCollisions.clear();
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
  auto newActors = std::move(this->dataPtr->newActors);
  auto newLights = std::move(this->dataPtr->newLights);
  auto newParticleEmitters = std::move(this->dataPtr->newParticleEmitters);
  auto newParticleEmittersCmds =
    std::move(this->dataPtr->newParticleEmittersCmds);
  auto removeEntities = std::move(this->dataPtr->removeEntities);
  auto entityPoses = std::move(this->dataPtr->entityPoses);
  auto entityLights = std::move(this->dataPtr->entityLights);
  auto trajectoryPoses = std::move(this->dataPtr->trajectoryPoses);
  auto actorTransforms = std::move(this->dataPtr->actorTransforms);
  auto actorAnimationData = std::move(this->dataPtr->actorAnimationData);
  auto entityTemp = std::move(this->dataPtr->entityTemp);
  auto newCollisionLinks = std::move(this->dataPtr->newCollisionLinks);
  auto thermalCameraData = std::move(this->dataPtr->thermalCameraData);

  this->dataPtr->newScenes.clear();
  this->dataPtr->newModels.clear();
  this->dataPtr->newLinks.clear();
  this->dataPtr->newVisuals.clear();
  this->dataPtr->newActors.clear();
  this->dataPtr->newLights.clear();
  this->dataPtr->newParticleEmitters.clear();
  this->dataPtr->newParticleEmittersCmds.clear();
  this->dataPtr->removeEntities.clear();
  this->dataPtr->entityPoses.clear();
  this->dataPtr->entityLights.clear();
  this->dataPtr->trajectoryPoses.clear();
  this->dataPtr->actorTransforms.clear();
  this->dataPtr->actorAnimationData.clear();
  this->dataPtr->entityTemp.clear();
  this->dataPtr->newCollisionLinks.clear();
  this->dataPtr->thermalCameraData.clear();

  this->dataPtr->markerManager.Update();

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
    if (scene.Sky())
    {
      this->dataPtr->scene->SetSkyEnabled(true);
    }

    // only one scene so break
    break;
  }

  // remove existing entities
  {
    IGN_PROFILE("RenderUtil::Update Remove");
    for (auto &entity : removeEntities)
    {
      auto node = this->dataPtr->sceneManager.NodeById(entity.first);
      this->dataPtr->selectedEntities.erase(std::remove(
          this->dataPtr->selectedEntities.begin(),
          this->dataPtr->selectedEntities.end(), entity.first),
          this->dataPtr->selectedEntities.end());
      this->dataPtr->sceneManager.RemoveEntity(entity.first);

      this->dataPtr->RemoveSensor(entity.first);
      this->dataPtr->RemoveBoundingBox(entity.first);
    }
  }

  // create new entities
  {
    IGN_PROFILE("RenderUtil::Update Create");
    for (const auto &model : newModels)
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

    for (const auto &link : newLinks)
    {
      this->dataPtr->sceneManager.CreateLink(
          std::get<0>(link), std::get<1>(link), std::get<2>(link));
    }

    for (const auto &visual : newVisuals)
    {
      this->dataPtr->sceneManager.CreateVisual(
          std::get<0>(visual), std::get<1>(visual), std::get<2>(visual));
    }

    for (const auto &actor : newActors)
    {
      this->dataPtr->sceneManager.CreateActor(
          std::get<0>(actor), std::get<1>(actor), std::get<2>(actor));
    }

    for (const auto &light : newLights)
    {
      this->dataPtr->sceneManager.CreateLight(
          std::get<0>(light), std::get<1>(light), std::get<2>(light));

      // create a new id for the light visual
      auto attempts = 100000u;
      for (auto i = 0u; i < attempts; ++i)
      {
        Entity id = std::numeric_limits<uint64_t>::min() + i;
        if (!this->dataPtr->sceneManager.HasEntity(id))
        {
          rendering::VisualPtr lightVisual =
            this->dataPtr->sceneManager.CreateLightVisual(
              id, std::get<1>(light), std::get<0>(light));
          this->dataPtr->matchLightWithVisuals[std::get<0>(light)] = id;
          break;
        }
      }
    }

    for (const auto &emitter : newParticleEmitters)
    {
      this->dataPtr->sceneManager.CreateParticleEmitter(
          std::get<0>(emitter), std::get<1>(emitter), std::get<2>(emitter));
    }

    for (const auto &emitterCmd : newParticleEmittersCmds)
    {
      this->dataPtr->sceneManager.UpdateParticleEmitter(
          emitterCmd.first, emitterCmd.second);
    }

    if (this->dataPtr->enableSensors && this->dataPtr->createSensorCb)
    {
      for (const auto &sensor : newSensors)
      {
        Entity entity = std::get<0>(sensor);
        const sdf::Sensor &dataSdf = std::get<1>(sensor);
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
            this->dataPtr->createSensorCb(entity, dataSdf, parentNode->Name());
        // Add to the system's scene manager
        if (!this->dataPtr->sceneManager.AddSensor(entity, sensorName, parent))
        {
          ignerr << "Failed to create sensor [" << sensorName << "]"
                 << std::endl;
        }
      }
    }
  }

  this->dataPtr->UpdateLights(entityLights);

  // update entities' pose
  {
    IGN_PROFILE("RenderUtil::Update Poses");
    for (const auto &pose : entityPoses)
    {
      auto node = this->dataPtr->sceneManager.NodeById(pose.first);
      if (!node)
        continue;

      // Don't move entity being manipulated (last selected)
      // TODO(anyone) Check top level visual instead of parent
      auto vis = std::dynamic_pointer_cast<rendering::Visual>(node);
      int updateNode = 0;
      Entity entityId = kNullEntity;
      if (vis)
      {
        // Get information from the visual's user data to indicate if
        // the render thread should pause updating it's true location,
        // this functionality is needed for temporal placement of a
        // visual such as an align preview
        updateNode = std::get<int>(vis->UserData("pause-update"));
        entityId = std::get<int>(vis->UserData("gazebo-entity"));
      }
      if ((this->dataPtr->transformActive &&
          (pose.first == this->dataPtr->selectedEntities.back() ||
          entityId == this->dataPtr->selectedEntities.back())) ||
          updateNode)
      {
        continue;
      }

      node->SetLocalPose(pose.second);
    }

    // update entities' local transformations
    if (this->dataPtr->actorManualSkeletonUpdate)
    {
      for (auto &tf : actorTransforms)
      {
        auto actorMesh = this->dataPtr->sceneManager.ActorMeshById(tf.first);
        auto actorVisual = this->dataPtr->sceneManager.NodeById(tf.first);
        if (!actorMesh || !actorVisual)
        {
          ignerr << "Actor with Entity ID '" << tf.first << "'. not found. "
                 << "Skipping skeleton animation update." << std::endl;
          continue;
        }

        math::Pose3d globalPose;
        if (entityPoses.find(tf.first) != entityPoses.end())
        {
          globalPose = entityPoses[tf.first];
        }

        math::Pose3d trajPose;
        // Trajectory from the ECS
        if (trajectoryPoses.find(tf.first) != trajectoryPoses.end())
        {
          trajPose = trajectoryPoses[tf.first];
        }
        // Trajectory from the SDF script
        else
        {
          trajPose.Pos() = tf.second["actorPose"].Translation();
          trajPose.Rot() = tf.second["actorPose"].Rotation();
        }

        actorVisual->SetLocalPose(trajPose + globalPose);

        tf.second.erase("actorPose");
        actorMesh->SetSkeletonLocalTransforms(tf.second);
      }
    }
    else
    {
      this->dataPtr->UpdateAnimation(actorAnimationData, entityPoses,
          trajectoryPoses);
    }
  }

  // set visual temperature
  for (const auto &temp : entityTemp)
  {
    auto node = this->dataPtr->sceneManager.NodeById(temp.first);
    if (!node)
      continue;

    auto visual =
        std::dynamic_pointer_cast<rendering::Visual>(node);
    if (!visual)
      continue;

    const auto &heatSignature = std::get<2>(temp.second);
    if (heatSignature.empty())
      visual->SetUserData("temperature", std::get<0>(temp.second));
    else
    {
      visual->SetUserData("minTemp", std::get<0>(temp.second));
      visual->SetUserData("maxTemp", std::get<1>(temp.second));
      visual->SetUserData("temperature", heatSignature);
    }
  }

  // create new collision visuals
  {
    for (const auto &link : newCollisionLinks)
    {
      std::vector<Entity> colEntities =
          this->dataPtr->linkToCollisionEntities[link];

      for (const auto &colEntity : colEntities)
      {
        if (!this->dataPtr->sceneManager.HasEntity(colEntity))
        {
          auto vis = this->dataPtr->sceneManager.CreateCollision(colEntity,
              this->dataPtr->entityCollisions[colEntity], link);
          this->dataPtr->viewingCollisions[colEntity] = true;

          // add geometry material to originalEmissive map
          for (auto g = 0u; g < vis->GeometryCount(); ++g)
          {
            auto geom = vis->GeometryByIndex(g);

            // Geometry material
            auto geomMat = geom->Material();
            if (nullptr == geomMat)
              continue;

            if (this->dataPtr->originalEmissive.find(geom->Name()) ==
                this->dataPtr->originalEmissive.end())
            {
              this->dataPtr->originalEmissive[geom->Name()] =
                  geomMat->Emissive();
            }
          }
        }
      }
    }
  }

  this->dataPtr->UpdateThermalCamera(thermalCameraData);
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
    this->sensorEntities.insert(_entity);
  };

  const std::string cameraSuffix{"/image"};
  const std::string depthCameraSuffix{"/depth_image"};
  const std::string rgbdCameraSuffix{""};
  const std::string thermalCameraSuffix{"/image"};
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
          model.SetRawPose(_pose->Data());
          this->newModels.push_back(
              std::make_tuple(_entity, model, _parent->Data(),
              _info.iterations));
          this->modelToModelEntities[_parent->Data()].push_back(_entity);
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
          link.SetRawPose(_pose->Data());
          this->newLinks.push_back(
              std::make_tuple(_entity, link, _parent->Data()));
          // used for collsions
          this->modelToLinkEntities[_parent->Data()].push_back(_entity);
          return true;
        });

    // visuals
    _ecm.Each<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::Transparency,
              components::VisibilityFlags,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Geometry *_geom,
            const components::CastShadows *_castShadows,
            const components::Transparency *_transparency,
            const components::VisibilityFlags *_visibilityFlags,
            const components::ParentEntity *_parent)->bool
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetRawPose(_pose->Data());
          visual.SetGeom(_geom->Data());
          visual.SetCastShadows(_castShadows->Data());
          visual.SetTransparency(_transparency->Data());
          visual.SetVisibilityFlags(_visibilityFlags->Data());

          // Optional components
          auto material = _ecm.Component<components::Material>(_entity);
          if (material != nullptr)
          {
            visual.SetMaterial(material->Data());
          }

          auto laserRetro = _ecm.Component<components::LaserRetro>(_entity);
          if (laserRetro != nullptr)
          {
            visual.SetLaserRetro(laserRetro->Data());
          }

          if (auto temp = _ecm.Component<components::Temperature>(_entity))
          {
            // get the uniform temperature for the entity
            this->entityTemp[_entity] =
              std::make_tuple<float, float, std::string>(
                  temp->Data().Kelvin(), 0.0, "");
          }
          else
          {
            // entity doesn't have a uniform temperature. Check if it has
            // a heat signature with an associated temperature range
            auto heatSignature =
              _ecm.Component<components::SourceFilePath>(_entity);
            auto tempRange =
               _ecm.Component<components::TemperatureRange>(_entity);
            if (heatSignature && tempRange)
            {
              this->entityTemp[_entity] =
                std::make_tuple<float, float, std::string>(
                    tempRange->Data().min.Kelvin(),
                    tempRange->Data().max.Kelvin(),
                    std::string(heatSignature->Data()));
            }
          }

          this->newVisuals.push_back(
              std::make_tuple(_entity, visual, _parent->Data()));
          return true;
        });

    // actors
    _ecm.Each<components::Actor, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Actor *_actor,
            const components::ParentEntity *_parent) -> bool
        {
          this->newActors.push_back(
              std::make_tuple(_entity, _actor->Data(), _parent->Data()));
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

    // collisions
    _ecm.Each<components::Collision, components::Name, components::Pose,
              components::Geometry, components::CollisionElement,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Collision *,
            const components::Name *,
            const components::Pose *,
            const components::Geometry *,
            const components::CollisionElement *_collElement,
            const components::ParentEntity *_parent) -> bool
        {
          this->entityCollisions[_entity] = _collElement->Data();
          this->linkToCollisionEntities[_parent->Data()].push_back(_entity);
          return true;
        });

    // particle emitters
    _ecm.Each<components::ParticleEmitter, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::ParticleEmitter *_emitter,
            const components::ParentEntity *_parent) -> bool
        {
          this->newParticleEmitters.push_back(
              std::make_tuple(_entity, _emitter->Data(), _parent->Data()));
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

      // Create thermal camera
      _ecm.Each<components::ThermalCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::ThermalCamera *_thermalCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _thermalCamera->Data(), _parent->Data(),
                         thermalCameraSuffix);
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
          model.SetRawPose(_pose->Data());
          this->newModels.push_back(
              std::make_tuple(_entity, model, _parent->Data(),
              _info.iterations));
          this->modelToModelEntities[_parent->Data()].push_back(_entity);
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
          link.SetRawPose(_pose->Data());
          this->newLinks.push_back(
              std::make_tuple(_entity, link, _parent->Data()));
          // used for collsions
          this->modelToLinkEntities[_parent->Data()].push_back(_entity);
          return true;
        });

    // visuals
    _ecm.EachNew<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::Transparency,
              components::VisibilityFlags,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Geometry *_geom,
            const components::CastShadows *_castShadows,
            const components::Transparency *_transparency,
            const components::VisibilityFlags *_visibilityFlags,
            const components::ParentEntity *_parent)->bool
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetRawPose(_pose->Data());
          visual.SetGeom(_geom->Data());
          visual.SetCastShadows(_castShadows->Data());
          visual.SetTransparency(_transparency->Data());
          visual.SetVisibilityFlags(_visibilityFlags->Data());

          // Optional components
          auto material = _ecm.Component<components::Material>(_entity);
          if (material != nullptr)
          {
            visual.SetMaterial(material->Data());
          }

          auto laserRetro = _ecm.Component<components::LaserRetro>(_entity);
          if (laserRetro != nullptr)
          {
            visual.SetLaserRetro(laserRetro->Data());
          }

          if (auto temp = _ecm.Component<components::Temperature>(_entity))
          {
            // get the uniform temperature for the entity
            this->entityTemp[_entity] =
              std::make_tuple<float, float, std::string>(
                  temp->Data().Kelvin(), 0.0, "");
          }
          else
          {
            // entity doesn't have a uniform temperature. Check if it has
            // a heat signature with an associated temperature range
            auto heatSignature =
              _ecm.Component<components::SourceFilePath>(_entity);
            auto tempRange =
               _ecm.Component<components::TemperatureRange>(_entity);
            if (heatSignature && tempRange)
            {
              this->entityTemp[_entity] =
                std::make_tuple<float, float, std::string>(
                    tempRange->Data().min.Kelvin(),
                    tempRange->Data().max.Kelvin(),
                    std::string(heatSignature->Data()));
            }
          }

          this->newVisuals.push_back(
              std::make_tuple(_entity, visual, _parent->Data()));
          return true;
        });

    // actors
    _ecm.EachNew<components::Actor, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Actor *_actor,
            const components::ParentEntity *_parent) -> bool
        {
          this->newActors.push_back(
              std::make_tuple(_entity, _actor->Data(), _parent->Data()));
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

    // collisions
    _ecm.EachNew<components::Collision, components::Name, components::Pose,
              components::Geometry, components::CollisionElement,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Collision *,
            const components::Name *,
            const components::Pose *,
            const components::Geometry *,
            const components::CollisionElement *_collElement,
            const components::ParentEntity *_parent) -> bool
        {
          this->entityCollisions[_entity] = _collElement->Data();
          this->linkToCollisionEntities[_parent->Data()].push_back(_entity);
          return true;
        });

    // particle emitters
    _ecm.EachNew<components::ParticleEmitter, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::ParticleEmitter *_emitter,
            const components::ParentEntity *_parent) -> bool
        {
          this->newParticleEmitters.push_back(
              std::make_tuple(_entity, _emitter->Data(), _parent->Data()));
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

      // Create thermal camera
      _ecm.EachNew<components::ThermalCamera, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::ThermalCamera *_thermalCamera,
            const components::ParentEntity *_parent)->bool
          {
            addNewSensor(_entity, _thermalCamera->Data(), _parent->Data(),
                         thermalCameraSuffix);
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

  // actors
  _ecm.Each<components::Actor, components::Pose>(
      [&](const Entity &_entity,
        const components::Actor *,
        const components::Pose *_pose)->bool
      {
        // Trajectory origin
        this->entityPoses[_entity] = _pose->Data();

        auto animTimeComp = _ecm.Component<components::AnimationTime>(_entity);
        auto animNameComp = _ecm.Component<components::AnimationName>(_entity);

        // Animation time set through ECM so ign-rendering can calculate bone
        // transforms
        if (animTimeComp && animNameComp)
        {
          auto skel = this->sceneManager.ActorSkeletonById(_entity);
          if (nullptr != skel)
          {
            AnimationUpdateData animData;
            animData.loop = true;
            animData.followTrajectory = true;
            animData.animationName = animNameComp->Data();
            animData.time = animTimeComp->Data();
            animData.rootTransform = skel->RootNode()->Transform();
            animData.valid = true;
            this->actorAnimationData[_entity] = animData;
          }
        }
        // Bone poses calculated by ign-common
        else if (this->actorManualSkeletonUpdate)
        {
          this->actorTransforms[_entity] =
              this->sceneManager.ActorSkeletonTransformsAt(
              _entity, this->simTime);
        }
        // Trajectory info from SDF so ign-rendering can calculate bone poses
        else
        {
          auto animData =
            this->sceneManager.ActorAnimationAt(_entity, this->simTime);

          if (animData.valid)
          {
            this->actorAnimationData[_entity] = animData;
          }
        }

        // Trajectory pose set by other systems
        auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
        if (trajPoseComp)
          this->trajectoryPoses[_entity] = trajPoseComp->Data();
        return true;
      });

  // update lights
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

  // Update thermal cameras
  _ecm.Each<components::ThermalCamera, components::Pose>(
      [&](const Entity &_entity,
        const components::ThermalCamera *,
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
        this->modelToLinkEntities.erase(_entity);
        this->modelToModelEntities.erase(_entity);
        return true;
      });

  _ecm.EachRemoved<components::Link>(
      [&](const Entity &_entity, const components::Link *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        this->linkToCollisionEntities.erase(_entity);
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
        this->removeEntities[matchLightWithVisuals[_entity]] =
          _info.iterations;
        matchLightWithVisuals.erase(_entity);
        return true;
      });

  // particle emitters
  _ecm.EachRemoved<components::ParticleEmitter>(
      [&](const Entity &_entity, const components::ParticleEmitter *)->bool
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

  // thermal cameras
  _ecm.EachRemoved<components::ThermalCamera>(
    [&](const Entity &_entity, const components::ThermalCamera *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        return true;
      });

  // collisions
  _ecm.EachRemoved<components::Collision>(
    [&](const Entity &_entity, const components::Collision *)->bool
      {
        this->removeEntities[_entity] = _info.iterations;
        this->viewingCollisions.erase(_entity);
        this->entityCollisions.erase(_entity);
        return true;
      });
}

/////////////////////////////////////////////////
void RenderUtil::Init()
{
  ignition::common::SystemPaths pluginPath;
  pluginPath.SetPluginPathEnv(kRenderPluginPathEnv);
  rendering::setPluginPaths(pluginPath.PluginPaths());

  std::map<std::string, std::string> params;
  if (this->dataPtr->useCurrentGLContext)
    params["useCurrentGLContext"] = "1";
  this->dataPtr->engine = rendering::engine(this->dataPtr->engineName, params);
  if (!this->dataPtr->engine)
  {
    ignerr << "Engine [" << this->dataPtr->engineName << "] is not supported. "
           << "Loading OGRE2 instead." << std::endl;
    this->dataPtr->engine = rendering::engine("ogre2", params);
  }

  // Scene
  this->dataPtr->scene =
      this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  if (!this->dataPtr->scene)
  {
    igndbg << "Create scene [" << this->dataPtr->sceneName << "]" << std::endl;
    this->dataPtr->scene =
        this->dataPtr->engine->CreateScene(this->dataPtr->sceneName);
    if (this->dataPtr->scene)
    {
      this->dataPtr->scene->SetAmbientLight(this->dataPtr->ambientLight);
      this->dataPtr->scene->SetBackgroundColor(this->dataPtr->backgroundColor);
      this->dataPtr->scene->SetSkyEnabled(this->dataPtr->skyEnabled);
    }
  }
  this->dataPtr->sceneManager.SetScene(this->dataPtr->scene);
  if (this->dataPtr->enableSensors)
    this->dataPtr->markerManager.SetTopic("/sensors/marker");
  this->dataPtr->markerManager.Init(this->dataPtr->scene);
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
  if (!this->dataPtr->scene)
    return;

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
void RenderUtil::SetSkyEnabled(bool _enabled)
{
  this->dataPtr->skyEnabled = _enabled;
}

/////////////////////////////////////////////////
void RenderUtil::SetUseCurrentGLContext(bool _enable)
{
  this->dataPtr->useCurrentGLContext = _enable;
}

/////////////////////////////////////////////////
void RenderUtil::SetEnableSensors(bool _enable,
    std::function<std::string(const gazebo::Entity &, const sdf::Sensor &,
      const std::string &)> _createSensorCb)
{
  this->dataPtr->enableSensors = _enable;
  this->dataPtr->createSensorCb = std::move(_createSensorCb);
}

/////////////////////////////////////////////////
void RenderUtil::SetRemoveSensorCb(
    std::function<void(const gazebo::Entity &)> _removeSensorCb)
{
  this->dataPtr->removeSensorCb = std::move(_removeSensorCb);
}

/////////////////////////////////////////////////
SceneManager &RenderUtil::SceneManager()
{
  return this->dataPtr->sceneManager;
}

/////////////////////////////////////////////////
MarkerManager &RenderUtil::MarkerManager()
{
  return this->dataPtr->markerManager;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration RenderUtil::SimTime() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
  return this->dataPtr->simTime;
}

/////////////////////////////////////////////////
void RenderUtil::SetSelectedEntity(const rendering::NodePtr &_node)
{
  if (!_node)
    return;

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  Entity entityId = kNullEntity;

  if (vis)
    entityId = std::get<int>(vis->UserData("gazebo-entity"));

  if (entityId == kNullEntity)
    return;

  this->dataPtr->selectedEntities.push_back(entityId);
  this->dataPtr->HighlightNode(_node);
}

/////////////////////////////////////////////////
void RenderUtil::DeselectAllEntities()
{
  for (const auto &entity : this->dataPtr->selectedEntities)
  {
    auto node = this->dataPtr->sceneManager.NodeById(entity);
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
      this->dataPtr->selectedEntities.back());
  return node;
}

/////////////////////////////////////////////////
const std::vector<Entity> &RenderUtil::SelectedEntities() const
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
  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  Entity entityId = kNullEntity;
  if (vis)
    entityId = std::get<int>(vis->UserData("gazebo-entity"));
  // If the entity is not found in the existing map, create a wire box
  auto wireBoxIt = this->wireBoxes.find(entityId);
  if (wireBoxIt == this->wireBoxes.end())
  {
    auto white = this->scene->Material("highlight_material");
    if (!white)
    {
      white = this->scene->CreateMaterial("highlight_material");
      white->SetAmbient(1.0, 1.0, 1.0);
      white->SetDiffuse(1.0, 1.0, 1.0);
      white->SetSpecular(1.0, 1.0, 1.0);
      white->SetEmissive(1.0, 1.0, 1.0);
    }

    auto aabb = vis->LocalBoundingBox();
    if (aabb == math::AxisAlignedBox())
    {
      // Infinite bounding box, skip highlighting this node.
      // This happens for Heightmaps, for example.
      return;
    }

    auto wireBox = this->scene->CreateWireBox();
    wireBox->SetBox(aabb);

    // Create visual and add wire box
    ignition::rendering::VisualPtr wireBoxVis =
      this->scene->CreateVisual();
    wireBoxVis->SetInheritScale(false);
    wireBoxVis->AddGeometry(wireBox);
    wireBoxVis->SetMaterial(white, false);
    vis->AddChild(wireBoxVis);

    // Add wire box to map for setting visibility
    this->wireBoxes.insert(
        std::pair<Entity, ignition::rendering::WireBoxPtr>(entityId, wireBox));
  }
  else
  {
    ignition::rendering::WireBoxPtr wireBox = wireBoxIt->second;
    ignition::math::AxisAlignedBox aabb = vis->LocalBoundingBox();
    wireBox->SetBox(aabb);
    auto visParent = wireBox->Parent();
    if (visParent)
      visParent->SetVisible(true);
  }
}

////////////////////////////////////////////////
void RenderUtilPrivate::LowlightNode(const rendering::NodePtr &_node)
{
  if (!_node)
    return;
  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  Entity entityId = kNullEntity;
  if (vis)
    entityId = std::get<int>(vis->UserData("gazebo-entity"));
  if (this->wireBoxes.find(entityId) != this->wireBoxes.end())
  {
    ignition::rendering::WireBoxPtr wireBox =
      this->wireBoxes[entityId];
    auto visParent = wireBox->Parent();
    if (visParent)
      visParent->SetVisible(false);
  }
}

/////////////////////////////////////////////////
void RenderUtilPrivate::RemoveSensor(const Entity _entity)
{
  auto sensorEntityIt = this->sensorEntities.find(_entity);
  if (sensorEntityIt != this->sensorEntities.end())
  {
    if (this->removeSensorCb)
      this->removeSensorCb(_entity);
    this->sensorEntities.erase(sensorEntityIt);
  }
}

/////////////////////////////////////////////////
void RenderUtilPrivate::RemoveBoundingBox(const Entity _entity)
{
  auto wireBoxIt = this->wireBoxes.find(_entity);
  if (wireBoxIt != this->wireBoxes.end())
  {
    this->scene->DestroyVisual(wireBoxIt->second->Parent());
    this->wireBoxes.erase(wireBoxIt);
  }
}

/////////////////////////////////////////////////
void RenderUtilPrivate::UpdateLights(
    const std::unordered_map<Entity, msgs::Light> &_entityLights)
{
  IGN_PROFILE("RenderUtil::Update Lights");
  for (const auto &light : _entityLights)
  {
    auto node = this->sceneManager.NodeById(light.first);
    if (!node)
      continue;
    auto l = std::dynamic_pointer_cast<rendering::Light>(node);
    if (l)
    {
      if (light.second.has_diffuse())
      {
        if (l->DiffuseColor() != msgs::Convert(light.second.diffuse()))
          l->SetDiffuseColor(msgs::Convert(light.second.diffuse()));
      }
      if (light.second.has_specular())
      {
        if (l->SpecularColor() != msgs::Convert(light.second.specular()))
        {
          l->SetSpecularColor(msgs::Convert(light.second.specular()));
        }
      }
      if (!ignition::math::equal(
          l->AttenuationRange(),
          static_cast<double>(light.second.range())))
      {
        l->SetAttenuationRange(light.second.range());
      }
      if (!ignition::math::equal(
          l->AttenuationLinear(),
          static_cast<double>(light.second.attenuation_linear())))
      {
        l->SetAttenuationLinear(light.second.attenuation_linear());
      }
      if (!ignition::math::equal(
          l->AttenuationConstant(),
          static_cast<double>(light.second.attenuation_constant())))
      {
        l->SetAttenuationConstant(light.second.attenuation_constant());
      }
      if (!ignition::math::equal(
          l->AttenuationQuadratic(),
          static_cast<double>(light.second.attenuation_quadratic())))
      {
        l->SetAttenuationQuadratic(light.second.attenuation_quadratic());
      }
      if (l->CastShadows() != light.second.cast_shadows())
        l->SetCastShadows(light.second.cast_shadows());
      auto lDirectional =
        std::dynamic_pointer_cast<rendering::DirectionalLight>(node);
      if (lDirectional)
      {
        if (light.second.has_direction())
        {
          if (lDirectional->Direction() !=
              msgs::Convert(light.second.direction()))
          {
            lDirectional->SetDirection(
              msgs::Convert(light.second.direction()));
          }
        }
      }
      auto lSpotLight =
        std::dynamic_pointer_cast<rendering::SpotLight>(node);
      if (lSpotLight)
      {
        if (light.second.has_direction())
        {
          if (lSpotLight->Direction() !=
            msgs::Convert(light.second.direction()))
          {
            lSpotLight->SetDirection(
              msgs::Convert(light.second.direction()));
          }
        }
        if (lSpotLight->InnerAngle() != light.second.spot_inner_angle())
          lSpotLight->SetInnerAngle(light.second.spot_inner_angle());
        if (lSpotLight->OuterAngle() != light.second.spot_outer_angle())
          lSpotLight->SetOuterAngle(light.second.spot_outer_angle());
        if (!ignition::math::equal(
            lSpotLight->Falloff(),
            static_cast<double>(light.second.spot_falloff())))
        {
          lSpotLight->SetFalloff(light.second.spot_falloff());
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void RenderUtilPrivate::UpdateThermalCamera(const std::unordered_map<Entity,
    std::tuple<double, components::TemperatureRangeInfo>> &_thermalCamData)
{
  for (const auto &thermal : _thermalCamData)
  {
    Entity id = thermal.first;
    rendering::ThermalCameraPtr camera =
        std::dynamic_pointer_cast<rendering::ThermalCamera>(
        this->sceneManager.NodeById(id));
    if (camera)
    {
      double resolution = std::get<0>(thermal.second);

      if (resolution > 0.0)
      {
        camera->SetLinearResolution(resolution);
      }
      else
      {
        ignwarn << "Unable to set thermal camera temperature linear resolution."
                << " Value must be greater than 0. Using the default value: "
                << camera->LinearResolution() << ". " << std::endl;
      }
      double minTemp = std::get<1>(thermal.second).min.Kelvin();
      double maxTemp = std::get<1>(thermal.second).max.Kelvin();
      if (maxTemp >= minTemp)
      {
        camera->SetMinTemperature(minTemp);
        camera->SetMaxTemperature(maxTemp);
      }
      else
      {
        ignwarn << "Unable to set thermal camera temperature range."
                << "Max temperature must be greater or equal to min. "
                << "Using the default values : [" << camera->MinTemperature()
                << ", " << camera->MaxTemperature() << "]." << std::endl;
      }
    }
  }
}

/////////////////////////////////////////////////
void RenderUtilPrivate::UpdateAnimation(const std::unordered_map<Entity,
    AnimationUpdateData> &_actorAnimationData,
    const std::unordered_map<Entity, math::Pose3d> &_entityPoses,
    const std::unordered_map<Entity, math::Pose3d> &_trajectoryPoses)
{
  for (auto &it : _actorAnimationData)
  {
    auto actorMesh = this->sceneManager.ActorMeshById(it.first);
    auto actorVisual = this->sceneManager.NodeById(it.first);
    auto actorSkel = this->sceneManager.ActorSkeletonById(
        it.first);
    if (!actorMesh || !actorVisual || !actorSkel)
    {
      ignerr << "Actor with Entity ID '" << it.first << "'. not found. "
             << "Skipping skeleton animation update." << std::endl;
      continue;
    }

    const AnimationUpdateData &animData = it.second;
    if (!animData.valid)
    {
      ignerr << "invalid animation update data" << std::endl;
      continue;
    }
    // Enable skeleton animation
    if (!actorMesh->SkeletonAnimationEnabled(animData.animationName))
    {
      // disable all animations for this actor
      for (unsigned int i = 0; i < actorSkel->AnimationCount(); ++i)
      {
        actorMesh->SetSkeletonAnimationEnabled(
            actorSkel->Animation(i)->Name(), false, false, 0.0);
      }

      // enable requested animation
      actorMesh->SetSkeletonAnimationEnabled(
          animData.animationName, true, animData.loop);

      // Set skeleton root node weight to zero so it is not affected by
      // the animation being played. This is needed if trajectory animation
      // is enabled. We need to let the trajectory animation set the
      // position of the actor instead
      common::SkeletonPtr skeleton =
          this->sceneManager.ActorSkeletonById(it.first);
      if (skeleton)
      {
        float rootBoneWeight = (animData.followTrajectory) ? 0.0 : 1.0;
        std::unordered_map<std::string, float> weights;
        weights[skeleton->RootNode()->Name()] = rootBoneWeight;
        actorMesh->SetSkeletonWeights(weights);
      }
    }
    // Update skeleton animation by setting animation time.
    // Note that animation time is different from sim time. An actor can
    // have multiple animations. Animation time is associated with
    // current animation that is being played. It is also adjusted if
    // interpotate_x is enabled.
    actorMesh->UpdateSkeletonAnimation(animData.time);

    // manually update root transform in order to sync with trajectory
    // animation
    if (animData.followTrajectory)
    {
      common::SkeletonPtr skeleton =
          this->sceneManager.ActorSkeletonById(it.first);
      std::map<std::string, math::Matrix4d> rootTf;
      rootTf[skeleton->RootNode()->Name()] = animData.rootTransform;
      actorMesh->SetSkeletonLocalTransforms(rootTf);
    }

    // update actor trajectory animation
    math::Pose3d globalPose;
    auto entityPosesIt = _entityPoses.find(it.first);
    if (entityPosesIt != _entityPoses.end())
    {
      globalPose = entityPosesIt->second;
    }

    math::Pose3d trajPose;
    // Trajectory from the ECS
    auto trajectoryPosesIt = _trajectoryPoses.find(it.first);
    if (trajectoryPosesIt != _trajectoryPoses.end())
    {
      trajPose = trajectoryPosesIt->second;
    }
    else
    {
      // trajectory from sdf script
      common::PoseKeyFrame poseFrame(0.0);
      if (animData.followTrajectory)
        animData.trajectory.Waypoints()->InterpolatedKeyFrame(poseFrame);
      trajPose.Pos() = poseFrame.Translation();
      trajPose.Rot() = poseFrame.Rotation();
    }

    actorVisual->SetLocalPose(trajPose + globalPose);
  }
}

/////////////////////////////////////////////////
void RenderUtil::ViewCollisions(const Entity &_entity)
{
  std::vector<Entity> colEntities;
  std::vector<Entity> links;

  if (this->dataPtr->linkToCollisionEntities.find(_entity) !=
      this->dataPtr->linkToCollisionEntities.end())
  {
    colEntities = this->dataPtr->linkToCollisionEntities[_entity];
  }
  else if (this->dataPtr->modelToLinkEntities.find(_entity) !=
           this->dataPtr->modelToLinkEntities.end())
  {
    links.insert(links.end(),
        this->dataPtr->modelToLinkEntities[_entity].begin(),
        this->dataPtr->modelToLinkEntities[_entity].end());
  }

  if (this->dataPtr->modelToModelEntities.find(_entity) !=
      this->dataPtr->modelToModelEntities.end())
  {
    std::stack<Entity> modelStack;
    modelStack.push(_entity);

    std::vector<Entity> childModels;
    while (!modelStack.empty())
    {
      Entity model = modelStack.top();
      modelStack.pop();

      links.insert(links.end(),
          this->dataPtr->modelToLinkEntities[model].begin(),
          this->dataPtr->modelToLinkEntities[model].end());

      childModels = this->dataPtr->modelToModelEntities[model];
      for (const auto &childModel : childModels)
      {
        modelStack.push(childModel);
      }
    }
  }

  for (const auto &link : links)
    colEntities.insert(colEntities.end(),
        this->dataPtr->linkToCollisionEntities[link].begin(),
        this->dataPtr->linkToCollisionEntities[link].end());

  // create and/or toggle collision visuals

  bool showCol, showColInit = false;
  // first loop looks for new collisions
  for (const auto &colEntity : colEntities)
  {
    if (this->dataPtr->viewingCollisions.find(colEntity) ==
        this->dataPtr->viewingCollisions.end())
    {
      this->dataPtr->newCollisions.push_back(_entity);
      showColInit = showCol = true;
    }
  }

  // second loop toggles already created collisions
  for (const auto &colEntity : colEntities)
  {
    if (this->dataPtr->viewingCollisions.find(colEntity) ==
        this->dataPtr->viewingCollisions.end())
      continue;

    // when viewing multiple collisions (e.g. _entity is a model),
    // boolean for view collisions is based on first colEntity in list
    if (!showColInit)
    {
      showCol = !this->dataPtr->viewingCollisions[colEntity];
      showColInit = true;
    }

    rendering::VisualPtr colVisual =
        this->dataPtr->sceneManager.VisualById(colEntity);
    if (colVisual == nullptr)
    {
      ignerr << "Could not find collision visual for entity [" << colEntity
             << "]" << std::endl;
      continue;
    }

    this->dataPtr->viewingCollisions[colEntity] = showCol;
    colVisual->SetVisible(showCol);

    if (showCol)
    {
      // turn off wireboxes for collision entity
      if (this->dataPtr->wireBoxes.find(colEntity)
            != this->dataPtr->wireBoxes.end())
      {
        ignition::rendering::WireBoxPtr wireBox =
          this->dataPtr->wireBoxes[colEntity];
        auto visParent = wireBox->Parent();
        if (visParent)
          visParent->SetVisible(false);
      }
    }
  }
}
