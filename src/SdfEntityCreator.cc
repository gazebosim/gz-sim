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

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <sdf/Types.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/AirPressureSensor.hh"
#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Atmosphere.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/LaserRetro.hh"
#include "ignition/gazebo/components/Lidar.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LightType.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/LogicalCamera.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include <ignition/gazebo/components/ParticleEmitter.hh>
#include "ignition/gazebo/components/Physics.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/SelfCollide.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Transparency.hh"
#include "ignition/gazebo/components/Visibility.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"

class ignition::gazebo::SdfEntityCreatorPrivate
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Pointer to event manager. We don't assume ownership.
  public: EventManager *eventManager{nullptr};

  /// \brief Keep track of new sensors being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::ElementPtr> newSensors;

  /// \brief Keep track of new models being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::ElementPtr> newModels;

  /// \brief Keep track of new visuals being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::ElementPtr> newVisuals;
};

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static math::Pose3d ResolveSdfPose(const sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    pose = _semPose.RawPose();
  }

  return pose;
}

/////////////////////////////////////////////////
static std::optional<sdf::JointAxis> ResolveJointAxis(
    const sdf::JointAxis &_unresolvedAxis)
{
  math::Vector3d axisXyz;
  const sdf::Errors resolveAxisErrors = _unresolvedAxis.ResolveXyz(axisXyz);
  if (!resolveAxisErrors.empty())
  {
    ignerr << "Failed to resolve axis" << std::endl;
    return std::nullopt;
  }

  sdf::JointAxis resolvedAxis = _unresolvedAxis;

  const sdf::Errors setXyzErrors = resolvedAxis.SetXyz(axisXyz);
  if (!setXyzErrors.empty())
  {
    ignerr << "Failed to resolve axis" << std::endl;
    return std::nullopt;
  }

  resolvedAxis.SetXyzExpressedIn("");
  return resolvedAxis;
}

//////////////////////////////////////////////////
/// \brief Find a descendent child link entity by name.
/// \param[in] _name The relative name of the link with "::" as the scope
/// delimiter
/// \param[in] _model Model entity that defines the scope
/// \param[in] _ecm Entity component manager
/// \return The Entity of the descendent link or kNullEntity if link was not
/// found
static Entity FindDescendentLinkEntityByName(const std::string &_name,
                                             const Entity &_model,
                                             const EntityComponentManager &_ecm)
{
  auto ind = _name.find(sdf::kSdfScopeDelimiter);
  std::vector<Entity> candidates;
  if (ind != std::string::npos)
  {
    candidates = _ecm.ChildrenByComponents(
        _model, components::Model(), components::Name(_name.substr(0, ind)));
    if (candidates.size() != 1 || (ind + 2 >= _name.size()))
    {
      return kNullEntity;
    }
    return FindDescendentLinkEntityByName(_name.substr(ind + 2),
                                          candidates.front(), _ecm);
  }
  else
  {
    candidates = _ecm.ChildrenByComponents(_model, components::Link(),
                                           components::Name(_name));

    if (candidates.size() != 1)
    {
      return kNullEntity;
    }
    return candidates.front();
  }
}

//////////////////////////////////////////////////
SdfEntityCreator::SdfEntityCreator(EntityComponentManager &_ecm,
          EventManager &_eventManager)
  : dataPtr(std::make_unique<SdfEntityCreatorPrivate>())
{
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->eventManager = &_eventManager;
}

/////////////////////////////////////////////////
SdfEntityCreator::SdfEntityCreator(const SdfEntityCreator &_creator)
  : dataPtr(std::make_unique<SdfEntityCreatorPrivate>(*_creator.dataPtr))
{
}

/////////////////////////////////////////////////
SdfEntityCreator::SdfEntityCreator(SdfEntityCreator &&_creator) noexcept
    = default;

//////////////////////////////////////////////////
SdfEntityCreator::~SdfEntityCreator() = default;

/////////////////////////////////////////////////
SdfEntityCreator &SdfEntityCreator::operator=(const SdfEntityCreator &_creator)
{
  *this->dataPtr = (*_creator.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
SdfEntityCreator &SdfEntityCreator::operator=(SdfEntityCreator &&_creator)
    noexcept = default;

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::World *_world)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::World)");

  // World entity
  Entity worldEntity = this->dataPtr->ecm->CreateEntity();

  // World components
  this->dataPtr->ecm->CreateComponent(worldEntity, components::World());
  this->dataPtr->ecm->CreateComponent(worldEntity,
      components::Name(_world->Name()));

  // scene
  if (_world->Scene())
  {
    this->dataPtr->ecm->CreateComponent(worldEntity,
        components::Scene(*_world->Scene()));
  }

  // atmosphere
  if (_world->Atmosphere())
  {
    this->dataPtr->ecm->CreateComponent(worldEntity,
        components::Atmosphere(*_world->Atmosphere()));
  }

  // Models
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    auto model = _world->ModelByIndex(modelIndex);
    auto modelEntity = this->CreateEntities(model);

    this->SetParent(modelEntity, worldEntity);
  }

  // Actors
  for (uint64_t actorIndex = 0; actorIndex < _world->ActorCount();
      ++actorIndex)
  {
    auto actor = _world->ActorByIndex(actorIndex);
    auto actorEntity = this->CreateEntities(actor);

    this->SetParent(actorEntity, worldEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _world->LightCount();
      ++lightIndex)
  {
    auto light = _world->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->SetParent(lightEntity, worldEntity);
  }

  // Gravity
  this->dataPtr->ecm->CreateComponent(worldEntity,
      components::Gravity(_world->Gravity()));

  // Physics
  // \todo(anyone) Support picking a specific physics profile
  auto physics = _world->PhysicsByIndex(0);
  if (!physics)
  {
    physics = _world->PhysicsDefault();
  }
  this->dataPtr->ecm->CreateComponent(worldEntity,
      components::Physics(*physics));

  // MagneticField
  this->dataPtr->ecm->CreateComponent(worldEntity,
      components::MagneticField(_world->MagneticField()));

  this->dataPtr->eventManager->Emit<events::LoadPlugins>(worldEntity,
      _world->Element());

  // Store the world's SDF DOM to be used when saving the world to file
  this->dataPtr->ecm->CreateComponent(
      worldEntity, components::WorldSdf(*_world));

  return worldEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Model *_model)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Model)");

  auto ent = this->CreateEntities(_model, false);

  // Load all model plugins afterwards, so we get scoped name for nested models.
  for (const auto &[entity, element] : this->dataPtr->newModels)
  {
    this->dataPtr->eventManager->Emit<events::LoadPlugins>(entity, element);
  }
  this->dataPtr->newModels.clear();

  // Load sensor plugins after model, so we get scoped name.
  for (const auto &[entity, element] : this->dataPtr->newSensors)
  {
    this->dataPtr->eventManager->Emit<events::LoadPlugins>(entity, element);
  }
  this->dataPtr->newSensors.clear();

  // Load visual plugins after model, so we get scoped name.
  for (const auto &[entity, element] : this->dataPtr->newVisuals)
  {
    this->dataPtr->eventManager->Emit<events::LoadPlugins>(entity, element);
  }
  this->dataPtr->newVisuals.clear();

  return ent;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Model *_model,
                                        bool _staticParent)
{
  // Entity
  Entity modelEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(modelEntity, components::Model());
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Pose(ResolveSdfPose(_model->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Name(_model->Name()));
  bool isStatic = _model->Static() || _staticParent;
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Static(isStatic));
  this->dataPtr->ecm->CreateComponent(
      modelEntity, components::WindMode(_model->EnableWind()));
  this->dataPtr->ecm->CreateComponent(
      modelEntity, components::SelfCollide(_model->SelfCollide()));
  this->dataPtr->ecm->CreateComponent(
      modelEntity, components::SourceFilePath(_model->Element()->FilePath()));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  const auto *canonicalLink = _model->CanonicalLink();

  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->SetParent(linkEntity, modelEntity);

    if (canonicalLink == link)
    {
      this->dataPtr->ecm->CreateComponent(linkEntity,
          components::CanonicalLink());
    }

    // Set wind mode if the link didn't override it
    if (!this->dataPtr->ecm->Component<components::WindMode>(linkEntity))
    {
      this->dataPtr->ecm->CreateComponent(
          linkEntity, components::WindMode(_model->EnableWind()));
    }
  }

  // Joints
  for (uint64_t jointIndex = 0; jointIndex < _model->JointCount();
      ++jointIndex)
  {
    auto joint = _model->JointByIndex(jointIndex);
    auto jointEntity = this->CreateEntities(joint);

    this->SetParent(jointEntity, modelEntity);
  }

  // Nested Models
  for (uint64_t modelIndex = 0; modelIndex < _model->ModelCount();
      ++modelIndex)
  {
    auto nestedModel = _model->ModelByIndex(modelIndex);
    auto nestedModelEntity = this->CreateEntities(nestedModel, isStatic);

    this->SetParent(nestedModelEntity, modelEntity);
  }

  // Find canonical link
  const auto canonicalLinkPair = _model->CanonicalLinkAndRelativeName();
  if (canonicalLinkPair.first)
  {
    Entity canonicalLinkEntity = FindDescendentLinkEntityByName(
        canonicalLinkPair.second, modelEntity, *this->dataPtr->ecm);
    if (kNullEntity != canonicalLinkEntity)
    {
      this->dataPtr->ecm->CreateComponent(
          modelEntity, components::ModelCanonicalLink(canonicalLinkEntity));
    }
    else
    {
      ignerr << "Could not find the canonical link entity for "
             << canonicalLinkPair.second << "\n";
    }
  }
  else
  {
    ignerr << "Could not resolve the canonical link for " << _model->Name()
           << "\n";
  }

  // Store the model's SDF DOM to be used when saving the world to file
  this->dataPtr->ecm->CreateComponent(
      modelEntity, components::ModelSdf(*_model));

  // Keep track of models so we can load their plugins after loading the entire
  // model and having its full scoped name.
  this->dataPtr->newModels[modelEntity] = _model->Element();

  return modelEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Actor *_actor)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Actor)");

  // Entity
  Entity actorEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(actorEntity, components::Actor(*_actor));
  this->dataPtr->ecm->CreateComponent(actorEntity,
      components::Pose(_actor->RawPose()));
  this->dataPtr->ecm->CreateComponent(actorEntity,
      components::Name(_actor->Name()));

  // Actor plugins
  this->dataPtr->eventManager->Emit<events::LoadPlugins>(actorEntity,
      _actor->Element());

  return actorEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Light *_light)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Light)");

  // Entity
  Entity lightEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(lightEntity, components::Light(*_light));
  this->dataPtr->ecm->CreateComponent(lightEntity,
      components::Pose(ResolveSdfPose(_light->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(lightEntity,
      components::Name(_light->Name()));

  this->dataPtr->ecm->CreateComponent(lightEntity,
    components::LightType(convert(_light->Type())));

  return lightEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Link *_link)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Link)");

  // Entity
  Entity linkEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(linkEntity, components::Link());

  this->dataPtr->ecm->CreateComponent(linkEntity,
      components::Pose(ResolveSdfPose(_link->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(linkEntity,
      components::Name(_link->Name()));
  this->dataPtr->ecm->CreateComponent(linkEntity,
      components::Inertial(_link->Inertial()));

  if (_link->EnableWind())
  {
    this->dataPtr->ecm->CreateComponent(
        linkEntity, components::WindMode(_link->EnableWind()));
  }

  // Visuals
  for (uint64_t visualIndex = 0; visualIndex < _link->VisualCount();
      ++visualIndex)
  {
    auto visual = _link->VisualByIndex(visualIndex);
    auto visualEntity = this->CreateEntities(visual);

    this->SetParent(visualEntity, linkEntity);
  }

  // Collisions
  for (uint64_t collisionIndex = 0; collisionIndex < _link->CollisionCount();
      ++collisionIndex)
  {
    auto collision = _link->CollisionByIndex(collisionIndex);
    auto collisionEntity = this->CreateEntities(collision);

    this->SetParent(collisionEntity, linkEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _link->LightCount();
      ++lightIndex)
  {
    auto light = _link->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->SetParent(lightEntity, linkEntity);
  }

  // Sensors
  for (uint64_t sensorIndex = 0; sensorIndex < _link->SensorCount();
      ++sensorIndex)
  {
    auto sensor = _link->SensorByIndex(sensorIndex);
    auto sensorEntity = this->CreateEntities(sensor);

    this->SetParent(sensorEntity, linkEntity);
  }

  // Particle emitters
  for (uint64_t emitterIndex = 0; emitterIndex  < _link->ParticleEmitterCount();
       ++emitterIndex)
  {
    auto emitter = _link->ParticleEmitterByIndex(emitterIndex);
    auto emitterEntity = this->CreateEntities(emitter);

    this->SetParent(emitterEntity, linkEntity);
  }

  return linkEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Joint *_joint)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Joint)");

  // Entity
  Entity jointEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::Joint());
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::JointType(_joint->Type()));

  if (_joint->Axis(0))
  {
    auto resolvedAxis = ResolveJointAxis(*_joint->Axis(0));
    if (!resolvedAxis)
    {
      ignerr << "Failed to resolve joint axis 0 for joint '" << _joint->Name()
             << "'" << std::endl;
      return kNullEntity;
    }

    this->dataPtr->ecm->CreateComponent(jointEntity,
        components::JointAxis(std::move(*resolvedAxis)));
  }

  if (_joint->Axis(1))
  {
    auto resolvedAxis = ResolveJointAxis(*_joint->Axis(1));
    if (!resolvedAxis)
    {
      ignerr << "Failed to resolve joint axis 1 for joint '" << _joint->Name()
             << "'" << std::endl;
      return kNullEntity;
    }

    this->dataPtr->ecm->CreateComponent(jointEntity,
        components::JointAxis2(std::move(*resolvedAxis)));
  }

  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::Pose(ResolveSdfPose(_joint->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(jointEntity ,
      components::Name(_joint->Name()));
  this->dataPtr->ecm->CreateComponent(jointEntity ,
      components::ThreadPitch(_joint->ThreadPitch()));

  std::string resolvedParentLinkName;
  const auto resolveParentErrors =
    _joint->ResolveParentLink(resolvedParentLinkName);
  if (!resolveParentErrors.empty())
  {
    ignerr << "Failed to resolve parent link for joint '" << _joint->Name()
           << "' with parent name '" << _joint->ParentLinkName() << "'"
           << std::endl;

    return kNullEntity;
  }
  this->dataPtr->ecm->CreateComponent(
      jointEntity, components::ParentLinkName(resolvedParentLinkName));

  std::string resolvedChildLinkName;
  const auto resolveChildErrors =
    _joint->ResolveChildLink(resolvedChildLinkName);
  if (!resolveChildErrors.empty())
  {
    ignerr << "Failed to resolve child link for joint '" << _joint->Name()
           << "' with child name '" << _joint->ChildLinkName() << "'"
           << std::endl;
    for (const auto &error : resolveChildErrors)
    {
      ignerr << error << std::endl;
    }

    return kNullEntity;
  }

  this->dataPtr->ecm->CreateComponent(
      jointEntity, components::ChildLinkName(resolvedChildLinkName));

  return jointEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Visual *_visual)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Visual)");

  // Entity
  Entity visualEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(visualEntity, components::Visual());
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::Pose(ResolveSdfPose(_visual->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::Name(_visual->Name()));
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::CastShadows(_visual->CastShadows()));
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::Transparency(_visual->Transparency()));
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::VisibilityFlags(_visual->VisibilityFlags()));

  if (_visual->HasLaserRetro())
  {
    this->dataPtr->ecm->CreateComponent(visualEntity,
        components::LaserRetro(_visual->LaserRetro()));
  }

  if (_visual->Geom())
  {
    this->dataPtr->ecm->CreateComponent(visualEntity,
        components::Geometry(*_visual->Geom()));
  }

  // \todo(louise) Populate with default material if undefined
  if (_visual->Material())
  {
    this->dataPtr->ecm->CreateComponent(visualEntity,
        components::Material(*_visual->Material()));
  }

  // Keep track of visuals so we can load their plugins after loading the
  // entire model and having its full scoped name.
  this->dataPtr->newVisuals[visualEntity] = _visual->Element();

  return visualEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::ParticleEmitter *_emitter)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::ParticleEmitter)");

  // Entity
  Entity emitterEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(emitterEntity,
      components::ParticleEmitter(convert<msgs::ParticleEmitter>(*_emitter)));
  this->dataPtr->ecm->CreateComponent(emitterEntity,
      components::Pose(ResolveSdfPose(_emitter->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(emitterEntity,
      components::Name(_emitter->Name()));

  return emitterEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Collision *_collision)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Collision)");

  // Entity
  Entity collisionEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::Collision());
  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::Pose(ResolveSdfPose(_collision->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::Name(_collision->Name()));

  if (_collision->Geom())
  {
    this->dataPtr->ecm->CreateComponent(collisionEntity,
        components::Geometry(*_collision->Geom()));
  }

  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::CollisionElement(*_collision));

  return collisionEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Sensor *_sensor)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Sensor)");

  // Entity
  Entity sensorEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(sensorEntity,
      components::Sensor());
  this->dataPtr->ecm->CreateComponent(sensorEntity,
      components::Pose(ResolveSdfPose(_sensor->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(sensorEntity,
      components::Name(_sensor->Name()));

  if (_sensor->Type() == sdf::SensorType::CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Camera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::GPU_LIDAR)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::GpuLidar(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::LIDAR)
  {
    // \todo(anyone) Implement CPU-base lidar
    // this->dataPtr->ecm->CreateComponent(sensorEntity,
    //     components::Lidar(*_sensor));
    ignwarn << "Sensor type LIDAR not supported yet. Try using"
      << "a GPU LIDAR instead." << std::endl;
  }
  else if (_sensor->Type() == sdf::SensorType::DEPTH_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::DepthCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::RGBD_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::RgbdCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::THERMAL_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::ThermalCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::AIR_PRESSURE)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::AirPressureSensor(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::ALTIMETER)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Altimeter(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldLinearVelocity(math::Vector3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::IMU)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::Imu(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::WorldPose(math::Pose3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::AngularVelocity(math::Vector3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::LinearAcceleration(math::Vector3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::LOGICAL_CAMERA)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::LogicalCamera(elem));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::MAGNETOMETER)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Magnetometer(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::CONTACT)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::ContactSensor(elem));
    // We will let the contact system create the necessary components for
    // physics to populate.
  }
  else
  {
    ignwarn << "Sensor type [" << static_cast<int>(_sensor->Type())
            << "] not supported yet." << std::endl;
  }

  // Keep track of sensors so we can load their plugins after loading the entire
  // model and having its full scoped name.
  this->dataPtr->newSensors[sensorEntity] = _sensor->Element();

  return sensorEntity;
}

//////////////////////////////////////////////////
void SdfEntityCreator::RequestRemoveEntity(Entity _entity, bool _recursive)
{
  // Leave children parentless
  if (!_recursive)
  {
    auto childEntities = this->dataPtr->ecm->ChildrenByComponents(_entity,
        components::ParentEntity(_entity));
    for (const auto childEntity : childEntities)
    {
      this->dataPtr->ecm->RemoveComponent<components::ParentEntity>(
          childEntity);
    }
  }

  this->dataPtr->ecm->RequestRemoveEntity(_entity, _recursive);
}

//////////////////////////////////////////////////
void SdfEntityCreator::SetParent(Entity _child, Entity _parent)
{
  // TODO(louise) Figure out a way to avoid duplication while keeping all
  // state in components and also keeping a convenient graph in the ECM
  this->dataPtr->ecm->SetParentEntity(_child, _parent);
  this->dataPtr->ecm->CreateComponent(_child,
      components::ParentEntity(_parent));
}
