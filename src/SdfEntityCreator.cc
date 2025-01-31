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

#include <set>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <sdf/Actor.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/Types.hh>

#include "gz/sim/Events.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/AirSpeedSensor.hh"
#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Atmosphere.hh"
#include "gz/sim/components/BoundingBoxCamera.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/CastShadows.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/CustomSensor.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/LaserRetro.hh"
#include "gz/sim/components/Level.hh"
#include "gz/sim/components/LevelEntityNames.hh"
#include "gz/sim/components/Lidar.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LightType.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocitySeed.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/LogicalCamera.hh"
#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/NavSat.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include <gz/sim/components/ParticleEmitter.hh>
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/sim/components/Projector.hh>
#include "gz/sim/components/RgbdCamera.hh"
#include "gz/sim/components/Scene.hh"
#include "gz/sim/components/SegmentationCamera.hh"
#include "gz/sim/components/SelfCollide.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/SystemPluginInfo.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/ThreadPitch.hh"
#include "gz/sim/components/Transparency.hh"
#include "gz/sim/components/Visibility.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/WideAngleCamera.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/WindMode.hh"
#include "gz/sim/components/World.hh"

#include "rendering/MaterialParser/MaterialParser.hh"
#include "ServerPrivate.hh"

class gz::sim::SdfEntityCreatorPrivate
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Pointer to event manager. We don't assume ownership.
  public: EventManager *eventManager{nullptr};

  /// \brief Keep track of new sensors being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::Plugins> newSensors;

  /// \brief Keep track of new models being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::Plugins> newModels;

  /// \brief Keep track of new visuals being added, so we load their plugins
  /// only after we have their scoped name.
  public: std::map<Entity, sdf::Plugins> newVisuals;

  /// \brief Parse Gazebo defined materials for visuals
  public: MaterialParser materialParser;
};

using namespace gz;
using namespace sim;

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
    gzerr << "Failed to resolve axis" << std::endl;
    return std::nullopt;
  }

  sdf::JointAxis resolvedAxis = _unresolvedAxis;

  const sdf::Errors setXyzErrors = resolvedAxis.SetXyz(axisXyz);
  if (!setXyzErrors.empty())
  {
    gzerr << "Failed to resolve axis" << std::endl;
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
  auto ind = _name.find(sdf::kScopeDelimiter);
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
  this->dataPtr->materialParser.Load();
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

  // World entity
  Entity worldEntity = this->dataPtr->ecm->CreateEntity();

  this->CreateEntities(_world, worldEntity);
  return worldEntity;
}

//////////////////////////////////////////////////
void SdfEntityCreator::CreateEntities(const sdf::World *_world,
    Entity _worldEntity)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::World)");

  if (!this->dataPtr->ecm->EntityHasComponentType(
        _worldEntity, components::World::typeId))
  {
    this->dataPtr->ecm->CreateComponent(_worldEntity, components::World());
  }

  this->dataPtr->ecm->CreateComponent(_worldEntity,
      components::Name(_world->Name()));

  // Gravity
  this->dataPtr->ecm->CreateComponent(_worldEntity,
      components::Gravity(_world->Gravity()));

  // MagneticField
  this->dataPtr->ecm->CreateComponent(_worldEntity,
      components::MagneticField(_world->MagneticField()));

  // Create Wind
  auto windEntity = this->dataPtr->ecm->CreateEntity();
  this->SetParent(windEntity, _worldEntity);
  this->dataPtr->ecm->CreateComponent(windEntity, components::Wind());
  this->dataPtr->ecm->CreateComponent(windEntity,
      components::WorldLinearVelocity(_world->WindLinearVelocity()));
  // Initially the wind linear velocity is used as the seed velocity
  this->dataPtr->ecm->CreateComponent(windEntity,
      components::WorldLinearVelocitySeed(_world->WindLinearVelocity()));

  // Set the parent of each level to the world
  this->dataPtr->ecm->Each<components::Level>([&](
        const Entity &_entity,
        const components::Level *) -> bool
  {
    this->SetParent(_entity, _worldEntity);
    return true;
  });

  // Get the entities that should be loaded based on level information.
  std::set<std::string> levelEntityNames;
  this->dataPtr->ecm->Each<components::DefaultLevel,
    components::LevelEntityNames> ([&](
          const Entity &,
          const components::DefaultLevel *,
          const components::LevelEntityNames *_names) -> bool
  {
    levelEntityNames = _names->Data();
    return true;
  });

  // scene
  if (_world->Scene())
  {
    this->dataPtr->ecm->CreateComponent(_worldEntity,
        components::Scene(*_world->Scene()));
  }

  // atmosphere
  if (_world->Atmosphere())
  {
    this->dataPtr->ecm->CreateComponent(_worldEntity,
        components::Atmosphere(*_world->Atmosphere()));
  }

  // spherical coordinates
  if (_world->SphericalCoordinates())
  {
    this->dataPtr->ecm->CreateComponent(_worldEntity,
        components::SphericalCoordinates(*_world->SphericalCoordinates()));
  }

  this->dataPtr->eventManager->Emit<events::LoadSdfPlugins>(_worldEntity,
      _world->Plugins());

  // Models
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    const sdf::Model *model = _world->ModelByIndex(modelIndex);
    if (levelEntityNames.empty() ||
        levelEntityNames.find(model->Name()) != levelEntityNames.end())

    {
      Entity modelEntity = this->CreateEntities(model);

      this->SetParent(modelEntity, _worldEntity);
    }
  }

  // Actors
  for (uint64_t actorIndex = 0; actorIndex < _world->ActorCount();
      ++actorIndex)
  {
    const sdf::Actor *actor = _world->ActorByIndex(actorIndex);
    if (levelEntityNames.empty() ||
        levelEntityNames.find(actor->Name()) != levelEntityNames.end())
    {
      Entity actorEntity = this->CreateEntities(actor);
      this->SetParent(actorEntity, _worldEntity);
    }
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _world->LightCount();
      ++lightIndex)
  {
    const sdf::Light *light = _world->LightByIndex(lightIndex);
    if (levelEntityNames.empty() ||
        levelEntityNames.find(light->Name()) != levelEntityNames.end())
    {
      Entity lightEntity = this->CreateEntities(light);

      this->SetParent(lightEntity, _worldEntity);
    }
  }

  // Attach performers to their parent entity
  this->dataPtr->ecm->Each<
    components::Performer,
    components::PerformerRef>([&](
          const Entity &_entity,
          const components::Performer *,
          const components::PerformerRef *_ref) -> bool
  {
    std::optional<Entity> parentEntity =
      this->dataPtr->ecm->EntityByName(_ref->Data());
    if (!parentEntity)
    {
      // Performers have not been created yet. Try to create the model
      // or actor and attach the peformer.
      if (_world->ModelNameExists(_ref->Data()))
      {
        const sdf::Model *model = _world->ModelByName(_ref->Data());
        Entity modelEntity = this->CreateEntities(model);
        this->SetParent(modelEntity, _worldEntity);
        this->SetParent(_entity, modelEntity);
      }
      else if (_world->ActorNameExists(_ref->Data()))
      {
        const sdf::Actor *actor = _world->ActorByName(_ref->Data());
        Entity actorEntity = this->CreateEntities(actor);
        this->SetParent(actorEntity, _worldEntity);
        this->SetParent(_entity, actorEntity);
      }
      else
      {
        gzerr << "Unable to find performer parent entity with name[" <<
          _ref->Data() << "]. This performer will not adhere to levels.\n";
      }
    }
    else
    {
      this->SetParent(_entity, *parentEntity);
    }
    return true;
  });

  // Physics
  // \todo(anyone) Support picking a specific physics profile
  auto physics = _world->PhysicsByIndex(0);
  if (!physics)
  {
    physics = _world->PhysicsDefault();
  }
  this->dataPtr->ecm->CreateComponent(_worldEntity,
      components::Physics(*physics));

  // Populate physics options that aren't accessible outside the Element()
  // See https://github.com/osrf/sdformat/issues/508
  if (physics->Element() && physics->Element()->HasElement("dart"))
  {
    auto dartElem = physics->Element()->GetElement("dart");

    if (dartElem->HasElement("collision_detector"))
    {
      auto collisionDetector =
          dartElem->Get<std::string>("collision_detector");

      this->dataPtr->ecm->CreateComponent(_worldEntity,
          components::PhysicsCollisionDetector(collisionDetector));
    }
    if (dartElem->HasElement("solver") &&
        dartElem->GetElement("solver")->HasElement("solver_type"))
    {
      auto solver =
          dartElem->GetElement("solver")->Get<std::string>("solver_type");

      this->dataPtr->ecm->CreateComponent(_worldEntity,
          components::PhysicsSolver(solver));
    }
  }

  // Store the world's SDF DOM to be used when saving the world to file
  this->dataPtr->ecm->CreateComponent(
      _worldEntity, components::WorldSdf(*_world));
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Model *_model)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Model)");

  auto ent = this->CreateEntities(_model, false);

  // Load all model plugins afterwards, so we get scoped name for nested models.
  this->LoadModelPlugins();

  return ent;
}

//////////////////////////////////////////////////
void SdfEntityCreator::LoadModelPlugins()
{
  for (const auto &[entity, plugins] : this->dataPtr->newModels)
  {
    this->dataPtr->eventManager->Emit<events::LoadSdfPlugins>(entity, plugins);
  }
  this->dataPtr->newModels.clear();

  // Load sensor plugins after model, so we get scoped name.
  for (const auto &[entity, plugins] : this->dataPtr->newSensors)
  {
    this->dataPtr->eventManager->Emit<events::LoadSdfPlugins>(entity, plugins);
  }
  this->dataPtr->newSensors.clear();

  // Load visual plugins after model, so we get scoped name.
  for (const auto &[entity, plugins] : this->dataPtr->newVisuals)
  {
    this->dataPtr->eventManager->Emit<events::LoadSdfPlugins>(entity, plugins);
  }
  this->dataPtr->newVisuals.clear();
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
  if (_model->Element())
  {
    this->dataPtr->ecm->CreateComponent(
        modelEntity, components::SourceFilePath(_model->Element()->FilePath()));
  }

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

    if (link->AutoInertia())
    {
      gzdbg << "Link has auto-inertial enabled: "
            << scopedName(linkEntity, *this->dataPtr->ecm, "::", false) << "\n";
      gzdbg << "  pose: " << link->Inertial().Pose() << "\n";
      gzdbg << "  mass: " << link->Inertial().MassMatrix().Mass() << "\n";
      gzdbg << "  ixx iyy izz: "
            << link->Inertial().MassMatrix().DiagonalMoments() << "\n";
      gzdbg << "  ixy ixz iyz: "
            << link->Inertial().MassMatrix().OffDiagonalMoments()
            << std::endl;
    }

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
      gzerr << "Could not find the canonical link entity for "
             << canonicalLinkPair.second << "\n";
    }
  }
  else if (!isStatic)
  {
    gzerr << "Could not resolve the canonical link for " << _model->Name()
           << "\n";
  }

  // Store the model's SDF DOM to be used when saving the world to file
  this->dataPtr->ecm->CreateComponent(
      modelEntity, components::ModelSdf(*_model));

  // Keep track of models so we can load their plugins after loading the entire
  // model and having its full scoped name.
  if (!_model->Plugins().empty())
    this->dataPtr->newModels[modelEntity] = _model->Plugins();

  return modelEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Actor *_actor)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Actor)");

  // Entity
  Entity actorEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(actorEntity, components::Actor(*_actor));
  this->dataPtr->ecm->CreateComponent(actorEntity,
      components::Pose(_actor->RawPose()));
  this->dataPtr->ecm->CreateComponent(actorEntity,
      components::Name(_actor->Name()));

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _actor->LinkCount();
      ++linkIndex)
  {
    auto link = _actor->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->SetParent(linkEntity, actorEntity);
  }

  // Actor plugins
  this->dataPtr->eventManager->Emit<events::LoadSdfPlugins>(actorEntity,
        _actor->Plugins());

  return actorEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Light *_light)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Light)");

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

  // Light Visual
  Entity lightVisualEntity = this->dataPtr->ecm->CreateEntity();
  this->dataPtr->ecm->CreateComponent(lightVisualEntity, components::Visual());
  this->dataPtr->ecm->CreateComponent(lightVisualEntity,
      components::Pose());
  this->dataPtr->ecm->CreateComponent(lightVisualEntity,
      components::Name(_light->Name() + "Visual"));
  this->dataPtr->ecm->CreateComponent(lightVisualEntity,
      components::CastShadows(false));
  this->dataPtr->ecm->CreateComponent(lightVisualEntity,
      components::Transparency(false));
  this->SetParent(lightVisualEntity, lightEntity);

  return lightEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Link *_link)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Link)");

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

  if (!_link->EnableGravity())
  {
    // If disable gravity, create a GravityEnabled component to the entity
    this->dataPtr->ecm->CreateComponent(
        linkEntity, components::GravityEnabled(false));
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
  for (uint64_t emitterIndex = 0; emitterIndex < _link->ParticleEmitterCount();
       ++emitterIndex)
  {
    auto emitter = _link->ParticleEmitterByIndex(emitterIndex);
    auto emitterEntity = this->CreateEntities(emitter);

    this->SetParent(emitterEntity, linkEntity);
  }

  // Projectors
  for (uint64_t projectorIndex = 0; projectorIndex < _link->ProjectorCount();
       ++projectorIndex)
  {
    auto projector = _link->ProjectorByIndex(projectorIndex);
    auto projectorEntity = this->CreateEntities(projector);

    this->SetParent(projectorEntity, linkEntity);
  }


  return linkEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Joint *_joint)
{
  return this->CreateEntities(_joint, false);
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Joint *_joint,
    bool _resolved)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Joint)");

  // Entity
  Entity jointEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::Joint());
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::JointType(_joint->Type()));

  // Sensors
  for (uint64_t sensorIndex = 0; sensorIndex < _joint->SensorCount();
      ++sensorIndex)
  {
    auto sensor = _joint->SensorByIndex(sensorIndex);
    auto sensorEntity = this->CreateEntities(sensor);

    this->SetParent(sensorEntity, jointEntity);
  }

  if (_joint->Axis(0))
  {
    auto resolvedAxis = ResolveJointAxis(*_joint->Axis(0));
    if (!resolvedAxis)
    {
      gzerr << "Failed to resolve joint axis 0 for joint '" << _joint->Name()
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
      gzerr << "Failed to resolve joint axis 1 for joint '" << _joint->Name()
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
  if (_resolved)
  {
    resolvedParentLinkName = _joint->ParentName();
  }
  else
  {

    const auto resolveParentErrors =
      _joint->ResolveParentLink(resolvedParentLinkName);
    if (!resolveParentErrors.empty())
    {
      gzerr << "Failed to resolve parent link for joint '" << _joint->Name()
             << "' with parent name '" << _joint->ParentName() << "'"
             << std::endl;
      for (const auto &error : resolveParentErrors)
      {
        gzerr << error << std::endl;
      }

      return kNullEntity;
    }
  }
  this->dataPtr->ecm->CreateComponent(
      jointEntity, components::ParentLinkName(resolvedParentLinkName));

  std::string resolvedChildLinkName;
  if (_resolved)
  {
    resolvedChildLinkName = _joint->ChildName();
  }
  else
  {
    const auto resolveChildErrors =
      _joint->ResolveChildLink(resolvedChildLinkName);
    if (!resolveChildErrors.empty())
    {
      gzerr << "Failed to resolve child link for joint '" << _joint->Name()
             << "' with child name '" << _joint->ChildName() << "'"
             << std::endl;
      for (const auto &error : resolveChildErrors)
      {
        gzerr << error << std::endl;
      }

      return kNullEntity;
    }
  }

  this->dataPtr->ecm->CreateComponent(
      jointEntity, components::ChildLinkName(resolvedChildLinkName));

  return jointEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Visual *_visual)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Visual)");

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
    sdf::Material visualMaterial = *_visual->Material();
    if (!_visual->Material()->ScriptUri().empty())
    {
      gzwarn << "Gazebo does not support Ogre material scripts. See " <<
      "https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials " <<
      "for details." << std::endl;
      std::string scriptUri = visualMaterial.ScriptUri();
      if (scriptUri != ServerPrivate::kClassicMaterialScriptUri)
      {
        gzwarn << "Custom material scripts are not supported."
          << std::endl;
      }
    }
    if (!_visual->Material()->ScriptName().empty())
    {
      std::string scriptName = visualMaterial.ScriptName();

      if ((scriptName.find("Gazebo/") == 0u))
      {
        gzwarn << "Using an internal gazebo.material to parse "
          << scriptName << std::endl;
        std::optional<MaterialParser::MaterialValues> parsed =
          this->dataPtr->materialParser.GetMaterialValues(scriptName);

        if(parsed.has_value())
        {
          visualMaterial.SetAmbient
            (parsed->ambient.value_or(visualMaterial.Ambient()));
          visualMaterial.SetDiffuse
            (parsed->diffuse.value_or(visualMaterial.Diffuse()));
          visualMaterial.SetSpecular
            (parsed->specular.value_or(visualMaterial.Specular()));
        }
        else
        {
          gzwarn << "Material " << scriptName <<
            " not recognized or supported, using default." << std::endl;
        }
      }
    }
    this->dataPtr->ecm->CreateComponent(visualEntity,
        components::Material(visualMaterial));
  }

  // store the plugin in a component
  if (!_visual->Plugins().empty())
  {
    this->dataPtr->ecm->CreateComponent(visualEntity,
        components::SystemPluginInfo(
          convert<msgs::Plugin_V>(_visual->Plugins())));
  }
  // Deprecate this in Garden
  if (_visual->Element())
  {
    sdf::ElementPtr pluginElem = _visual->Element()->FindElement("plugin");
    if (pluginElem)
    {
      this->dataPtr->ecm->CreateComponent(visualEntity,
          components::VisualPlugin(pluginElem));
    }
  }

  // Keep track of visuals so we can load their plugins after loading the
  // entire model and having its full scoped name.
  if (!_visual->Plugins().empty())
    this->dataPtr->newVisuals[visualEntity] = _visual->Plugins();

  return visualEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::ParticleEmitter *_emitter)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::ParticleEmitter)");

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
Entity SdfEntityCreator::CreateEntities(const sdf::Projector *_projector)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Projector)");

  // Entity
  Entity projectorEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(projectorEntity,
      components::Projector(*_projector));
  this->dataPtr->ecm->CreateComponent(projectorEntity,
      components::Pose(ResolveSdfPose(_projector->SemanticPose())));
  this->dataPtr->ecm->CreateComponent(projectorEntity,
      components::Name(_projector->Name()));

  return projectorEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Collision *_collision)
{
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Collision)");

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
  GZ_PROFILE("SdfEntityCreator::CreateEntities(sdf::Sensor)");

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
    // \todo(anyone) Implement CPU-based lidar
    // this->dataPtr->ecm->CreateComponent(sensorEntity,
    //     components::Lidar(*_sensor));
    gzwarn << "Sensor type LIDAR not supported yet. Try using"
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
  else if (_sensor->Type() == sdf::SensorType::SEGMENTATION_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::SegmentationCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::BOUNDINGBOX_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::BoundingBoxCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::WIDE_ANGLE_CAMERA)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WideAngleCamera(*_sensor));
  }
  else if (_sensor->Type() == sdf::SensorType::AIR_PRESSURE)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::AirPressureSensor(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::AIR_SPEED)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::AirSpeedSensor(*_sensor));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldLinearVelocity(math::Vector3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldAngularVelocity(math::Vector3d::Zero));
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
  else if (_sensor->Type() == sdf::SensorType::GPS ||
           _sensor->Type() == sdf::SensorType::NAVSAT)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::NavSat(*_sensor));

    // Create components to be filled by physics.
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
  else if (_sensor->Type() == sdf::SensorType::FORCE_TORQUE)
  {
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::ForceTorque(*_sensor));
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
  else if (_sensor->Type() == sdf::SensorType::CUSTOM)
  {
    auto elem = _sensor->Element();
    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::CustomSensor(*_sensor));
  }
  else
  {
    gzwarn << "Sensor type [" << static_cast<int>(_sensor->Type())
            << "] not supported yet." << std::endl;
  }

  // Keep track of sensors so we can load their plugins after loading the entire
  // model and having its full scoped name.
  if (!_sensor->Plugins().empty())
    this->dataPtr->newSensors[sensorEntity] = _sensor->Plugins();

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
