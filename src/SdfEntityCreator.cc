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

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
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
#include "ignition/gazebo/components/Light.hh"
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
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

class ignition::gazebo::SdfEntityCreatorPrivate
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Pointer to event manager. We don't assume ownership.
  public: EventManager *eventManager{nullptr};
};

using namespace ignition;
using namespace gazebo;

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

  // Models
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    auto model = _world->ModelByIndex(modelIndex);
    auto modelEntity = this->CreateEntities(model);

    this->SetParent(modelEntity, worldEntity);
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

  // MagneticField
  this->dataPtr->ecm->CreateComponent(worldEntity,
      components::MagneticField(_world->MagneticField()));

  this->dataPtr->eventManager->Emit<events::LoadPlugins>(worldEntity,
      _world->Element());

  return worldEntity;
}

//////////////////////////////////////////////////
Entity SdfEntityCreator::CreateEntities(const sdf::Model *_model)
{
  IGN_PROFILE("SdfEntityCreator::CreateEntities(sdf::Model)");

  // Entity
  Entity modelEntity = this->dataPtr->ecm->CreateEntity();

  // Components
  this->dataPtr->ecm->CreateComponent(modelEntity, components::Model());
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Pose(_model->Pose()));
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Name(_model->Name()));
  this->dataPtr->ecm->CreateComponent(modelEntity,
      components::Static(_model->Static()));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->SetParent(linkEntity, modelEntity);
    if (linkIndex == 0)
    {
      this->dataPtr->ecm->CreateComponent(linkEntity,
          components::CanonicalLink());
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

  // Model plugins
  this->dataPtr->eventManager->Emit<events::LoadPlugins>(modelEntity,
      _model->Element());

  return modelEntity;
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
      components::Pose(_light->Pose()));
  this->dataPtr->ecm->CreateComponent(lightEntity,
      components::Name(_light->Name()));

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
      components::Pose(_link->Pose()));
  this->dataPtr->ecm->CreateComponent(linkEntity,
      components::Name(_link->Name()));
  this->dataPtr->ecm->CreateComponent(linkEntity,
      components::Inertial(_link->Inertial()));

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
    this->dataPtr->ecm->CreateComponent(jointEntity,
        components::JointAxis(*_joint->Axis(0)));
  }

  if (_joint->Axis(1))
  {
    this->dataPtr->ecm->CreateComponent(jointEntity,
        components::JointAxis2(*_joint->Axis(1)));
  }

  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::Pose(_joint->Pose()));
  this->dataPtr->ecm->CreateComponent(jointEntity ,
      components::Name(_joint->Name()));
  this->dataPtr->ecm->CreateComponent(jointEntity ,
      components::ThreadPitch(_joint->ThreadPitch()));
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::ParentLinkName(_joint->ParentLinkName()));
  this->dataPtr->ecm->CreateComponent(jointEntity,
      components::ChildLinkName(_joint->ChildLinkName()));

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
      components::Pose(_visual->Pose()));
  this->dataPtr->ecm->CreateComponent(visualEntity,
      components::Name(_visual->Name()));

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

  return visualEntity;
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
      components::Pose(_collision->Pose()));
  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::Name(_collision->Name()));

  if (_collision->Geom())
  {
    this->dataPtr->ecm->CreateComponent(collisionEntity,
        components::Geometry(*_collision->Geom()));
  }

  this->dataPtr->ecm->CreateComponent(collisionEntity,
      components::CollisionElement(_collision->Element()));

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
      components::Pose(_sensor->Pose()));
  this->dataPtr->ecm->CreateComponent(sensorEntity,
      components::Name(_sensor->Name()));

  if (_sensor->Type() == sdf::SensorType::CAMERA)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Camera(elem));
  }
  else if (_sensor->Type() == sdf::SensorType::GPU_LIDAR)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::GpuLidar(elem));
  }
  else if (_sensor->Type() == sdf::SensorType::DEPTH_CAMERA)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::DepthCamera(elem));
  }
  else if (_sensor->Type() == sdf::SensorType::ALTIMETER)
  {
     auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Altimeter(elem));

    // create components to be filled by physics
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::WorldLinearVelocity(math::Vector3d::Zero));
  }
  else if (_sensor->Type() == sdf::SensorType::IMU)
  {
    auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
            components::Imu(elem));

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
     auto elem = _sensor->Element();

    this->dataPtr->ecm->CreateComponent(sensorEntity,
        components::Magnetometer(elem));

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
