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

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/common/Time.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/Manager.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "SceneManager.hh"
#include "Sensors.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::SensorsPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief Create / update rendering entities
  /// \param[in] _ecm The entity-component manager
  public: void UpdateRenderingEntities(const EntityComponentManager &_ecm);

  /// \brief Name of rendering engine
  public: std::string engineName;

  /// \brief Scene manager
  public: SceneManager sceneManager;
};

//////////////////////////////////////////////////
Sensors::Sensors() : System(), dataPtr(std::make_unique<SensorsPrivate>())
{
}

//////////////////////////////////////////////////
Sensors::~Sensors()
{
}

//////////////////////////////////////////////////
void Sensors::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  // Setup rendering
  this->dataPtr->engineName =
      _sdf->Get<std::string>("render_engine", "ogre").first;
}

//////////////////////////////////////////////////
void Sensors::Update(const UpdateInfo &/*_info*/, EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    // TODO(anyone) Only do this if we do have rendering sensors
    auto engine = ignition::rendering::engine(this->dataPtr->engineName);
    if (!engine)
    {
      ignerr << "Failed to load engine ["
             << this->dataPtr->engineName << "]" << std::endl;
      return;
    }
    auto scene = engine->CreateScene("scene");
    // Create simulation runner sensor manager
    this->dataPtr->sensorManager.SetRenderingScene(scene);
    this->dataPtr->sceneManager.SetScene(scene);

    this->dataPtr->initialized = true;
  }
  this->dataPtr->UpdateRenderingEntities(_ecm);
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &/*_ecm*/)
{
  auto time = math::durationToSecNsec(_info.simTime);
  this->dataPtr->sensorManager.RunOnce(common::Time(time.first, time.second));
}

//////////////////////////////////////////////////
void SensorsPrivate::UpdateRenderingEntities(const EntityComponentManager &_ecm)
{
  // TODO(anyone) support multiple scenes?
  // Get all the worlds
  // _ecm.Each<components::World, components::Name>(
  //     [&](const Entity &_entity,
  //       const components::World * /* _world */,
  //       const components::Name *_name)->bool
  //     {
  //       return true;
  //     });

  _ecm.Each<components::Model, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Model * /* _model */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        auto entity = this->sceneManager.EntityById(_entity);
        if (!entity)
        {
          sdf::Model model;
          model.SetName(_name->Data());
          model.SetPose(_pose->Data());
          this->sceneManager.CreateModel(_entity, model,
              _parent->Data());
        }
        else
        {
          entity->SetLocalPose(_pose->Data());
        }
        return true;
      });

  _ecm.Each<components::Link, components::Name, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Link * /* _link */,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        auto entity = this->sceneManager.EntityById(_entity);
        if (!entity)
        {
          sdf::Link link;
          link.SetName(_name->Data());
          link.SetPose(_pose->Data());

          this->sceneManager.CreateLink(_entity, link,
              _parent->Data());
        }
        else
        {
          entity->SetLocalPose(_pose->Data());
        }

        return true;
      });

  // visuals
  _ecm.Each<components::Visual, components::Name, components::Pose,
            components::Geometry, components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Visual * /*_visual*/,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::Geometry *_geom,
        const components::ParentEntity *_parent)->bool
      {
        auto entity = this->sceneManager.EntityById(_entity);
        if (!entity)
        {
          sdf::Visual visual;
          visual.SetName(_name->Data());
          visual.SetPose(_pose->Data());
          visual.SetGeom(_geom->Data());

          // Optional components
          auto material = _ecm.Component<components::Material>(_entity);
          if (material)
          {
            visual.SetMaterial(material->Data());
          }

          this->sceneManager.CreateVisual(_entity, visual,
              _parent->Data());
        }
        else
        {
          entity->SetLocalPose(_pose->Data());
        }

        return true;
      });

  // lights
  _ecm.Each<components::Light, components::Pose, components::ParentEntity>(
      [&](const Entity &_entity,
        const components::Light*  _light,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        auto entity = this->sceneManager.EntityById(_entity);
        if (!entity)
        {
          this->sceneManager.CreateLight(_entity, _light->Data(),
              _parent->Data());
        }
        else
        {
          entity->SetLocalPose(_pose->Data());
        }

        return true;
      });

  // Create cameras
  _ecm.Each<components::Camera, components::Pose, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Camera *_camera,
        const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
      {
        auto entity = this->sceneManager.EntityById(_entity);
        if (!entity)
        {
          // two camera models with the same camera sensor name
          // causes name conflicts. We'll need to use scoped names
          // TODO(anyone) do this in ign-sensors?
          auto parent = sceneManager.EntityById(_parent->Data());
          if (!parent)
            return false;
          auto data = _camera->Data()->Clone();
          std::string scopedName = parent->Name() + "::"
              + data->Get<std::string>("name");
          data->GetAttribute("name")->Set(scopedName);
          auto sensor =
              this->sensorManager.CreateSensor<sensors::CameraSensor>(data);
          return this->sceneManager.AddSensor(
              _entity, sensor->Name(), _parent->Data());
        }
        else
        {
          entity->SetLocalPose(_pose->Data());
        }

        return true;
      });
}

IGNITION_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemUpdate,
  Sensors::ISystemPostUpdate
)
