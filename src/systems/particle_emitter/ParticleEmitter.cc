/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/msgs/particle_emitter.pb.h>

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Material.hh>

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParticleEmitter.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Conversions.hh>
#include "ParticleEmitter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::ParticleEmitterPrivate
{
  /// \brief The particle emitter.
  public: ignition::msgs::ParticleEmitter emitter;
};

//////////////////////////////////////////////////
ParticleEmitter::ParticleEmitter()
  : System(), dataPtr(std::make_unique<ParticleEmitterPrivate>())
{
}

//////////////////////////////////////////////////
void ParticleEmitter::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  // Name.
  if (!_sdf->HasElement("emitter_name"))
  {
    ignerr << "Missing <emitter_name>. Ignoring particle emitter." << std::endl;
    return;
  }
  this->dataPtr->emitter.set_name(
    _sdf->Get<std::string>("emitter_name"));

  // Type. The default type is point.
  this->dataPtr->emitter.set_type(
      ignition::msgs::ParticleEmitter_EmitterType_POINT);
  std::string type = _sdf->Get<std::string>("type", "point").first;
  if (type == "box")
  {
    this->dataPtr->emitter.set_type(
      ignition::msgs::ParticleEmitter_EmitterType_BOX);
  }
  else if (type == "cylinder")
  {
    this->dataPtr->emitter.set_type(
      ignition::msgs::ParticleEmitter_EmitterType_CYLINDER);
  }
  else if (type == "ellipsoid")
  {
    this->dataPtr->emitter.set_type(
      ignition::msgs::ParticleEmitter_EmitterType_ELLIPSOID);
  }
  else if (type != "point")
  {
    ignerr << "Unknown emitter type [" << type << "]. Using [point] instead"
           << std::endl;
  }

  // Pose.
  ignition::math::Pose3d pose =
    _sdf->Get<ignition::math::Pose3d>("pose");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_pose(), pose);

  // Size.
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  if (_sdf->HasElement("size"))
    size = _sdf->Get<ignition::math::Vector3d>("size");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_size(), size);

  // Rate.
  this->dataPtr->emitter.set_rate(_sdf->Get<double>("rate", 10).first);

  // Duration.
  this->dataPtr->emitter.set_duration(_sdf->Get<double>("duration", 0).first);

  // Emitting.
  this->dataPtr->emitter.set_emitting(_sdf->Get<bool>("emitting", false).first);

  // Particle size.
  size = ignition::math::Vector3d::One;
  if (_sdf->HasElement("particle_size"))
    size = _sdf->Get<ignition::math::Vector3d>("particle_size");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_particle_size(), size);

  // Lifetime.
  this->dataPtr->emitter.set_lifetime(_sdf->Get<double>("lifetime", 5).first);

  // Material.
  if (_sdf->HasElement("material"))
  {
    auto materialElem = _sdf->GetElementImpl("material");
    sdf::Material material;
    material.Load(materialElem);
    ignition::msgs::Material materialMsg = convert<msgs::Material>(material);
  }

  // Min velocity.
  this->dataPtr->emitter.set_min_velocity(
    _sdf->Get<double>("min_velocity", 1).first);

  // Max velocity.
  this->dataPtr->emitter.set_max_velocity(
    _sdf->Get<double>("max_velocity", 1).first);

  // Color start.
  ignition::math::Color color = ignition::math::Color::White;
  if (_sdf->HasElement("color_start"))
    color = _sdf->Get<ignition::math::Color>("color_start");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_color_start(), color);

  // Color end.
  color = ignition::math::Color::White;
  if (_sdf->HasElement("color_end"))
    color = _sdf->Get<ignition::math::Color>("color_end");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_color_end(), color);

  // Scale rate.
  this->dataPtr->emitter.set_scale_rate(
    _sdf->Get<double>("scale_rate", 1).first);

  // Color range image.
  this->dataPtr->emitter.set_color_range_image(
    _sdf->Get<std::string>("color_range_image", "").first);

  igndbg << "Loading particle emitter:" << std::endl
         << this->dataPtr->emitter.DebugString() << std::endl;

  // Create an audio source entity.
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    ignerr << "Failed to create a particle emitter entity" << std::endl;
    return;
  }

  // Create components.
  _ecm.CreateComponent(entity,
    components::Name("particle_emitter_" + this->dataPtr->emitter.name()));

  particles::Emitter emitterData;
  emitterData.data.CopyFrom(this->dataPtr->emitter);
  _ecm.CreateComponent(entity, components::ParticleEmitter(emitterData));

  _ecm.CreateComponent(entity, components::Pose(pose));
}

//////////////////////////////////////////////////
void ParticleEmitter::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ParticleEmitter::PreUpdate");
}

IGNITION_ADD_PLUGIN(ParticleEmitter,
                    ignition::gazebo::System,
                    ParticleEmitter::ISystemConfigure,
                    ParticleEmitter::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParticleEmitter,
                          "ignition::gazebo::systems::ParticleEmitter")
