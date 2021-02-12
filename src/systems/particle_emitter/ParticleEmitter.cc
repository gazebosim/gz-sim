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
#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParticleEmitter.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Conversions.hh>
#include <sdf/Material.hh>
#include "ParticleEmitter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::ParticleEmitterPrivate
{
  /// \brief The particle emitter.
  public: ignition::msgs::ParticleEmitter emitter;

  /// \brief Get a RGBA color representation based on a color's
  /// string representation.
  /// \param[in] _colorStr The string representation of a color (ex: "black"),
  /// which is case sensitive (the string representation should be lowercase).
  /// \return The Color, represented in RGBA format. If _colorStr is invalid,
  /// ignition::math::Color::White is returned
  public: ignition::math::Color GetColor(const std::string &_colorStr) const;
};

//////////////////////////////////////////////////
ParticleEmitter::ParticleEmitter()
  : System(), dataPtr(std::make_unique<ParticleEmitterPrivate>())
{
}

//////////////////////////////////////////////////
void ParticleEmitter::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Create a particle emitter entity.
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    ignerr << "Failed to create a particle emitter entity" << std::endl;
    return;
  }

  // Name.
  std::string name = "particle_emitter_entity_" + std::to_string(entity);
  if (_sdf->HasElement("emitter_name"))
    name = _sdf->Get<std::string>("emitter_name");
  this->dataPtr->emitter.set_name(name);

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
  ignition::msgs::Set(this->dataPtr->emitter.mutable_color_start(),
      this->dataPtr->GetColor(_sdf->Get<std::string>("color_start")));

  // Color end.
  ignition::msgs::Set(this->dataPtr->emitter.mutable_color_end(),
      this->dataPtr->GetColor(_sdf->Get<std::string>("color_end")));

  // Scale rate.
  this->dataPtr->emitter.set_scale_rate(
    _sdf->Get<double>("scale_rate", 1).first);

  // Color range image.
  this->dataPtr->emitter.set_color_range_image(
    _sdf->Get<std::string>("color_range_image", "").first);

  igndbg << "Loading particle emitter:" << std::endl
         << this->dataPtr->emitter.DebugString() << std::endl;

  // Create components.
  _ecm.CreateComponent(entity, components::Name(this->dataPtr->emitter.name()));

  _ecm.CreateComponent(entity,
    components::ParticleEmitter(this->dataPtr->emitter));

  _ecm.CreateComponent(entity, components::Pose(pose));

  igndbg << "Particle emitter has been loaded." << std::endl;
}

//////////////////////////////////////////////////
void ParticleEmitter::PreUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("ParticleEmitter::PreUpdate");
}

//////////////////////////////////////////////////
ignition::math::Color ParticleEmitterPrivate::GetColor(
    const std::string &_colorStr) const
{
  if (_colorStr == "black")
    return  ignition::math::Color::Black;
  if (_colorStr == "red")
    return ignition::math::Color::Red;
  if (_colorStr == "green")
    return ignition::math::Color::Green;
  if (_colorStr == "blue")
    return ignition::math::Color::Blue;
  if (_colorStr == "yellow")
    return ignition::math::Color::Yellow;
  if (_colorStr == "magenta")
    return ignition::math::Color::Magenta;
  if (_colorStr == "cyan")
    return ignition::math::Color::Cyan;

  // let users know an invalid string was given
  // (_colorStr.empty() means that an empty string was parsed from SDF,
  // which probably means that users never specified a color and are
  // relying on the defaults)
  if (!_colorStr.empty() && (_colorStr != "white"))
  {
    ignwarn << "Invalid color given (" << _colorStr
      << "). Defaulting to white." << std::endl;
  }
  return ignition::math::Color::White;
}

IGNITION_ADD_PLUGIN(ParticleEmitter,
                    ignition::gazebo::System,
                    ParticleEmitter::ISystemConfigure,
                    ParticleEmitter::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParticleEmitter,
                          "ignition::gazebo::systems::ParticleEmitter")
