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

#include <gz/msgs/particle_emitter.pb.h>

#include <mutex>
#include <set>
#include <string>

#include <gz/utils/SuppressWarning.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
#include <gz/plugin/Register.hh>
IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
#include <gz/transport/Node.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParticleEmitter.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SourceFilePath.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <sdf/Material.hh>
#include "ParticleEmitter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::ParticleEmitterPrivate
{
  /// \brief Callback for receiving particle emitter commands.
  /// \param[in] _msg Particle emitter message.
  public: void OnCmd(const ignition::msgs::ParticleEmitter &_msg);

  /// \brief The particle emitter parsed from SDF.
  public: ignition::msgs::ParticleEmitter emitter;

  /// \brief The transport node.
  public: ignition::transport::Node node;

  /// \brief Particle emitter entity.
  public: Entity emitterEntity{kNullEntity};

  /// \brief The particle emitter command requested externally.
  public: ignition::msgs::ParticleEmitter userCmd;

  public: bool newDataReceived = false;

  /// \brief A mutex to protect the user command.
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
void ParticleEmitterPrivate::OnCmd(const msgs::ParticleEmitter &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->userCmd = _msg;
  this->newDataReceived = true;
}

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
  Model model = Model(_entity);

  if (!model.Valid(_ecm))
  {
    ignerr << "ParticleEmitter plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Create a particle emitter entity.
  this->dataPtr->emitterEntity = _ecm.CreateEntity();
  if (this->dataPtr->emitterEntity == kNullEntity)
  {
    ignerr << "Failed to create a particle emitter entity" << std::endl;
    return;
  }

  // allow_renaming
  bool allowRenaming = false;
  if (_sdf->HasElement("allow_renaming"))
    allowRenaming = _sdf->Get<bool>("allow_renaming");

  // Name.
  std::string name = "particle_emitter_entity_" +
      std::to_string(this->dataPtr->emitterEntity);
  if (_sdf->HasElement("emitter_name"))
  {
    std::set<std::string> emitterNames;
    std::string emitterName = _sdf->Get<std::string>("emitter_name");

    // check to see if name is already taken
    _ecm.Each<components::Name, components::ParticleEmitter>(
        [&emitterNames](const Entity &, const components::Name *_name,
                      const components::ParticleEmitter *)
        {
          emitterNames.insert(_name->Data());
          return true;
        });

    name = emitterName;

    // rename emitter if needed
    if (emitterNames.find(emitterName) != emitterNames.end())
    {
      if (!allowRenaming)
      {
        ignwarn << "Entity named [" << name
                << "] already exists and "
                << "[allow_renaming] is false. Entity not spawned."
                << std::endl;
        return;
      }
      int counter = 0;
      while (emitterNames.find(name) != emitterNames.end())
      {
        name = emitterName + "_" + std::to_string(++counter);
      }
      ignmsg << "Entity named [" << emitterName
             << "] already exists. Renaming it to " << name << std::endl;
    }
  }
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
  this->dataPtr->emitter.mutable_rate()->set_data(
      _sdf->Get<double>("rate", 10).first);

  // Duration.
  this->dataPtr->emitter.mutable_duration()->set_data(
      _sdf->Get<double>("duration", 0).first);

  // Emitting.
  this->dataPtr->emitter.mutable_emitting()->set_data(
      _sdf->Get<bool>("emitting", false).first);

  // Particle size.
  size = ignition::math::Vector3d::One;
  if (_sdf->HasElement("particle_size"))
    size = _sdf->Get<ignition::math::Vector3d>("particle_size");
  ignition::msgs::Set(this->dataPtr->emitter.mutable_particle_size(), size);

  // Lifetime.
  this->dataPtr->emitter.mutable_lifetime()->set_data(
      _sdf->Get<double>("lifetime", 5).first);

  // Material.
  if (_sdf->HasElement("material"))
  {
    auto materialElem = _sdf->GetElementImpl("material");
    sdf::Material material;
    material.Load(materialElem);
    ignition::msgs::Material materialMsg = convert<msgs::Material>(material);
    this->dataPtr->emitter.mutable_material()->CopyFrom(materialMsg);
  }

  // Min velocity.
  this->dataPtr->emitter.mutable_min_velocity()->set_data(
    _sdf->Get<double>("min_velocity", 1).first);

  // Max velocity.
  this->dataPtr->emitter.mutable_max_velocity()->set_data(
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
  this->dataPtr->emitter.mutable_scale_rate()->set_data(
    _sdf->Get<double>("scale_rate", 1).first);

  // Color range image.
  if (_sdf->HasElement("color_range_image"))
  {
    auto modelPath = _ecm.ComponentData<components::SourceFilePath>(_entity);
    auto colorRangeImagePath = _sdf->Get<std::string>("color_range_image");
    auto path = asFullPath(colorRangeImagePath, modelPath.value());

    const std::string absolutePath =
        common::SystemPaths::LocateLocalFile(path, gazebo::resourcePaths());

    this->dataPtr->emitter.mutable_color_range_image()->set_data(
        absolutePath);
  }

  // particle scatter ratio
  const std::string scatterRatioKey = "particle_scatter_ratio";
  if (_sdf->HasElement(scatterRatioKey))
  {
    // todo(anyone) add particle_scatter_ratio field in next release of ign-msgs
    auto data = this->dataPtr->emitter.mutable_header()->add_data();
    data->set_key(scatterRatioKey);
    std::string *value = data->add_value();
    *value = _sdf->Get<std::string>(scatterRatioKey);
  }

  igndbg << "Loading particle emitter:" << std::endl
         << this->dataPtr->emitter.DebugString() << std::endl;

  // Create components.
  SdfEntityCreator sdfEntityCreator(_ecm, _eventMgr);
  sdfEntityCreator.SetParent(this->dataPtr->emitterEntity, _entity);

  _ecm.CreateComponent(this->dataPtr->emitterEntity,
    components::Name(this->dataPtr->emitter.name()));

  _ecm.CreateComponent(this->dataPtr->emitterEntity,
    components::ParticleEmitter(this->dataPtr->emitter));

  _ecm.CreateComponent(this->dataPtr->emitterEntity, components::Pose(pose));

  // Advertise the topic to receive particle emitter commands.
  const std::string kDefaultTopic =
    "/model/" + model.Name(_ecm) + "/particle_emitter/" + name;
  std::string topic = _sdf->Get<std::string>("topic", kDefaultTopic).first;
  if (!this->dataPtr->node.Subscribe(
         topic, &ParticleEmitterPrivate::OnCmd, this->dataPtr.get()))
  {
    ignerr << "Error subscribing to topic [" << topic << "]. "
        << "Particle emitter will not receive updates." << std::endl;
    return;
  }
  igndbg << "Subscribed to " << topic << " for receiving particle emitter "
         << "updates" << std::endl;
}

//////////////////////////////////////////////////
void ParticleEmitter::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ParticleEmitter::PreUpdate");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->newDataReceived)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->newDataReceived = false;

  // Create component.
  auto emitterComp = _ecm.Component<components::ParticleEmitterCmd>(
      this->dataPtr->emitterEntity);
  if (!emitterComp)
  {
    _ecm.CreateComponent(
        this->dataPtr->emitterEntity,
        components::ParticleEmitterCmd(this->dataPtr->userCmd));
  }
  else
  {
    emitterComp->Data() = this->dataPtr->userCmd;

    // Note: we process the cmd component in RenderUtil but if there is only
    // rendering on the gui side, it will not be able to remove the cmd
    // component from the ECM. It seems like adding OneTimeChange here will make
    // sure the cmd component is found again in Each call on GUI side.
    // todo(anyone) find a better way to process this cmd component in
    // RenderUtil.cc
    _ecm.SetChanged(this->dataPtr->emitterEntity,
        components::ParticleEmitterCmd::typeId,
        ComponentState::OneTimeChange);
  }

  igndbg << "New ParticleEmitterCmd component created" << std::endl;
}

IGNITION_ADD_PLUGIN(ParticleEmitter,
                    ignition::gazebo::System,
                    ParticleEmitter::ISystemConfigure,
                    ParticleEmitter::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParticleEmitter,
                          "ignition::gazebo::systems::ParticleEmitter")
