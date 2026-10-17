/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "SimplifiedPhysics.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/FrameData.hh>
#include <sdf/Element.hh>

#include "../physics/CommandClearer.hh"
#include "../physics/FrameDataWriter.hh"
#include "../physics/OwnerFilter.hh"
#include "ModelParams.hh"
#include "OdometryNoise.hh"
#include "PhysicsMode.hh"
#include "PlanarMotion.hh"
#include "WheelInference.hh"
#include "WheelKinematics.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/SupportedPhysicsFeatures.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace systems::simplified_physics;
using systems::physics_system::OwnerFilter;

namespace
{
/// \brief Convert frame data to the type that the frame data writers take.
/// \param[in] _frame Frame data in the world frame.
/// \return The same data as gz-physics frame data.
physics::FrameData3d ToFrameData(const RigidFrame &_frame)
{
  physics::FrameData3d data;
  data.pose = math::eigen3::convert(_frame.pose);
  data.linearVelocity = math::eigen3::convert(_frame.linearVelocity);
  data.angularVelocity = math::eigen3::convert(_frame.angularVelocity);
  data.linearAcceleration = math::eigen3::convert(_frame.linearAcceleration);
  data.angularAcceleration = math::eigen3::convert(_frame.angularAcceleration);
  return data;
}

/// \brief True if the pose can be applied to a model.
/// \param[in] _pose A pose from a command.
/// \return True if every component is finite and the rotation is valid.
bool IsValidPose(const math::Pose3d &_pose)
{
  return _pose.Pos().IsFinite() && _pose.Rot().IsFinite() &&
      _pose.Rot() != math::Quaterniond::Zero;
}

/// \brief True if a number is not exactly zero. Written with fpclassify
/// because the project builds with -Wfloat-equal.
/// \param[in] _value Any number.
/// \return False for 0 and -0.
bool NonZero(double _value)
{
  return std::fpclassify(_value) != FP_ZERO;
}

/// \brief True if a link, sensor or collision has a frame data component.
/// Other systems create these when they need the data, and the physics
/// providers only fill components that exist. Without a reader, computing and
/// writing the frames of every link is wasted work.
/// \param[in] _ecm Constant reference to the ECM.
/// \return True if one of the nine frame data components is in use.
bool FrameDataWanted(const EntityComponentManager &_ecm)
{
  // The wind entity of the world carries a WorldLinearVelocity in every
  // simulation. It is no reader, so that type is checked below.
  const ComponentTypeId types[] = {
      components::WorldPose::typeId,
      components::WorldAngularVelocity::typeId,
      components::WorldLinearAcceleration::typeId,
      components::WorldAngularAcceleration::typeId,
      components::LinearVelocity::typeId,
      components::AngularVelocity::typeId,
      components::LinearAcceleration::typeId,
      components::AngularAcceleration::typeId};
  if (std::any_of(std::begin(types), std::end(types),
      [&_ecm](const ComponentTypeId &_type)
      {
        return _ecm.HasComponentType(_type);
      }))
  {
    return true;
  }

  bool wanted{false};
  if (_ecm.HasComponentType(components::WorldLinearVelocity::typeId))
  {
    _ecm.Each<components::WorldLinearVelocity>(
        [&](const Entity &_entity, const components::WorldLinearVelocity *)
            -> bool
        {
          wanted = _ecm.Component<components::Wind>(_entity) == nullptr;
          return !wanted;
        });
  }
  return wanted;
}

/// \brief True if the system can move a top-level model.
/// \param[in] _ecm Constant reference to the ECM.
/// \param[in] _model Top-level model.
/// \return False for a static model, which nothing moves, and for a model
/// without a pose.
bool CanSimulate(const EntityComponentManager &_ecm, const Entity &_model)
{
  const auto isStatic = _ecm.ComponentData<components::Static>(_model);
  return !(isStatic.has_value() && *isStatic) &&
      _ecm.Component<components::Pose>(_model) != nullptr;
}

/// \brief Read the parameters of the model's SimplifiedPhysicsModel plugin.
/// \param[in] _ecm Constant reference to the ECM.
/// \param[in] _model Top-level model.
/// \param[in] _name Name of the model, for messages.
/// \return The parameters, or the defaults if the model has no such plugin.
ModelParams LoadParams(const EntityComponentManager &_ecm,
    const Entity &_model, const std::string &_name)
{
  const auto *modelSdf = _ecm.Component<components::ModelSdf>(_model);
  if (!modelSdf)
    return {};

  for (const auto &plugin : modelSdf->Data().Plugins())
  {
    if (plugin.Name() == kModelPluginName)
      return ParseModelParams(plugin.Contents(), _name, false);
  }
  return {};
}

/// \brief Write the position and velocity of one joint to the components
/// that other systems created for it.
///
/// Like Physics, the system fills only components that exist. It marks the
/// position as changed and leaves the velocity unmarked.
///
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _joint Joint entity.
/// \param[in] _position Position in rad or m.
/// \param[in] _velocity Velocity in rad/s or m/s.
/// \param[in] _writePosition False to leave the position alone, when it did
/// not change and the component is already filled.
/// \param[in] _writeVelocity False to leave the velocity alone, when it did
/// not change.
void WriteJointState(EntityComponentManager &_ecm, const Entity &_joint,
    double _position, double _velocity, bool _writePosition,
    bool _writeVelocity)
{
  if (auto *position = _ecm.Component<components::JointPosition>(_joint))
  {
    auto &data = position->Data();
    if (_writePosition || data.size() != 1)
    {
      data.assign(1, _position);
      _ecm.SetChanged(_joint, components::JointPosition::typeId,
          ComponentState::PeriodicChange);
    }
  }
  if (!_writeVelocity)
    return;
  if (auto *velocity = _ecm.Component<components::JointVelocity>(_joint))
    velocity->Data().assign(1, _velocity);
}
}

/// \brief A link of a model that the system simulates.
struct LinkState
{
  /// \brief Pose of the link in the model frame.
  math::Pose3d pose;

  /// \brief Frame data from the last write.
  RigidFrame frame;
};

/// \brief A wheel of a model that the system simulates.
struct WheelState
{
  /// \brief Joint entity.
  Entity joint{kNullEntity};

  /// \brief Position that the system reports, in rad. It includes the
  /// odometry noise, if any.
  double position{0.0};

  /// \brief Velocity from the last write, in rad/s.
  double velocity{0.0};

  /// \brief True if the next step has to write the position, for example
  /// after a reset.
  bool dirty{true};

  /// \brief Error that the wheel adds to its travel, if the model asks for
  /// odometry noise.
  std::optional<OdometryNoise> noise;
};

/// \brief A one degree of freedom joint of a model that is no wheel.
struct JointState
{
  /// \brief Joint entity.
  Entity joint{kNullEntity};

  /// \brief Position limits.
  double lower{0.0};
  double upper{0.0};

  /// \brief Position, in rad or m.
  double position{0.0};

  /// \brief Velocity from the last write.
  double velocity{0.0};

  /// \brief True if the next step has to write the position.
  bool dirty{true};
};

/// \brief A model that the system simulates.
struct OwnedModel
{
  /// \brief Links of the model, including links of nested models.
  std::vector<Entity> links;

  /// \brief The model and all its descendants, as of the last time that new
  /// entities appeared.
  std::vector<Entity> entities;

  /// \brief Position of the canonical link in the model frame. Velocity
  /// commands turn about this point, as they do on Physics.
  math::Vector3d pivot;

  /// \brief The WorldPoseCmd that the system applied and has not removed yet.
  std::optional<math::Pose3d> appliedPoseCmd;

  /// \brief True if the last step moved the model. The step after it writes
  /// the frame data once more, so that the velocities read zero.
  bool wasMoving{false};

  /// \brief True until the frame data has been written once.
  bool needsFrameData{true};

  /// \brief True after the system warned about an invalid command.
  bool warnedInvalidCmd{false};

  /// \brief True after the system warned about a command it does not follow.
  bool warnedUnsupportedCmd{false};

  /// \brief True after the system warned that the model gets model velocity
  /// commands and wheel commands in the same step.
  bool warnedCommandConflict{false};

  /// \brief The wheels as a drive, if the model has any.
  std::optional<WheelDrive> drive;

  /// \brief State of the wheels, in the order of the drive.
  std::vector<WheelState> wheels;

  /// \brief Scratch space for the commanded rates of the wheels.
  std::vector<double> wheelRates;

  /// \brief Joints that are no wheels and that the mode can follow.
  std::vector<JointState> joints;

  /// \brief Joints that get a command the mode cannot follow.
  std::vector<Entity> unsupportedJoints;

  /// \brief True after the system warned about a command on a joint that it
  /// cannot follow.
  bool warnedUnsupportedJointCmd{false};

  /// \brief True once the first step has filled the joint components that
  /// exist.
  bool jointsSettled{false};
};

/// \brief Where a joint is kept.
struct JointRef
{
  /// \brief The top-level model.
  Entity model{kNullEntity};

  /// \brief True for a wheel, false for another joint.
  bool wheel{false};

  /// \brief Index in OwnedModel::wheels or OwnedModel::joints.
  std::size_t index{0};
};

/// \brief Private SimplifiedPhysics data class.
class gz::sim::systems::SimplifiedPhysicsPrivate
{
  /// \brief Start to simulate a model if the system owns it.
  /// \param[in] _ecm Mutable reference to the ECM.
  /// \param[in] _owner Tells whether this system owns a model. It is resolved
  /// here, once per model, and recorded in ownedEntities.
  /// \param[in] _model Candidate model.
  public: void AddModel(EntityComponentManager &_ecm,
              const OwnerFilter &_owner, const Entity &_model);

  /// \brief Settle a top-level model that the PhysicsOwner components name as
  /// this system's. If the system can simulate it, make sure that it lists
  /// the features of the mode, because the model may be owned through the
  /// world's default and then no model plugin wrote them. If it cannot (a
  /// static model, for example), give the model back to the default Physics
  /// system with an empty PhysicsOwner on the model, which takes precedence
  /// over the world's. Physics then simulates it, and it has no
  /// SupportedPhysicsFeatures, so it supports everything.
  /// \param[in] _ecm Mutable reference to the ECM.
  /// \param[in] _model Top-level model.
  /// \return True if the system simulates the model.
  public: bool ClaimModel(EntityComponentManager &_ecm, const Entity &_model);

  /// \brief Record the entities under the models that gained entities, so
  /// that entities created later are covered.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void RefreshOwnedEntities(const EntityComponentManager &_ecm);

  /// \brief Warn once per model about link velocity commands, which this
  /// mode does not follow.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void WarnUnsupportedCommands(const EntityComponentManager &_ecm);

  /// \brief Take the positions that JointPositionReset sets for wheels and
  /// other joints that the system follows. The reset itself is cleared with
  /// the other commands.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void ApplyJointResets(const EntityComponentManager &_ecm);

  /// \brief Find the wheels and other joints of a model and set up their
  /// state.
  /// \param[in] _ecm Constant reference to the ECM.
  /// \param[in] _model Top-level model.
  /// \param[in] _modelPose World pose of the model.
  /// \param[in] _canonicalLink Canonical link of the model.
  /// \param[in,out] _owned State of the model.
  public: void AddJoints(const EntityComponentManager &_ecm,
              const Entity &_model, const math::Pose3d &_modelPose,
              const Entity &_canonicalLink, OwnedModel &_owned);

  /// \brief Read the wheel commands of a model.
  /// \param[in] _ecm Constant reference to the ECM.
  /// \param[in,out] _owned State of the model. The commanded rates go to
  /// its scratch space.
  /// \return The set of wheels with a command, as a bit mask over the wheels.
  public: std::uint64_t ReadWheelCommands(const EntityComponentManager &_ecm,
              OwnedModel &_owned) const;

  /// \brief Move the joint positions of a model along with the achieved
  /// motion of the model and the commands of the other joints.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in,out] _owned State of the model.
  /// \param[in] _dt Time step in seconds. Zero when paused.
  /// \param[in] _achieved Planar twist that the model achieved in this step,
  /// in the model frame. Zero if it did not move.
  public: void StepJoints(EntityComponentManager &_ecm, OwnedModel &_owned,
              double _dt, const PlanarTwist &_achieved);

  /// \brief Make the filter for the helpers that clear commands every step.
  /// It accepts the entities under the models that this system simulates.
  /// The filter reads ownedEntities and is valid while this object lives.
  /// \return A hash lookup per entity.
  public: OwnerFilter OwnedFilter() const
  {
    return OwnerFilter([this](const Entity &_entity)
        {
          return this->ownedEntities.find(_entity) !=
              this->ownedEntities.end();
        });
  }

  /// \brief Stop simulating a model.
  /// \param[in] _model Model entity.
  public: void RemoveModel(const Entity &_model);

  /// \brief Find the models that the system owns.
  /// \param[in] _ecm Mutable reference to the ECM.
  public: void UpdateModels(EntityComponentManager &_ecm);

  /// \brief Apply the commands of one model and move it.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _model Model entity.
  /// \param[in,out] _owned State of the model.
  /// \param[in] _dt Time step in seconds. Zero when paused.
  /// \param[in] _frames True if some system reads frame data, see
  /// FrameDataWanted().
  /// \return True if frame data was written for the model.
  public: bool Step(EntityComponentManager &_ecm, const Entity &_model,
              OwnedModel &_owned, double _dt, bool _frames);

  /// \brief Models that the system simulates, ordered by entity so that the
  /// order of the steps does not depend on hashing.
  public: std::map<Entity, OwnedModel> models;

  /// \brief State of the links of all models.
  public: std::unordered_map<Entity, LinkState> links;

  /// \brief The wheels and joints that the system follows, by joint entity.
  public: std::unordered_map<Entity, JointRef> jointRefs;

  /// \brief The models that the system simulates and everything under them.
  /// This is the only record of ownership that the per-step code reads.
  /// Ownership is decided once, when a model is added.
  public: std::unordered_set<Entity> ownedEntities;

  /// \brief True until the first Update has looked at every model. A system
  /// that is loaded after its models exist does not see them as new.
  public: bool needsScan{true};

  /// \brief False if the plugin is not attached to a world. Such an instance
  /// does nothing, so two instances never work on the same models.
  public: bool active{true};
};

//////////////////////////////////////////////////
SimplifiedPhysics::SimplifiedPhysics()
  : System(), dataPtr(std::make_unique<SimplifiedPhysicsPrivate>())
{
}

//////////////////////////////////////////////////
SimplifiedPhysics::~SimplifiedPhysics() = default;

//////////////////////////////////////////////////
void SimplifiedPhysics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  if (!_ecm.Component<components::World>(_entity))
  {
    gzerr << "SimplifiedPhysics must be attached to a world, but it is "
          << "attached to entity [" << _entity << "]. The plugin is ignored."
          << std::endl;
    this->dataPtr->active = false;
    return;
  }

  // Models that the world's PhysicsOwner gives to this system need their
  // features before the system manager activates the plugins on them, which
  // happens before the first Update. This only works if the world's
  // PhysicsOwner exists when this plugin is configured. Models that come
  // later, or a world owner that is set later, are settled in Update.
  _ecm.Each<components::Model>(
      [&](const Entity &_model, const components::Model *) -> bool
      {
        if (topLevelModel(_model, _ecm) == _model &&
            isOwnedBy(_model, _ecm, kProviderName, false))
        {
          this->dataPtr->ClaimModel(_ecm, _model);
        }
        return true;
      });
}

//////////////////////////////////////////////////
System::PriorityType SimplifiedPhysics::ConfigurePriority()
{
  // Run before Physics, so that the commands of the models that this system
  // owns are read before anything else can touch them.
  return kPrePhysicsPriority;
}

//////////////////////////////////////////////////
void SimplifiedPhysics::Reset(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  // The ECM is back to the state of the SDF, with the PhysicsOwner of every
  // model as the model plugins set it. Look at the models again.
  this->dataPtr->models.clear();
  this->dataPtr->links.clear();
  this->dataPtr->jointRefs.clear();
  this->dataPtr->ownedEntities.clear();
  this->dataPtr->needsScan = true;
}

//////////////////////////////////////////////////
void SimplifiedPhysics::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SimplifiedPhysics::Update");

  if (!this->dataPtr->active)
    return;

  this->dataPtr->UpdateModels(_ecm);
  if (this->dataPtr->models.empty())
    return;

  // A paused world does not move, but it still takes a teleport.
  double dt = std::chrono::duration<double>(_info.dt).count();
  if (_info.paused || dt < 0.0)
    dt = 0.0;

  this->dataPtr->ApplyJointResets(_ecm);

  const bool frames = FrameDataWanted(_ecm);

  bool wroteFrameData{false};
  for (auto &[model, owned] : this->dataPtr->models)
  {
    if (this->dataPtr->Step(_ecm, model, owned, dt, frames))
      wroteFrameData = true;
  }

  // Sensors and collisions on the links that were written. This is the same
  // data that Physics writes for its own links.
  if (wroteFrameData)
  {
    GZ_PROFILE("Child frame data");
    physics_system::WriteChildFrameData(_ecm,
        [this](const Entity &_link, const math::Pose3d &_offset)
            -> std::optional<physics::FrameData3d>
        {
          auto it = this->dataPtr->links.find(_link);
          if (it == this->dataPtr->links.end())
            return std::nullopt;

          const auto &frame = it->second.frame;
          return ToFrameData(PlanarFrameAt(frame.pose, frame.linearVelocity,
              frame.angularVelocity.Z(), _offset));
        });
  }

  // The commands were consumed. Physics clears its own. The velocity commands
  // of the model and of its joints are zeroed and kept: a system that writes
  // them every step (VelocityControl, DiffDrive) then overwrites a component
  // instead of creating one, which is much cheaper, and a zero command is no
  // motion.
  this->dataPtr->WarnUnsupportedCommands(_ecm);
  physics_system::ClearCommands(_ecm, this->dataPtr->OwnedFilter(), true,
      true);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::UpdateModels(EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<components::Model>(
      [&](const Entity &_entity, const components::Model *) -> bool
      {
        this->RemoveModel(_entity);
        return true;
      });

  // Resolve ownership once per model. The check is free while no entity has
  // a PhysicsOwner, and then this system owns nothing.
  const auto owner =
      OwnerFilter::ForProvider(_ecm, kProviderName, false);

  if (this->needsScan)
  {
    this->needsScan = false;
    _ecm.Each<components::Model>(
        [&](const Entity &_entity, const components::Model *) -> bool
        {
          this->AddModel(_ecm, owner, _entity);
          return true;
        });
    return;
  }

  _ecm.EachNew<components::Model>(
      [&](const Entity &_entity, const components::Model *) -> bool
      {
        this->AddModel(_ecm, owner, _entity);
        return true;
      });

  // A link, joint or collision added to a model that is already owned is
  // owned too.
  if (!this->models.empty() && _ecm.HasNewEntities())
    this->RefreshOwnedEntities(_ecm);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::RefreshOwnedEntities(
    const EntityComponentManager &_ecm)
{
  // Only the models that gained entities. Every entity but the world has a
  // parent.
  std::unordered_set<Entity> changed;
  _ecm.EachNew<components::ParentEntity>(
      [&](const Entity &_entity, const components::ParentEntity *) -> bool
      {
        const auto top = topLevelModel(_entity, _ecm);
        if (this->models.find(top) != this->models.end())
          changed.insert(top);
        return true;
      });

  for (const auto &model : changed)
  {
    auto &owned = this->models.at(model);
    for (const auto &entity : owned.entities)
      this->ownedEntities.erase(entity);

    const auto descendants = _ecm.Descendants(model);
    owned.entities.assign(descendants.begin(), descendants.end());
    this->ownedEntities.insert(descendants.begin(), descendants.end());
  }
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::WarnUnsupportedCommands(
    const EntityComponentManager &_ecm)
{
  const auto warn = [&](const Entity &_entity)
  {
    const auto model = topLevelModel(_entity, _ecm);
    auto it = this->models.find(model);
    if (it == this->models.end() || it->second.warnedUnsupportedCmd)
      return;

    it->second.warnedUnsupportedCmd = true;
    gzwarn << "Model [" << _ecm.ComponentData<components::Name>(model)
        .value_or(std::to_string(model)) << "] gets a link velocity command, "
        << "which physics mode [" << ModeName(PhysicsMode::PlanarKinematic)
        << "] ignores. The mode follows the velocity commands of the whole "
        << "model (VelocityControl) and of wheel joints (DiffDrive), not "
        << "those of single links." << std::endl;
  };

  _ecm.Each<components::Link, components::LinearVelocityCmd>(
      [&](const Entity &_entity, const components::Link *,
          const components::LinearVelocityCmd *_cmd) -> bool
      {
        if (_cmd->Data() != math::Vector3d::Zero &&
            this->ownedEntities.find(_entity) != this->ownedEntities.end())
        {
          warn(_entity);
        }
        return true;
      });

  _ecm.Each<components::Link, components::AngularVelocityCmd>(
      [&](const Entity &_entity, const components::Link *,
          const components::AngularVelocityCmd *_cmd) -> bool
      {
        if (_cmd->Data() != math::Vector3d::Zero &&
            this->ownedEntities.find(_entity) != this->ownedEntities.end())
        {
          warn(_entity);
        }
        return true;
      });
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::ApplyJointResets(
    const EntityComponentManager &_ecm)
{
  if (this->jointRefs.empty())
    return;

  _ecm.Each<components::JointPositionReset>(
      [&](const Entity &_entity, const components::JointPositionReset *_reset)
          -> bool
      {
        const auto ref = this->jointRefs.find(_entity);
        if (ref == this->jointRefs.end() || _reset->Data().empty())
          return true;

        auto &owned = this->models.at(ref->second.model);
        if (ref->second.wheel)
        {
          auto &wheel = owned.wheels[ref->second.index];
          wheel.position = _reset->Data()[0];
          wheel.dirty = true;
        }
        else
        {
          auto &joint = owned.joints[ref->second.index];
          joint.position = AdvanceJoint(_reset->Data()[0], 0.0, 0.0,
              joint.lower, joint.upper);
          joint.dirty = true;
        }
        return true;
      });
}

//////////////////////////////////////////////////
bool SimplifiedPhysicsPrivate::ClaimModel(EntityComponentManager &_ecm,
    const Entity &_model)
{
  if (CanSimulate(_ecm, _model))
  {
    if (!_ecm.Component<components::SupportedPhysicsFeatures>(_model))
    {
      _ecm.SetComponentData<components::SupportedPhysicsFeatures>(_model,
          SupportedFeatures(PhysicsMode::PlanarKinematic));
    }
    return true;
  }

  // Nothing here can move the model. Leaving it with this system would leave
  // it with nobody, because Physics skips every model that another provider
  // owns.
  const bool explicitOwner =
      _ecm.Component<components::PhysicsOwner>(_model) != nullptr;
  const auto name =
      _ecm.ComponentData<components::Name>(_model).value_or("");
  const std::string message = "Model [" + name + "] is static or has no pose, "
      "so physics mode [" + ModeName(PhysicsMode::PlanarKinematic) +
      "] cannot move it. The model goes to the default physics system.";
  if (explicitOwner)
    gzwarn << message << std::endl;
  else
    gzdbg << message << std::endl;

  _ecm.SetComponentData<components::PhysicsOwner>(_model, "");
  return false;
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::AddModel(EntityComponentManager &_ecm,
    const OwnerFilter &_owner, const Entity &_model)
{
  if (this->models.find(_model) != this->models.end())
    return;

  if (!_owner.Owns(_model))
    return;

  // The model plugin refuses nested models, but the world default can still
  // reach them. Their pose is relative to the parent, and the parent decides.
  if (topLevelModel(_model, _ecm) != _model)
    return;

  if (!this->ClaimModel(_ecm, _model))
    return;

  const auto modelPose = worldPose(_model, _ecm);

  OwnedModel owned;
  std::vector<Entity> pending{_model};
  while (!pending.empty())
  {
    const Model model(pending.back());
    pending.pop_back();

    for (const auto &link : model.Links(_ecm))
    {
      owned.links.push_back(link);
      LinkState state;
      state.pose = modelPose.Inverse() * worldPose(link, _ecm);
      this->links[link] = state;
    }
    for (const auto &nested : model.Models(_ecm))
      pending.push_back(nested);
  }

  // Velocity commands turn about the canonical link, like on Physics.
  const auto canonical = Model(_model).CanonicalLink(_ecm);
  auto canonicalIt = this->links.find(canonical);
  if (canonicalIt != this->links.end())
    owned.pivot = canonicalIt->second.pose.Pos();

  this->AddJoints(_ecm, _model, modelPose, canonical, owned);

  const auto descendants = _ecm.Descendants(_model);
  owned.entities.assign(descendants.begin(), descendants.end());
  this->ownedEntities.insert(descendants.begin(), descendants.end());

  const auto it = this->models.emplace(_model, std::move(owned)).first;
  for (std::size_t i = 0; i < it->second.wheels.size(); ++i)
    this->jointRefs[it->second.wheels[i].joint] = {_model, true, i};
  for (std::size_t i = 0; i < it->second.joints.size(); ++i)
    this->jointRefs[it->second.joints[i].joint] = {_model, false, i};
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::AddJoints(const EntityComponentManager &_ecm,
    const Entity &_model, const math::Pose3d &_modelPose,
    const Entity &_canonicalLink, OwnedModel &_owned)
{
  const auto name = _ecm.ComponentData<components::Name>(_model)
      .value_or(std::to_string(_model));
  const auto params = LoadParams(_ecm, _model, name);
  auto roles = ClassifyJoints(_ecm, _model, _canonicalLink, _modelPose,
      params.wheels, name);

  if (!roles.wheels.empty())
  {
    std::vector<WheelGeometry> geometry;
    for (const auto &wheel : roles.wheels)
      geometry.push_back(wheel.geometry);
    _owned.drive.emplace(std::move(geometry));

    // The drive drops wheels beyond its limit.
    const auto count = _owned.drive->Size();
    const double track = _owned.drive->Track();
    for (std::size_t i = 0; i < count; ++i)
    {
      WheelState wheel;
      wheel.joint = roles.wheels[i].joint;
      if (params.odometryNoise > 0.0)
      {
        wheel.noise.emplace(NoiseSeed(math::Rand::Seed(),
            scopedName(wheel.joint, _ecm)), params.odometryNoise, track);
      }
      _owned.wheels.push_back(std::move(wheel));
    }
    _owned.wheelRates.assign(count, 0.0);
    gzdbg << "Model [" << name << "] has " << count << " wheel joint(s) for "
          << "physics mode [" << ModeName(PhysicsMode::PlanarKinematic)
          << "]." << std::endl;
  }
  else if (params.odometryNoise > 0.0)
  {
    gzwarn << "Model [" << name << "] asks for odometry_noise, but has no "
           << "wheel joints. The noise has no effect." << std::endl;
  }

  for (const auto &joint : roles.articulated)
  {
    JointState state;
    state.joint = joint.joint;
    state.lower = joint.lower;
    state.upper = joint.upper;
    _owned.joints.push_back(state);
  }
  _owned.unsupportedJoints = std::move(roles.unsupported);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::RemoveModel(const Entity &_model)
{
  auto it = this->models.find(_model);
  if (it == this->models.end())
    return;

  for (const auto &link : it->second.links)
    this->links.erase(link);
  for (const auto &wheel : it->second.wheels)
    this->jointRefs.erase(wheel.joint);
  for (const auto &joint : it->second.joints)
    this->jointRefs.erase(joint.joint);
  for (const auto &entity : it->second.entities)
    this->ownedEntities.erase(entity);
  this->models.erase(it);
}

//////////////////////////////////////////////////
bool SimplifiedPhysicsPrivate::Step(EntityComponentManager &_ecm,
    const Entity &_model, OwnedModel &_owned, double _dt, bool _frames)
{
  auto poseComp = _ecm.Component<components::Pose>(_model);
  if (!poseComp)
    return false;

  math::Pose3d pose = poseComp->Data();

  // A WorldPoseCmd stays in the ECM for one more step after the step that
  // applies it. Applying it a second time would undo one step of motion, so
  // the data is compared with what was applied.
  bool teleported{false};
  if (auto cmd = _ecm.Component<components::WorldPoseCmd>(_model))
  {
    if (_owned.appliedPoseCmd && *_owned.appliedPoseCmd == cmd->Data())
    {
      _ecm.RemoveComponent<components::WorldPoseCmd>(_model);
      _owned.appliedPoseCmd.reset();
    }
    else
    {
      _owned.appliedPoseCmd = cmd->Data();
      if (IsValidPose(cmd->Data()))
      {
        pose = cmd->Data();
        teleported = true;
      }
      else
      {
        gzerr << "Unable to set world pose. Invalid pose value: "
              << cmd->Data() << std::endl;
      }
    }
  }
  else
  {
    _owned.appliedPoseCmd.reset();
  }

  // Which command moves the model. A model velocity command (VelocityControl)
  // wins over wheel commands (DiffDrive), as the most direct statement of
  // the wanted motion; wheel commands then only drive the wheel joint
  // positions. Both are in the model frame.
  //
  // A model velocity command describes the canonical link, which is what
  // Physics gives it to. The model origin moves as a rigid body with that
  // link: v_origin = v_link - w x pivot. Wheel commands give the velocity of
  // the origin directly. Rotate to the world, keep x, y and yaw, and express
  // the linear velocity in the heading frame, which is where IntegrateArc
  // holds it constant.
  PlanarDelta delta;
  math::Vector3d velocity;
  double wz{0.0};
  PlanarTwist bodyTwist;
  if (_dt > 0.0)
  {
    math::Vector3d linearBody;
    math::Vector3d angularBody;
    bool commanded{false};

    const auto *linearCmd =
        _ecm.Component<components::LinearVelocityCmd>(_model);
    const auto *angularCmd =
        _ecm.Component<components::AngularVelocityCmd>(_model);
    if (linearCmd || angularCmd)
    {
      const math::Vector3d linearLink = linearCmd ?
          linearCmd->Data() : math::Vector3d::Zero;
      angularBody = angularCmd ? angularCmd->Data() : math::Vector3d::Zero;
      linearBody = linearLink - angularBody.Cross(_owned.pivot);
      commanded = linearLink != math::Vector3d::Zero ||
          angularBody != math::Vector3d::Zero;
    }

    if (_owned.drive)
    {
      const auto driven = this->ReadWheelCommands(_ecm, _owned);
      if (commanded)
      {
        if (!_owned.warnedCommandConflict &&
            std::any_of(_owned.wheelRates.begin(), _owned.wheelRates.end(),
                NonZero))
        {
          _owned.warnedCommandConflict = true;
          gzwarn << "Model [" << _ecm.ComponentData<components::Name>(_model)
              .value_or(std::to_string(_model)) << "] gets a model velocity "
              << "command and wheel commands in the same step. The model "
              << "velocity command moves the model and the wheels follow."
              << std::endl;
        }
      }
      else if (driven != 0)
      {
        const auto twist = _owned.drive->TwistFromRates(
            _owned.wheelRates.data(), driven);
        linearBody.Set(twist.vx, twist.vy, 0.0);
        angularBody.Set(0.0, 0.0, twist.wz);
        commanded = true;
      }
    }

    if (commanded)
    {
      const math::Vector3d linear = pose.Rot().RotateVector(linearBody);
      const math::Vector3d angular = pose.Rot().RotateVector(angularBody);

      if (linear.IsFinite() && angular.IsFinite())
      {
        const double yaw = pose.Rot().Yaw();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        wz = angular.Z();
        delta = IntegrateArc(yaw, c * linear.X() + s * linear.Y(),
            -s * linear.X() + c * linear.Y(), wz, _dt);

        // The velocity at the end of the step, after the heading turned.
        const double c2 = std::cos(delta.yaw);
        const double s2 = std::sin(delta.yaw);
        velocity.Set(c2 * linear.X() - s2 * linear.Y(),
            s2 * linear.X() + c2 * linear.Y(), 0.0);
        bodyTwist = {linearBody.X(), linearBody.Y(), angularBody.Z()};
      }
      else if (!_owned.warnedInvalidCmd)
      {
        _owned.warnedInvalidCmd = true;
        gzwarn << "Ignoring a velocity command that is not finite, for "
               << "entity [" << _model << "]." << std::endl;
      }
    }
  }

  const bool moved = !delta.IsZero();
  if (moved)
  {
    pose.Pos().X() += delta.x;
    pose.Pos().Y() += delta.y;
    // Rotating about the world z axis leaves roll and pitch as they were.
    pose.Rot() = math::Quaterniond(0.0, 0.0, delta.yaw) * pose.Rot();
    pose.Rot().Normalize();
  }

  if (moved || teleported)
  {
    *poseComp = components::Pose(pose);
    _ecm.SetChanged(_model, components::Pose::typeId,
        teleported ? ComponentState::OneTimeChange :
                     ComponentState::PeriodicChange);
  }

  // The wheels follow the motion the model achieved, not the commands.
  if (!_owned.wheels.empty() || !_owned.joints.empty() ||
      !_owned.unsupportedJoints.empty())
  {
    this->StepJoints(_ecm, _owned, _dt, moved ? bodyTwist : PlanarTwist());
  }

  // Write the frame data again on the step after the model stopped, so that
  // readers see zero velocity. Without a reader nothing is computed, and the
  // first write waits for the first reader.
  const bool write = _frames && (moved || teleported || _owned.wasMoving ||
      _owned.needsFrameData);
  _owned.wasMoving = moved;
  if (!write)
    return false;
  _owned.needsFrameData = false;

  if (!moved)
  {
    velocity = math::Vector3d::Zero;
    wz = 0.0;
  }

  for (const auto &link : _owned.links)
  {
    auto &state = this->links[link];
    state.frame = PlanarFrameAt(pose, velocity, wz, state.pose);
    physics_system::WriteLinkFrameData(_ecm, link, ToFrameData(state.frame));
  }
  return true;
}

//////////////////////////////////////////////////
std::uint64_t SimplifiedPhysicsPrivate::ReadWheelCommands(
    const EntityComponentManager &_ecm, OwnedModel &_owned) const
{
  std::uint64_t driven{0};
  for (std::size_t i = 0; i < _owned.wheels.size(); ++i)
  {
    double rate{0.0};
    const auto *cmd =
        _ecm.Component<components::JointVelocityCmd>(_owned.wheels[i].joint);
    if (cmd && !cmd->Data().empty() && std::isfinite(cmd->Data()[0]))
    {
      rate = cmd->Data()[0];
      driven |= std::uint64_t{1} << i;
    }
    _owned.wheelRates[i] = rate;
  }
  return driven;
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::StepJoints(EntityComponentManager &_ecm,
    OwnedModel &_owned, double _dt, const PlanarTwist &_achieved)
{
  // The first step fills the components that exist, so that a reader such as
  // DiffDrive finds a value in a robot that has not moved. After that only a
  // change is written.
  const bool settle = !_owned.jointsSettled;
  _owned.jointsSettled = true;

  const bool rolling = NonZero(_achieved.vx) || NonZero(_achieved.vy) ||
      NonZero(_achieved.wz);
  for (std::size_t i = 0; i < _owned.wheels.size(); ++i)
  {
    auto &wheel = _owned.wheels[i];

    // Travel of the contact point along the rolling direction, as an angle.
    double delta{0.0};
    if (rolling && _dt > 0.0)
    {
      delta = _owned.drive->RateFromTwist(i, _achieved) * _dt;
      if (wheel.noise)
      {
        const double radius = _owned.drive->Wheel(i).radius;
        delta = wheel.noise->Apply(delta * radius) / radius;
      }
    }

    const double velocity = _dt > 0.0 ? delta / _dt : 0.0;
    const bool moved = NonZero(delta);
    if (moved)
      wheel.position += delta;

    const bool velocityChanged = NonZero(velocity - wheel.velocity);
    if (moved || wheel.dirty || settle || velocityChanged)
    {
      WriteJointState(_ecm, wheel.joint, wheel.position, velocity,
          moved || wheel.dirty || settle,
          velocityChanged || wheel.dirty || settle);
    }
    wheel.velocity = velocity;
    wheel.dirty = false;
  }

  for (auto &joint : _owned.joints)
  {
    double rate{0.0};
    if (_dt > 0.0)
    {
      const auto *cmd =
          _ecm.Component<components::JointVelocityCmd>(joint.joint);
      if (cmd && !cmd->Data().empty())
        rate = cmd->Data()[0];
    }

    const double next =
        AdvanceJoint(joint.position, rate, _dt, joint.lower, joint.upper);
    const double velocity = _dt > 0.0 ? (next - joint.position) / _dt : 0.0;
    const bool moved = NonZero(next - joint.position);
    joint.position = next;

    const bool velocityChanged = NonZero(velocity - joint.velocity);
    if (moved || joint.dirty || settle || velocityChanged)
    {
      WriteJointState(_ecm, joint.joint, joint.position, velocity,
          moved || joint.dirty || settle,
          velocityChanged || joint.dirty || settle);
    }
    joint.velocity = velocity;
    joint.dirty = false;
  }

  if (_dt > 0.0 && !_owned.warnedUnsupportedJointCmd)
  {
    for (const auto &joint : _owned.unsupportedJoints)
    {
      const auto *cmd = _ecm.Component<components::JointVelocityCmd>(joint);
      if (!cmd || std::none_of(cmd->Data().begin(), cmd->Data().end(),
          NonZero))
      {
        continue;
      }

      _owned.warnedUnsupportedJointCmd = true;
      gzwarn << "Joint [" << scopedName(joint, _ecm) << "] has more than "
          << "one degree of freedom or is a screw or gearbox joint, which "
          << "physics mode [" << ModeName(PhysicsMode::PlanarKinematic)
          << "] does not follow. Its velocity command is ignored."
          << std::endl;
      break;
    }
  }
}
