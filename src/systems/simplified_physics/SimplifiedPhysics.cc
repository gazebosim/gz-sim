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

#include <chrono>
#include <cmath>
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
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/FrameData.hh>
#include <sdf/Element.hh>

#include "../physics/CommandClearer.hh"
#include "../physics/FrameDataWriter.hh"
#include "../physics/OwnerFilter.hh"
#include "PhysicsMode.hh"
#include "PlanarMotion.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
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
}

/// \brief A link of a model that the system simulates.
struct LinkState
{
  /// \brief Pose of the link in the model frame.
  math::Pose3d pose;

  /// \brief Frame data from the last write.
  RigidFrame frame;
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

  /// \brief Warn once per model about commands that this mode does not
  /// follow yet: joint velocity commands and link velocity commands.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void WarnUnsupportedCommands(const EntityComponentManager &_ecm);

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
  /// \return True if frame data was written for the model.
  public: bool Step(EntityComponentManager &_ecm, const Entity &_model,
              OwnedModel &_owned, double _dt);

  /// \brief Models that the system simulates, ordered by entity so that the
  /// order of the steps does not depend on hashing.
  public: std::map<Entity, OwnedModel> models;

  /// \brief State of the links of all models.
  public: std::unordered_map<Entity, LinkState> links;

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

  bool wroteFrameData{false};
  for (auto &[model, owned] : this->dataPtr->models)
  {
    if (this->dataPtr->Step(_ecm, model, owned, dt))
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
  // of the model are zeroed and kept: a system that writes them every step
  // then overwrites a component instead of creating one, which is much
  // cheaper, and a zero command is no motion.
  this->dataPtr->WarnUnsupportedCommands(_ecm);
  physics_system::ClearCommands(_ecm, this->dataPtr->OwnedFilter(), true);
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
        .value_or(std::to_string(model)) << "] gets a joint or link velocity "
        << "command, which physics mode [" << ModeName(
        PhysicsMode::PlanarKinematic) << "] ignores. The mode only follows "
        << "the velocity commands of the whole model (VelocityControl), and "
        << "wheel joints are not simulated yet, so a wheel controller does "
        << "not move this model." << std::endl;
  };

  _ecm.Each<components::JointVelocityCmd>(
      [&](const Entity &_entity, const components::JointVelocityCmd *_cmd)
          -> bool
      {
        if (!_cmd->Data().empty() &&
            this->ownedEntities.find(_entity) != this->ownedEntities.end())
        {
          warn(_entity);
        }
        return true;
      });

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

  const auto descendants = _ecm.Descendants(_model);
  owned.entities.assign(descendants.begin(), descendants.end());
  this->ownedEntities.insert(descendants.begin(), descendants.end());

  this->models.emplace(_model, std::move(owned));
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::RemoveModel(const Entity &_model)
{
  auto it = this->models.find(_model);
  if (it == this->models.end())
    return;

  for (const auto &link : it->second.links)
    this->links.erase(link);
  for (const auto &entity : it->second.entities)
    this->ownedEntities.erase(entity);
  this->models.erase(it);
}

//////////////////////////////////////////////////
bool SimplifiedPhysicsPrivate::Step(EntityComponentManager &_ecm,
    const Entity &_model, OwnedModel &_owned, double _dt)
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

  // The commands are in the model frame, and they describe the canonical
  // link, which is what Physics gives the velocity to. The model origin moves
  // as a rigid body with that link: v_origin = v_link - w x pivot. Rotate to
  // the world, keep x, y and yaw, and express the linear velocity in the
  // heading frame, which is where IntegrateArc holds it constant.
  PlanarDelta delta;
  math::Vector3d velocity;
  double wz{0.0};
  if (_dt > 0.0)
  {
    const auto *linearCmd =
        _ecm.Component<components::LinearVelocityCmd>(_model);
    const auto *angularCmd =
        _ecm.Component<components::AngularVelocityCmd>(_model);
    if (linearCmd || angularCmd)
    {
      const math::Vector3d linearBody = linearCmd ?
          linearCmd->Data() : math::Vector3d::Zero;
      const math::Vector3d angularBody = angularCmd ?
          angularCmd->Data() : math::Vector3d::Zero;
      const math::Vector3d linear = pose.Rot().RotateVector(
          linearBody - angularBody.Cross(_owned.pivot));
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

  // Write the frame data again on the step after the model stopped, so that
  // readers see zero velocity.
  const bool write = moved || teleported || _owned.wasMoving ||
      _owned.needsFrameData;
  _owned.wasMoving = moved;
  _owned.needsFrameData = false;
  if (!write)
    return false;

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
