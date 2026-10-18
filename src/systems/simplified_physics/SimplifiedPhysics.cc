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
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <sched.h>
#endif

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
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
#include "PlanarCollision.hh"
#include "PlanarMotion.hh"
#include "WheelInference.hh"
#include "WheelKinematics.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/ContactSensor.hh"
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

/// \brief CPUs that this process may run on. Counts the affinity mask on
/// Linux (taskset, cpusets, and the CPU limit that a container runtime sets
/// through them) and falls back to the hardware thread count elsewhere.
/// \return At least 1.
int AvailableCpus()
{
#if defined(__linux__)
  cpu_set_t set;
  CPU_ZERO(&set);
  if (sched_getaffinity(0, sizeof(set), &set) == 0)
    return std::max(1, CPU_COUNT(&set));
#endif
  return std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
}

/// \brief Threads for `<collision_threads>0` (automatic): half the CPUs that
/// the process may use, at most 4. At 500 robots in a warehouse, 2 threads save
/// about 60 percent of the step time that 8 threads save and 4 threads about
/// 85 percent. Half of the CPUs stay free for the rest of the simulation.
/// \return Between 1 and 4.
int AutomaticCollisionThreads()
{
  return std::clamp(AvailableCpus() / 2, 1, 4);
}

/// \brief True if two poses are bit-for-bit the same in position and
/// rotation. The collision world is told about a pose only when it differs
/// from the last one that it holds, so the test must not use the tolerance of
/// gz-math's operator==.
/// \param[in] _a A pose.
/// \param[in] _b Another pose.
/// \return True if no component differs.
bool SamePose(const math::Pose3d &_a, const math::Pose3d &_b)
{
  return !(NonZero(_a.Pos().X() - _b.Pos().X()) ||
      NonZero(_a.Pos().Y() - _b.Pos().Y()) ||
      NonZero(_a.Pos().Z() - _b.Pos().Z()) ||
      NonZero(_a.Rot().W() - _b.Rot().W()) ||
      NonZero(_a.Rot().X() - _b.Rot().X()) ||
      NonZero(_a.Rot().Y() - _b.Rot().Y()) ||
      NonZero(_a.Rot().Z() - _b.Rot().Z()));
}

/// \brief True if two rotations are bit-for-bit the same.
/// \param[in] _a A rotation.
/// \param[in] _b Another rotation.
/// \return True if no component differs.
bool SameRotation(const math::Quaterniond &_a, const math::Quaterniond &_b)
{
  return !(NonZero(_a.W() - _b.W()) || NonZero(_a.X() - _b.X()) ||
      NonZero(_a.Y() - _b.Y()) || NonZero(_a.Z() - _b.Z()));
}

/// \brief An angle in [-pi, pi].
/// \param[in] _angle Any angle in radians.
/// \return The same angle, turned by whole circles.
double WrapAngle(double _angle)
{
  if (std::abs(_angle) > GZ_PI)
    _angle = std::remainder(_angle, 2.0 * GZ_PI);
  return _angle;
}

/// \brief True if the entity has one of the nine frame data components.
/// Other systems create these when they need the data, and the physics
/// providers only fill components that exist. Without a reader, computing and
/// writing the frames of a model is wasted work.
/// \param[in] _ecm Constant reference to the ECM.
/// \param[in] _entity A link, a sensor, a collision or another entity.
/// \return True if the entity has one of them.
bool HasFrameComponent(const EntityComponentManager &_ecm,
    const Entity &_entity)
{
  const ComponentTypeId types[] = {
      components::WorldPose::typeId,
      components::WorldLinearVelocity::typeId,
      components::WorldAngularVelocity::typeId,
      components::WorldLinearAcceleration::typeId,
      components::WorldAngularAcceleration::typeId,
      components::LinearVelocity::typeId,
      components::AngularVelocity::typeId,
      components::LinearAcceleration::typeId,
      components::AngularAcceleration::typeId};
  return std::any_of(std::begin(types), std::end(types),
      [&](const ComponentTypeId &_type)
      {
        return _ecm.EntityHasComponentType(_entity, _type);
      });
}

/// \brief Call a function for every entity whose component of a type was
/// created or replaced in this step.
/// \tparam ComponentT Component type.
/// \tparam Fn Function of an entity.
/// \param[in] _ecm Constant reference to the ECM.
/// \param[in] _fn Called for each such entity.
template<typename ComponentT, typename Fn>
void EachCreated(const EntityComponentManager &_ecm, const Fn &_fn)
{
  if (!_ecm.HasComponentType(ComponentT::typeId))
    return;

  _ecm.Each<ComponentT>(
      [&](const Entity &_entity, const ComponentT *) -> bool
      {
        if (_ecm.ComponentState(_entity, ComponentT::typeId) ==
            ComponentState::OneTimeChange)
        {
          _fn(_entity);
        }
        return true;
      });
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

/// \brief True if the name is one of the names that the SimplifiedPhysicsModel
/// plugin is registered under: the alias without the version namespace and the
/// class name with it.
/// \param[in] _name The `name` attribute of a `<plugin>` element.
/// \return True for a SimplifiedPhysicsModel name.
bool IsModelPluginName(const std::string &_name)
{
  const std::string suffix{"::SimplifiedPhysicsModel"};
  return _name == kModelPluginName ||
      (_name.rfind("gz::sim::", 0) == 0 && _name.size() > suffix.size() &&
       _name.compare(_name.size() - suffix.size(), suffix.size(), suffix) == 0);
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
    if (IsModelPluginName(plugin.Name()))
      return ParseModelParams(plugin.Contents(), _name, false);
  }
  return {};
}

/// \brief True if a joint has a state component that has no value yet. A
/// system that needs the joint state, such as DiffDrive, creates the
/// component empty, in any step.
/// \param[in] _position The joint's position component, or null.
/// \param[in] _velocity The joint's velocity component, or null.
/// \return True if one of them exists and is empty.
bool HasEmptyState(const components::JointPosition *_position,
    const components::JointVelocity *_velocity)
{
  return (_position && _position->Data().size() != 1) ||
      (_velocity && _velocity->Data().size() != 1);
}

/// \brief Write the position and velocity of one joint to the components
/// that other systems created for it.
///
/// Like Physics, the system fills only components that exist. It marks the
/// position as changed and leaves the velocity unmarked. A component without
/// a value is always filled.
///
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _joint Joint entity.
/// \param[in,out] _positionComp The joint's position component, or null.
/// \param[in,out] _velocityComp The joint's velocity component, or null.
/// \param[in] _position Position in rad or m.
/// \param[in] _velocity Velocity in rad/s or m/s.
/// \param[in] _writePosition False to leave the position alone, when it did
/// not change and the component is already filled.
/// \param[in] _writeVelocity False to leave the velocity alone, when it did
/// not change.
void WriteJointState(EntityComponentManager &_ecm, const Entity &_joint,
    components::JointPosition *_positionComp,
    components::JointVelocity *_velocityComp,
    double _position, double _velocity, bool _writePosition,
    bool _writeVelocity)
{
  if (_positionComp)
  {
    auto &data = _positionComp->Data();
    if (_writePosition || data.size() != 1)
    {
      data.assign(1, _position);
      _ecm.SetChanged(_joint, components::JointPosition::typeId,
          ComponentState::PeriodicChange);
    }
  }
  if (_velocityComp)
  {
    auto &data = _velocityComp->Data();
    if (_writeVelocity || data.size() != 1)
      data.assign(1, _velocity);
  }
}
}

struct OwnedModel;

/// \brief A link of a model that the system simulates.
struct LinkState
{
  /// \brief Pose of the link in the model frame.
  math::Pose3d pose;

  /// \brief Frame data from the last write.
  RigidFrame frame;

  /// \brief The model that the link belongs to.
  const OwnedModel *owner{nullptr};
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

  /// \brief True for a joint that looks like a wheel but is held at its
  /// position, see JointRoles::held. It ignores velocity commands, and a
  /// non-zero one is reported.
  bool held{false};
};

/// \brief What a model wants to do in one step, before collisions. Plan
/// fills it and Finish reads it.
struct StepPlan
{
  /// \brief The model's Pose component, which stays valid between Plan and
  /// Finish because nothing creates or removes a Pose in between.
  components::Pose *poseComp{nullptr};

  /// \brief Pose at the start of the motion, after a teleport.
  math::Pose3d pose;

  /// \brief True if a WorldPoseCmd moved the model in this step.
  bool teleported{false};

  /// \brief Motion without obstacles.
  PlanarDelta delta;

  /// \brief Velocity of the model origin in the world frame at the end of the
  /// motion without obstacles.
  math::Vector3d velocity;

  /// \brief Yaw rate of that motion.
  double wz{0.0};

  /// \brief Twist in the model frame that gives the motion.
  PlanarTwist bodyTwist;

  /// \brief Index of the model's request for the collision world, or -1.
  int request{-1};
};

/// \brief A model that the system simulates.
struct OwnedModel
{
  /// \brief What the model wants to do in this step.
  StepPlan plan;

  /// \brief Reaction to obstacles (`collision_response`).
  PlanarResponse response{PlanarResponse::Slide};

  /// \brief True if the collision world holds footprints for the model. A
  /// model whose collisions all have a zero collide_bitmask, or none with a
  /// shape, moves through everything.
  bool collides{false};

  /// \brief The pose that the collision world holds for the model, as of the
  /// end of the last step. A different Pose component means that something
  /// else moved the model, and the world is told before the next motion.
  math::Pose3d worldPose;

  /// \brief Heading of the model for the rotation `yawRotation`. The yaw of
  /// a quaternion costs an `atan2` and more, and the system needs it for
  /// every moving model in every step, while the rotation changes only by the
  /// yaw that the system adds. Use YawOf and SetYaw.
  double yaw{0.0};
  math::Quaterniond yawRotation;
  bool yawKnown{false};

  /// \brief Heading of a rotation. Uses the stored one if the rotation is
  /// the one that it belongs to, bit for bit, and works it out otherwise (a
  /// teleport, or another system that wrote the pose).
  /// \param[in] _rotation The rotation of the model.
  /// \return The yaw of the rotation.
  double YawOf(const math::Quaterniond &_rotation)
  {
    if (!this->yawKnown || !SameRotation(_rotation, this->yawRotation))
      this->SetYaw(_rotation, _rotation.Yaw());
    return this->yaw;
  }

  /// \brief Store the heading of a rotation.
  /// \param[in] _rotation The rotation of the model.
  /// \param[in] _yaw Its yaw.
  void SetYaw(const math::Quaterniond &_rotation, double _yaw)
  {
    this->yawRotation = _rotation;
    this->yaw = _yaw;
    this->yawKnown = true;
  }

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

  /// \brief True if a link, sensor or collision of the model has a frame data
  /// component. The model computes and writes frame data only then. Once set
  /// it stays set.
  bool frameReaders{false};

  /// \brief True until the frame data has been written once, and again when
  /// a reader appears. The next step then writes it, also for a model that
  /// does not move.
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

  /// \brief True after the system warned about a velocity command on a held
  /// wheel.
  bool warnedHeldWheelCmd{false};

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

  /// \brief Read the commands of one model and work out the motion that it
  /// wants. Nothing moves yet: the collision world decides how far it gets.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _model Model entity.
  /// \param[in,out] _owned State of the model. Its plan is filled.
  /// \param[in] _dt Time step in seconds. Zero when paused.
  /// \return False if the model has no pose and cannot move.
  public: bool Plan(EntityComponentManager &_ecm, const Entity &_model,
              OwnedModel &_owned, double _dt);

  /// \brief Move one model by the motion that the collision world allowed,
  /// and update everything that follows the motion: the pose, the wheel and
  /// joint states and the frame data.
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _model Model entity.
  /// \param[in,out] _owned State of the model.
  /// \param[in] _dt Time step in seconds. Zero when paused.
  /// \param[in] _result What the collision world did with the model's
  /// request, or null if the model had none: it moves as it wanted.
  /// \return True if frame data was written for the model.
  public: bool Finish(EntityComponentManager &_ecm, const Entity &_model,
              OwnedModel &_owned, double _dt,
              const PlanarMotionResult *_result);

  /// \brief Note a model that this system does not move. It enters the
  /// collision world in FlushObstacles, once the system moves a model at all.
  /// A world that loads the system but has no planar model pays nothing for
  /// the footprints of its scenery.
  /// \param[in] _model Candidate model.
  public: void AddObstacle(const Entity &_model);

  /// \brief Put the noted models into the collision world if the system moves
  /// at least one model: scenery if nothing can move a model, else a body
  /// that other code moves, which is an obstacle that follows the model's
  /// Pose.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void FlushObstacles(const EntityComponentManager &_ecm);

  /// \brief Put one top-level model into the collision world.
  /// \param[in] _ecm Constant reference to the ECM.
  /// \param[in] _model Top-level model.
  public: void AddObstacleNow(const EntityComponentManager &_ecm,
              const Entity &_model);

  /// \brief Tell the collision world where the models that other systems move
  /// are now. They lag Physics by one step, because Physics writes their
  /// poses after this system ran.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void FollowForeignModels(const EntityComponentManager &_ecm);

  /// \brief Build the footprints of static models again that Physics moved
  /// on a WorldPoseCmd. Physics writes the new Pose after this system ran, so
  /// the footprints lag by one step, like those of the foreign models.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void FollowScenery(const EntityComponentManager &_ecm);

  /// \brief Look for frame data components that were created in this step
  /// under the models. Components created in a step carry a one-time change
  /// until the step ends, so this runs in PostUpdate, after every system had
  /// its turn.
  /// \param[in] _ecm Constant reference to the ECM.
  public: void FindNewReaders(const EntityComponentManager &_ecm);

  /// \brief Record that an entity has a new frame data component, so that its
  /// model writes frame data in the next step.
  /// \param[in] _ecm Constant reference to the ECM.
  /// \param[in] _entity The entity.
  public: void AddReader(const EntityComponentManager &_ecm,
              const Entity &_entity);

  /// \brief Models that the system simulates, ordered by entity so that the
  /// order of the steps does not depend on hashing.
  public: std::map<Entity, OwnedModel> models;

  /// \brief State of the links of all models.
  public: std::unordered_map<Entity, LinkState> links;

  /// \brief Footprints of the scenery, of the models that the system moves and
  /// of the bodies of other physics systems. Planar models are moved through
  /// it with one call per step.
  public: std::unique_ptr<PlanarCollisionWorld> collision{
              std::make_unique<PlanarCollisionWorld>()};

  /// \brief Settings of the collision world, from the plugin parameters. A
  /// world reset builds the world again with them.
  public: PlanarCollisionOptions collisionOptions;

  /// \brief Models that other systems move, with the pose the collision world
  /// holds for them. Their pose is read from the Pose component each step.
  public: std::unordered_map<Entity, math::Pose3d> foreign;

  /// \brief Static models in the collision world, with the pose their
  /// footprints were built at. A WorldPoseCmd on one of them (the GUI
  /// transform tool, the set_pose service) builds them again.
  public: std::unordered_map<Entity, math::Pose3d> scenery;

  /// \brief Models seen by AddModel that no planar model has needed yet.
  public: std::unordered_set<Entity> pendingObstacles;

  /// \brief Requests for the collision world in this step, ordered by the
  /// entity of the model, and the results.
  public: std::vector<PlanarMotionRequest> requests;
  public: std::vector<PlanarMotionResult> results;

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
    const std::shared_ptr<const sdf::Element> &_sdf,
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

  // Threads are opt-in, as the other parallel features of the server are.
  int threads = 1;
  if (_sdf && _sdf->HasElement("collision_threads"))
  {
    const int requested = _sdf->Get<int>("collision_threads");
    if (requested == 0)
    {
      threads = AutomaticCollisionThreads();
    }
    else
    {
      threads = std::clamp(requested, 1, kMaxPlanarCollisionThreads);
      if (threads != requested)
      {
        gzwarn << "SimplifiedPhysics: <collision_threads> must be 0 (automatic)"
               << " or between 1 and " << kMaxPlanarCollisionThreads
               << ", but it is [" << requested << "]. Using [" << threads
               << "]." << std::endl;
      }
    }
    const int cpus = AvailableCpus();
    if (threads > cpus)
    {
      gzwarn << "SimplifiedPhysics: <collision_threads> is [" << threads
             << "], more than the [" << cpus << "] CPUs that the server may "
             << "use." << std::endl;
    }
  }
  this->dataPtr->collisionOptions.threads = threads;
  this->dataPtr->collision = std::make_unique<PlanarCollisionWorld>(
      this->dataPtr->collisionOptions);

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
  this->dataPtr->foreign.clear();
  this->dataPtr->scenery.clear();
  this->dataPtr->pendingObstacles.clear();
  this->dataPtr->collision = std::make_unique<PlanarCollisionWorld>(
      this->dataPtr->collisionOptions);
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

  // Every model says what it wants to do, the collision world decides how
  // far each one gets, and then the models move. Models are ordered by entity,
  // so the requests do not depend on hashing.
  auto &requests = this->dataPtr->requests;
  requests.clear();
  for (auto &[model, owned] : this->dataPtr->models)
  {
    owned.plan.request = -1;
    if (!this->dataPtr->Plan(_ecm, model, owned, dt) || !owned.collides)
      continue;

    // A teleport, or any other change of the pose, moves the model in the
    // collision world before the motion starts from there.
    if (!SamePose(owned.plan.pose, owned.worldPose))
    {
      this->dataPtr->collision->SetPose(model, owned.plan.pose);
      owned.worldPose = owned.plan.pose;
    }

    if (!owned.plan.delta.IsZero())
    {
      owned.plan.request = static_cast<int>(requests.size());
      PlanarMotionRequest request;
      request.model = model;
      request.desired = owned.plan.delta;
      request.response = owned.response;
      requests.push_back(request);
    }
  }

  if (!this->dataPtr->foreign.empty())
    this->dataPtr->FollowForeignModels(_ecm);
  if (!this->dataPtr->scenery.empty())
    this->dataPtr->FollowScenery(_ecm);
  if (!requests.empty())
    this->dataPtr->collision->Move(requests, this->dataPtr->results);

  bool wroteFrameData{false};
  for (auto &[model, owned] : this->dataPtr->models)
  {
    const PlanarMotionResult *result{nullptr};
    if (owned.plan.request >= 0)
    {
      result = &this->dataPtr->results[
          static_cast<std::size_t>(owned.plan.request)];
    }
    if (this->dataPtr->Finish(_ecm, model, owned, dt, result))
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
          // A model without a reader has no frame components, but one that
          // was created outside the system phases may exist. Its cached frame
          // can be stale, because the model does not write while nobody reads.
          auto it = this->dataPtr->links.find(_link);
          if (it == this->dataPtr->links.end() || !it->second.owner ||
              !it->second.owner->frameReaders)
          {
            return std::nullopt;
          }

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
void SimplifiedPhysics::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("SimplifiedPhysics::PostUpdate");

  // Nothing to find unless the system has models and something was created in
  // this step.
  if (!this->dataPtr->active || this->dataPtr->models.empty() ||
      !_ecm.HasOneTimeComponentChanges())
  {
    return;
  }

  this->dataPtr->FindNewReaders(_ecm);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::FindNewReaders(
    const EntityComponentManager &_ecm)
{
  const auto add = [&](const Entity &_entity)
  {
    this->AddReader(_ecm, _entity);
  };
  EachCreated<components::WorldPose>(_ecm, add);
  EachCreated<components::WorldLinearVelocity>(_ecm, add);
  EachCreated<components::WorldAngularVelocity>(_ecm, add);
  EachCreated<components::WorldLinearAcceleration>(_ecm, add);
  EachCreated<components::WorldAngularAcceleration>(_ecm, add);
  EachCreated<components::LinearVelocity>(_ecm, add);
  EachCreated<components::AngularVelocity>(_ecm, add);
  EachCreated<components::LinearAcceleration>(_ecm, add);
  EachCreated<components::AngularAcceleration>(_ecm, add);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::AddReader(const EntityComponentManager &_ecm,
    const Entity &_entity)
{
  const auto it = this->models.find(topLevelModel(_entity, _ecm));
  if (it == this->models.end())
    return;

  // A model that has a reader already writes only when it moves, so a second
  // reader needs the write as well.
  it->second.frameReaders = true;
  it->second.needsFrameData = true;
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
    this->FlushObstacles(_ecm);
    return;
  }

  _ecm.EachNew<components::Model>(
      [&](const Entity &_entity, const components::Model *) -> bool
      {
        this->AddModel(_ecm, owner, _entity);
        return true;
      });
  this->FlushObstacles(_ecm);

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
  {
    this->AddObstacle(_model);
    return;
  }

  // The model plugin refuses nested models, but the world default can still
  // reach them. Their pose is relative to the parent, and the parent decides.
  if (topLevelModel(_model, _ecm) != _model)
    return;

  if (!this->ClaimModel(_ecm, _model))
  {
    this->AddObstacle(_model);
    return;
  }

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

  // The collision world reports no contacts yet, so a contact sensor on the
  // model never fires.
  const bool hasContactSensor = std::any_of(descendants.begin(),
      descendants.end(), [&](const Entity &_entity)
      {
        return _ecm.Component<components::ContactSensor>(_entity) != nullptr;
      });
  if (hasContactSensor)
  {
    const auto modelName = _ecm.ComponentData<components::Name>(_model)
        .value_or(std::to_string(_model));
    gzwarn << "Model [" << modelName
           << "] has a contact sensor, which reports nothing in physics mode ["
           << ModeName(PhysicsMode::PlanarKinematic) << "]." << std::endl;
  }

  owned.entities.assign(descendants.begin(), descendants.end());
  this->ownedEntities.insert(descendants.begin(), descendants.end());

  // Readers that exist already: components from the SDF or from systems that
  // configured before this one. Later ones are found in PostUpdate.
  owned.frameReaders = std::any_of(descendants.begin(), descendants.end(),
      [&](const Entity &_entity)
      {
        return HasFrameComponent(_ecm, _entity);
      });

  // The footprints of the model's collisions. A model whose collisions all
  // have a zero collide_bitmask, or none with a shape, is not part of the
  // collision world and moves through everything.
  owned.collides = this->collision->AddMovableModel(_ecm, _model);
  if (const auto *poseComp = _ecm.Component<components::Pose>(_model))
    owned.worldPose = poseComp->Data();

  const auto it = this->models.emplace(_model, std::move(owned)).first;
  for (const auto &link : it->second.links)
    this->links[link].owner = &it->second;
  for (std::size_t i = 0; i < it->second.wheels.size(); ++i)
    this->jointRefs[it->second.wheels[i].joint] = {_model, true, i};
  for (std::size_t i = 0; i < it->second.joints.size(); ++i)
    this->jointRefs[it->second.joints[i].joint] = {_model, false, i};
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::AddObstacle(const Entity &_model)
{
  this->pendingObstacles.insert(_model);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::FlushObstacles(
    const EntityComponentManager &_ecm)
{
  if (this->models.empty() || this->pendingObstacles.empty())
    return;

  // The order of insertion into the collision world decides ties between
  // obstacles that are hit at the same time, so add them in entity order.
  std::vector<Entity> pending(this->pendingObstacles.begin(),
      this->pendingObstacles.end());
  this->pendingObstacles.clear();
  std::sort(pending.begin(), pending.end());
  for (const auto &model : pending)
  {
    // Nested models are part of their top-level model's footprints.
    if (_ecm.Component<components::Model>(model) &&
        topLevelModel(model, _ecm) == model)
    {
      this->AddObstacleNow(_ecm, model);
    }
  }
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::AddObstacleNow(
    const EntityComponentManager &_ecm, const Entity &_model)
{
  const auto isStatic = _ecm.ComponentData<components::Static>(_model);
  const auto *poseComp = _ecm.Component<components::Pose>(_model);
  if ((isStatic.has_value() && *isStatic) || !poseComp)
  {
    // Nothing integrates its motion. Only a WorldPoseCmd moves it, which
    // FollowScenery watches for.
    if (this->collision->AddStaticModel(_ecm, _model) && poseComp)
      this->scenery[_model] = poseComp->Data();
    return;
  }

  // Another system moves this model, Physics for the most part. The planar
  // models see it as an obstacle of infinite mass at the pose of the Pose
  // component, and do not push it.
  if (this->collision->AddMovableModel(_ecm, _model))
    this->foreign[_model] = poseComp->Data();
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::FollowForeignModels(
    const EntityComponentManager &_ecm)
{
  for (auto &[model, known] : this->foreign)
  {
    const auto *poseComp = _ecm.Component<components::Pose>(model);
    if (!poseComp || SamePose(poseComp->Data(), known))
      continue;

    known = poseComp->Data();
    this->collision->SetPose(model, known);
  }
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::FollowScenery(
    const EntityComponentManager &_ecm)
{
  // The view is empty in nearly every step. A command stays in the ECM for
  // two steps, and Physics writes the Pose in the first of them, after this
  // system ran. The Pose differs from the known one in the second step.
  _ecm.Each<components::Model, components::WorldPoseCmd>(
      [&](const Entity &_model, const components::Model *,
          const components::WorldPoseCmd *) -> bool
      {
        auto it = this->scenery.find(_model);
        if (it == this->scenery.end())
          return true;
        const auto *poseComp = _ecm.Component<components::Pose>(_model);
        if (!poseComp || SamePose(poseComp->Data(), it->second))
          return true;

        it->second = poseComp->Data();
        this->collision->AddStaticModel(_ecm, _model);
        return true;
      });
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
  _owned.response = params.collisionResponse;

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
  // Wheels the mode cannot drive stay where they are. They are articulated
  // joints with no travel, so that their state components still get a value.
  for (const auto &joint : roles.held)
  {
    JointState state;
    state.joint = joint;
    state.held = true;
    _owned.joints.push_back(state);
  }
  _owned.unsupportedJoints = std::move(roles.unsupported);
}

//////////////////////////////////////////////////
void SimplifiedPhysicsPrivate::RemoveModel(const Entity &_model)
{
  // Scenery and bodies of other systems are in the collision world too.
  this->collision->RemoveModel(_model);
  this->foreign.erase(_model);
  this->scenery.erase(_model);
  this->pendingObstacles.erase(_model);

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
bool SimplifiedPhysicsPrivate::Plan(EntityComponentManager &_ecm,
    const Entity &_model, OwnedModel &_owned, double _dt)
{
  auto &plan = _owned.plan;
  plan = StepPlan();
  plan.poseComp = _ecm.Component<components::Pose>(_model);
  if (!plan.poseComp)
    return false;

  math::Pose3d pose = plan.poseComp->Data();

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
  // the linear velocity in the world frame. IntegrateArc turns the arc
  // about the heading, and the heading cancels (see the call below).
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
        // The arc is worked out in the heading frame and turned into the
        // world frame by the heading. The velocity was turned into the
        // heading frame by the same heading, and rotations about z commute
        // with the arc, so the heading cancels. Integrating the world
        // velocity at heading zero gives the same displacement without the
        // yaw of the pose.
        wz = angular.Z();
        delta = IntegrateArc(0.0, linear.X(), linear.Y(), wz, _dt);

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

  plan.pose = pose;
  plan.teleported = teleported;
  plan.delta = delta;
  plan.velocity = velocity;
  plan.wz = wz;
  plan.bodyTwist = bodyTwist;
  return true;
}

//////////////////////////////////////////////////
bool SimplifiedPhysicsPrivate::Finish(EntityComponentManager &_ecm,
    const Entity &_model, OwnedModel &_owned, double _dt,
    const PlanarMotionResult *_result)
{
  auto &plan = _owned.plan;
  if (!plan.poseComp)
    return false;

  math::Pose3d pose = plan.pose;
  const bool teleported = plan.teleported;
  PlanarDelta delta = plan.delta;
  math::Vector3d velocity = plan.velocity;
  double wz = plan.wz;
  PlanarTwist bodyTwist = plan.bodyTwist;

  // A cut motion is a displacement that no command asked for. The wheels and
  // the velocities that readers see follow the motion that happened, so the
  // twist is worked out again from it. A free motion is exactly the one that
  // was asked for, and keeps its command.
  if (_result && (_result->blocked || !_result->known))
  {
    delta = _result->achieved;
    const double yaw = _owned.YawOf(pose.Rot());
    bodyTwist = TwistFromDelta(yaw, delta, _dt);
    wz = bodyTwist.wz;
    const double heading = yaw + delta.yaw;
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    velocity.Set(c * bodyTwist.vx - s * bodyTwist.vy,
        s * bodyTwist.vx + c * bodyTwist.vy, 0.0);
  }

  const bool moved = !delta.IsZero();
  if (moved)
  {
    pose.Pos().X() += delta.x;
    pose.Pos().Y() += delta.y;
    if (NonZero(delta.yaw))
    {
      // Rotating about the world z axis leaves roll and pitch as they were,
      // and adds the turn to the yaw. The rotation is the quaternion of a
      // turn about z, so no Euler conversion is needed.
      const double yaw = _owned.YawOf(pose.Rot());
      pose.Rot() = math::Quaterniond(std::cos(0.5 * delta.yaw), 0.0, 0.0,
          std::sin(0.5 * delta.yaw)) * pose.Rot();
      pose.Rot().Normalize();
      _owned.SetYaw(pose.Rot(), WrapAngle(yaw + delta.yaw));
    }
  }

  if (moved || teleported)
  {
    *plan.poseComp = components::Pose(pose);
    _ecm.SetChanged(_model, components::Pose::typeId,
        teleported ? ComponentState::OneTimeChange :
                     ComponentState::PeriodicChange);
  }
  if (_owned.collides)
    _owned.worldPose = pose;

  // The wheels follow the motion the model achieved, not the commands.
  if (!_owned.wheels.empty() || !_owned.joints.empty() ||
      !_owned.unsupportedJoints.empty())
  {
    this->StepJoints(_ecm, _owned, _dt, moved ? bodyTwist : PlanarTwist());
  }

  // Write the frame data again on the step after the model stopped, so that
  // readers see zero velocity. Without a reader nothing is computed. The
  // first reader sets needsFrameData, and then the model writes in the next
  // step, whether it moves or not.
  const bool write = _owned.frameReaders && (moved || teleported ||
      _owned.wasMoving || _owned.needsFrameData);
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
  // change is written, and a component that another system created later,
  // for example for a robot spawned at runtime, is filled in the step it
  // appears in.
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

    // A paused step keeps the last velocity, as Physics does.
    const double velocity = _dt > 0.0 ? delta / _dt : wheel.velocity;
    const bool moved = NonZero(delta);
    if (moved)
      wheel.position += delta;

    auto *positionComp = _ecm.Component<components::JointPosition>(wheel.joint);
    auto *velocityComp = _ecm.Component<components::JointVelocity>(wheel.joint);
    const bool velocityChanged = NonZero(velocity - wheel.velocity);
    if (moved || wheel.dirty || settle || velocityChanged ||
        HasEmptyState(positionComp, velocityComp))
    {
      WriteJointState(_ecm, wheel.joint, positionComp, velocityComp,
          wheel.position, velocity, moved || wheel.dirty || settle,
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

    if (joint.held)
    {
      if (NonZero(rate) && !_owned.warnedHeldWheelCmd)
      {
        _owned.warnedHeldWheelCmd = true;
        gzwarn << "Joint [" << scopedName(joint.joint, _ecm) << "] looks "
            << "like a wheel, but it hangs from the chassis through a joint "
            << "that physics mode [" << ModeName(PhysicsMode::PlanarKinematic)
            << "] does not follow (a steering joint, a swivel or a bogie). "
            << "The mode does not drive the model from this wheel and holds "
            << "it at its position, so its velocity command has no effect. "
            << "Name the joint in a <wheel> element to drive the model from "
            << "it as if it were fixed." << std::endl;
      }
      rate = 0.0;
    }

    const double next =
        AdvanceJoint(joint.position, rate, _dt, joint.lower, joint.upper);
    const double velocity = _dt > 0.0 ? (next - joint.position) / _dt :
        joint.velocity;
    const bool moved = NonZero(next - joint.position);
    joint.position = next;

    auto *positionComp = _ecm.Component<components::JointPosition>(joint.joint);
    auto *velocityComp = _ecm.Component<components::JointVelocity>(joint.joint);
    const bool velocityChanged = NonZero(velocity - joint.velocity);
    if (moved || joint.dirty || settle || velocityChanged ||
        HasEmptyState(positionComp, velocityComp))
    {
      WriteJointState(_ecm, joint.joint, positionComp, velocityComp,
          joint.position, velocity, moved || joint.dirty || settle,
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
