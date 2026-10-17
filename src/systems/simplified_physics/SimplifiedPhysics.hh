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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_SIMPLIFIED_PHYSICS_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_SIMPLIFIED_PHYSICS_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class SimplifiedPhysicsPrivate;

  /// \brief A physics provider that moves models without a physics engine.
  ///
  /// The system simulates the models whose `components::PhysicsOwner` names
  /// it. A model gets that component from the SimplifiedPhysicsModel plugin.
  /// The default Physics system skips these models, so the two systems can run
  /// in one world, each with its own models. Neither sees the other's models
  /// as bodies: a model on Physics does not collide with one on this system.
  ///
  /// Only the `planar_kinematic` mode exists so far. For each model in that
  /// mode, in every Update:
  ///
  /// 1. A `WorldPoseCmd` on the model teleports it. The system removes the
  ///    command one step later, so that other systems can react to it first.
  ///    Physics differs: it applies the command in two steps. This system
  ///    applies it once, so the step after the teleport moves the model on
  ///    from where it is. The system compares the command's value with the
  ///    one it applied, so a command that repeats its value in every step is
  ///    applied in every other step. A system that holds a model in place
  ///    should send a new value in every step.
  /// 2. The model's `LinearVelocityCmd` and `AngularVelocityCmd`, which
  ///    VelocityControl writes in the model frame, are projected onto x, y and
  ///    yaw. They describe the model's canonical link, as on Physics, so the
  ///    model turns about that link and not about its own origin. The model
  ///    then moves along the exact arc for that velocity over the step. Its
  ///    z, roll and pitch do not change. A step without a command is a step
  ///    without motion.
  /// 3. If the model has no model velocity command (or it is zero), the
  ///    `JointVelocityCmd` of its wheel joints moves it (see "Wheels"). A
  ///    non-zero model velocity command takes precedence over wheel commands,
  ///    and the system warns once per model if both are non-zero in one
  ///    step. The wheels still follow the motion that the model achieves.
  /// 4. The system writes the frame data of the model's links, sensors and
  ///    collisions that other systems have asked for (world pose, velocities
  ///    and accelerations, in the world and body frames), the model's
  ///    `Pose`, and the `JointPosition` and `JointVelocity` of wheels and
  ///    other joints that have those components. It marks `Pose` as changed
  ///    only when the model moved. It computes frame data for a model only
  ///    while some link, sensor or collision of that model has one of the nine
  ///    frame data components, so models without a reader cost nothing for
  ///    it. A reader that appears later (a component created in any step,
  ///    also on a model that stands still) gets its first value in the next
  ///    step: PostUpdate notes the new component, and the next Update writes
  ///    the model's frames once, whether or not the model moves.
  /// 5. The system clears the commands it consumed. The velocity commands of
  ///    the model and of its joints stay in the ECM with a zero value instead
  ///    of being removed, because a system that writes them in every step
  ///    then overwrites a component and does not create one.
  ///
  /// ## Wheels
  ///
  /// The system finds the drive wheels of a model once, when it first sees
  /// it. A joint is a wheel if it is revolute or continuous with a horizontal
  /// axis, joins the canonical link (the chassis) to a link with a sphere
  /// collision or a cylinder collision along the axis, and that collision
  /// reaches down to the lowest point of the model. The collision gives the
  /// radius and the contact point. `<wheel>` elements in the
  /// SimplifiedPhysicsModel parameters replace the inference. See
  /// WheelDrive for the rolling constraints that turn wheel rates into the
  /// planar twist. A DiffDrive model needs nothing else: its
  /// `JointVelocityCmd` on the two wheels give v = r (wL + wR) / 2 and
  /// wz = r (wR - wL) / b exactly. A skid-steer vehicle with several wheels
  /// per side turns like a differential drive with the same track, the
  /// ideal turn. Full physics turns it more slowly, by its slip (4 wheels at
  /// (+-0.15, +-0.2) m and 4 and 6 rad/s: 1.05 rad on dartsim, 1.50 rad
  /// here, in 3 s). No accuracy contract covers skid-steer yet.
  ///
  /// The `JointPosition` and `JointVelocity` of every wheel follow the motion
  /// that the model achieved, not the command, so wheel odometry equals the
  /// ground truth of the planar motion. This holds for a model that moves
  /// under VelocityControl too. With `<odometry_noise>` above zero the
  /// written position differs from the achieved motion on purpose (see
  /// OdometryNoise). A passive wheel on a fixed axle rolls like a driven one.
  /// The system fills the components that exist, like Physics, and it fills
  /// a `JointPosition` or `JointVelocity` that has no value in the step in
  /// which it finds it empty, also for a robot that stands still. A robot
  /// spawned at runtime gets its wheel state, and DiffDrive its odometry, in
  /// the step after the robot appears.
  ///
  /// A wheel may hang from the chassis through prismatic joints (a
  /// suspension or a wheel drop). The mode treats them as fixed, at the
  /// position the model has when the system first sees it, and reads the
  /// wheel geometry there.
  ///
  /// Other revolute, continuous and prismatic joints (steering joints,
  /// swivel casters, lifts, turntables, the prismatic joint of a suspension)
  /// integrate their `JointVelocityCmd` into `JointPosition` and
  /// `JointVelocity` and stop at their limits. A `JointPositionReset` sets the
  /// position. The child link does not move, there is no dynamics, and no
  /// force or effort is simulated.
  ///
  /// A joint that passes every wheel test except the parent test, because it
  /// hangs from the chassis through a revolute joint (the wheel of a swivel
  /// caster, a steered wheel of an Ackermann vehicle, a bogie), is held at its
  /// position and does not move the model. The system warns once per model
  /// when such a joint gets a non-zero velocity command, naming the joint, so
  /// that a robot is never reported to drive when it does not: the model
  /// stays put and its odometry reads zero. Name the joint in a `<wheel>`
  /// element to drive the model from it as if the joint were fixed.
  ///
  /// The system runs at `kPrePhysicsPriority`, before Physics. It reads the
  /// commands of its models and clears them, and Physics, which does not own
  /// these models, leaves them alone.
  ///
  /// ## Which models the system simulates
  ///
  /// The models whose `components::PhysicsOwner` names this system. The
  /// SimplifiedPhysicsModel plugin sets it for one model. The world can also
  /// name this system as its default owner, and then it applies to every model
  /// that has no owner of its own. Two rules keep such a world working:
  ///
  /// * The system cannot move a static model or one without a pose. It gives
  ///   such a model back to Physics with an empty `PhysicsOwner` on the model,
  ///   before Physics looks at it, so that the ground of the world is still
  ///   simulated as before.
  /// * The system lists the features of the mode in the model's
  ///   `SupportedPhysicsFeatures`, so that systems on the model that need
  ///   other features are not run. This happens in Configure, and a world
  ///   `PhysicsOwner` that a plugin sets after this system is configured is
  ///   too late for systems that are already on the models. The plugin that
  ///   sets the world's owner has to come first in the world file. A model
  ///   that opts out of the default must clear its features too, as
  ///   SimplifiedPhysicsModel does for the `full` mode.
  ///
  /// Only an instance attached to the world works. An instance in a model
  /// prints an error and does nothing.
  ///
  /// ## Not simulated yet
  ///
  /// Collisions between models or with the world, link velocity commands,
  /// contacts, ray hits for CPU lidar and the `planar_dynamic` mode. A model
  /// moves through everything. Link velocity commands on a model that this
  /// system owns are cleared, and the system warns once for the model.
  ///
  /// Wheels and joints: the links of wheels and of articulated joints do not
  /// move or turn (`animate_wheels` is not implemented), so a rendered wheel
  /// stays still and a sensor on a lift does not follow it. Joint velocity
  /// commands on two-axis, screw and gearbox joints are ignored, with one
  /// warning per model. `JointVelocityReset`, forces, efforts and limit
  /// commands have no effect. A model whose steering joints change the rolling
  /// direction of its wheels (Ackermann) or whose wheels slide sideways by
  /// design (mecanum) is not supported: wheels see only the fixed direction of
  /// their axis. An Ackermann model drives only on its rear wheels, with the
  /// held front wheels reported as above, so it goes straight. A mecanum model
  /// solves to no motion for a sideways command, and nothing warns about it
  /// yet.
  ///
  /// The relative poses of a model's links are read once, when the model is
  /// first seen, and are assumed constant.
  ///
  /// ## Accuracy of the mode
  ///
  /// What the mode gives up against Physics, for the accuracy contract:
  ///
  /// * The yaw rate is the world z part of the commanded angular velocity. On
  ///   a pitched or rolled model a body z command of w turns the model at
  ///   `w cos(pitch) cos(roll)` (4.5 percent slow at 0.3 rad), and the yaw
  ///   angle is ill-defined near a pitch of 90 degrees.
  /// * The frame data of a link uses the same 1e-6 tolerance as Physics, so a
  ///   link's `WorldPose` does not update for motion below 1 micrometer per
  ///   step, while the model's `Pose` is written exactly.
  /// * A frame data component that a system adds to a model gets its first
  ///   value in the next step, also for a model that stands still, but is not
  ///   refreshed after that until the model moves. Physics refreshes it in
  ///   every step. A component created outside the system phases (in a
  ///   service callback, say) is found when the model first moves, or not at
  ///   all if the model never moves again.
  /// * A wheel touches the floor at the center of its collision. A flat
  ///   cylinder on dartsim reports contacts at both rims of the wheel in
  ///   every step, so the contact patch is wider than a point and the model
  ///   turns faster there for the same wheel speeds: 8.7 percent faster with
  ///   4 cm wide wheels on a 0.4 m track, 22 percent with 8 cm wide wheels
  ///   (3 s at fixed wheel rates of 4 and 6 rad/s). The size depends on the
  ///   run, for example on the acceleration limits of the controller. Sphere
  ///   wheels agree to 0.02 percent of the distance
  ///   (INTEGRATION_simplified_physics_wheels, MatchesDartsim). Most AMRs
  ///   have cylinder wheels, so a contract against dartsim for them fails by
  ///   this amount of the heading, and dartsim's own wheel odometry has the
  ///   same error against dartsim's ground truth, because DiffDrive assumes
  ///   the nominal track. This mode matches the controller's model.
  /// * Wheels roll without slip, so wheel odometry equals the planar ground
  ///   truth. Slip comes only from `<odometry_noise>`. The noise is sized for
  ///   a differential drive (see OdometryNoise).
  ///
  /// The system has no parameters yet.
  class SimplifiedPhysics:
    public System,
    public ISystemConfigure,
    public ISystemConfigurePriority,
    public ISystemReset,
    public ISystemUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: SimplifiedPhysics();

    /// \brief Destructor
    public: ~SimplifiedPhysics() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    // Documentation inherited
    public: System::PriorityType ConfigurePriority() final;

    // Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<SimplifiedPhysicsPrivate> dataPtr;
  };
}
}
}
}

#endif
