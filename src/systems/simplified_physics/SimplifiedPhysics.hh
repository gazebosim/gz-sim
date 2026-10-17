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
  /// 3. The system writes the frame data of the model's links, sensors and
  ///    collisions that other systems have asked for (world pose, velocities
  ///    and accelerations, in the world and body frames), and the model's
  ///    `Pose`. It marks `Pose` as changed only when the model moved.
  /// 4. The system clears the commands it consumed. The model's velocity
  ///    commands stay in the ECM with a zero value instead of being removed,
  ///    because a system that writes them in every step then overwrites a
  ///    component and does not create one.
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
  /// Collisions between models or with the world, wheel joints and joint
  /// commands, link velocity commands, contacts, ray hits for CPU lidar and
  /// the `planar_dynamic` mode. A model moves through everything, and wheel
  /// joints stay at their initial position, so wheel odometry does not work.
  /// A joint or link velocity command on a model that this system owns is
  /// cleared, and the system warns once for the model.
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
  /// * A component that a system adds to an idle model after the first step
  ///   keeps its initial value until the model moves. Physics refreshes it in
  ///   every step.
  ///
  /// The system has no parameters yet.
  class SimplifiedPhysics:
    public System,
    public ISystemConfigure,
    public ISystemConfigurePriority,
    public ISystemReset,
    public ISystemUpdate
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

    /// \brief Private data pointer.
    private: std::unique_ptr<SimplifiedPhysicsPrivate> dataPtr;
  };
}
}
}
}

#endif
