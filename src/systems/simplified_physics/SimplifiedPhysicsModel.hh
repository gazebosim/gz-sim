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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_SIMPLIFIED_PHYSICS_MODEL_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_SIMPLIFIED_PHYSICS_MODEL_HH_

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
  /// \brief Chooses how the physics of one model is simulated.
  ///
  /// This is a model plugin that only runs in Configure. For a planar mode it
  /// marks the model as owned by the SimplifiedPhysics system, so that the
  /// default Physics system leaves the model alone, and it lists the physics
  /// features that this mode serves. Systems on the model that need other
  /// features are not run, with a warning (see ISystemPhysicsRequirements).
  /// Nothing else reads the mode name.
  ///
  /// If the world names another provider as its default owner, a model with
  /// the `full` mode (or whose request is refused) opts out of that default:
  /// it gets an empty PhysicsOwner, which Physics reads, and loses any
  /// SupportedPhysicsFeatures that the world's provider listed for it.
  ///
  /// The SimplifiedPhysics system must be loaded in the world, or no system
  /// simulates a model with a planar mode.
  ///
  /// The plugin goes on a top-level model. It is the same for every
  /// fidelity of one robot, so one model file serves all modes: the mode is
  /// chosen where the model is included, for example with
  /// `<experimental:params>`.
  ///
  /// ## Parameters
  ///
  /// - `<mode>`: `full` (default), `planar_kinematic` or `planar_dynamic`.
  ///   `full` leaves the model to the default Physics system.
  ///   `planar_kinematic` moves the model in x, y and yaw at the velocity
  ///   that its VelocityControl system commands. `planar_dynamic` is not
  ///   implemented yet: the plugin prints a warning and keeps the model on
  ///   `full`. An unknown value also keeps the model on `full`.
  /// - `<collision_response>`: `slide` (default) or `stop`, for
  ///   `planar_kinematic`. What the model does when its motion runs into the
  ///   world or another model: `slide` keeps the part of the motion along the
  ///   obstacle, `stop` ends the motion at the first touch. Either way, a
  ///   model that touches an obstacle cannot turn in place into it, and two
  ///   models whose commands push them together stay put until a command
  ///   changes. Any other value is ignored with a warning and `slide` is
  ///   used.
  /// - `<odometry_noise>`: fraction, default 0 (off). Only for a model with
  ///   wheel joints. Adds a fixed scale error per wheel, drawn once, and a
  ///   zero-mean random walk to the wheel positions that the model reports,
  ///   so that wheel odometry (DiffDrive) drifts. It is sized so that a
  ///   differential drive ends a 10 m straight run with a median odometry
  ///   error of this fraction of the distance (0.015 gives 15 cm), mostly
  ///   sideways. The noise is repeatable for a fixed world seed (the
  ///   server's `--seed`): each wheel's seed comes from the world seed and
  ///   the joint's scoped name. Without a seed the server picks a random one
  ///   and runs differ. The actual motion of the model stays exact.
  /// - `<wheel radius="r">joint</wheel>`: repeatable. Names a wheel joint
  ///   and optionally its radius in meters. When present these joints are
  ///   the wheels and nothing is inferred. A named joint needs a sphere or
  ///   cylinder collision on its child link, or a radius.
  /// - `<animate_wheels>`: accepted and warned about. The wheel links do not
  ///   turn yet.
  ///
  /// ## Example
  ///
  /// ```xml
  /// <plugin filename="gz-sim-simplified-physics-system"
  ///         name="gz::sim::systems::SimplifiedPhysicsModel">
  ///   <mode>planar_kinematic</mode>
  /// </plugin>
  /// ```
  class SimplifiedPhysicsModel:
    public System,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: SimplifiedPhysicsModel();

    /// \brief Destructor
    public: ~SimplifiedPhysicsModel() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;
  };
}
}
}
}

#endif
