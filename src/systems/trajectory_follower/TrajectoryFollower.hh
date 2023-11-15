/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_TRAJECTORYFOLLOWER_HH_
#define GZ_SIM_SYSTEMS_TRAJECTORYFOLLOWER_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class TrajectoryFollowerPrivate;

  /// \brief A plugin that scripts the movement of a model based on waypoints.
  /// Note that at this point, the trajectory is always in 2D (x, y).
  /// The plugin applies a torque until the model is aligned with the next
  /// waypoint. Then, it applies a force until the model is close enough to
  /// the waypoint.
  ///
  /// This plugin loads a set of waypoints via SDF and traverse the waypoints
  /// in sequence. The movement is generated applying force and torque to one of
  /// of the model links. It's also possible to loop through the waypoints for
  /// generating a never ending trajectory.
  /// Waypoints may be inserted manually via the <waypoints> element, or
  /// generated relative to the model's initial position via the <line> or
  /// <circle> elements.  Only one of these options should be used.
  ///
  /// ## System Parameters
  ///
  /// This plugin requires the following SDF parameters:
  ///
  /// Required parameters:
  ///
  /// - `<link_name>`: The name of the link within the model where the
  ///   force/torque will be applied when moving the vehicle.
  ///
  /// Optional parameters:
  ///
  /// - `<loop>`: When true, all waypoints will be visited continously in a
  ///   circular pattern. If false, the model will stop when the
  ///   last waypoint is reached. Note, that if the vehicle moves,
  ///   it will still try to reach the very last waypoint.
  ///
  /// - `<waypoints>`: Element specifying the set of waypoints that the
  ///   the model should navigate through. This block should contain
  ///   at least one of these blocks:
  ///   - `<waypoint>`: This block should contain the X, Y of a waypoint.
  ///
  /// - `<range_tolerance>`: The current waypoint is considered reached if the
  ///   distance to it is within +- this tolerance (m).
  ///   The default value is 0.5m.
  ///
  /// - `<bearing_tolerance>`: If the bearing to the current waypoint is within
  ///   +- this tolerance, a torque won't be applied (degree).
  ///   The default value is 2 deg.
  ///
  /// - `<zero_vel_on_bearing_reached>`: Force angular velocity to be zero when
  ///   target bearing is reached.
  ///   Default is false.
  ///   Note: this is an experimental parameter and may be removed in the
  ///   future.
  ///
  /// - `<force>`: The force to apply at every plugin iteration in the X
  ///   direction of the link (N). The default value is 60.
  ///
  /// - `<torque>`: The torque to apply at every plugin iteration in the Yaw
  ///   direction of the link (Nm). The default value is 50.
  ///
  /// - `<line>`: Element that indicates the model should travel in "line" mode.
  ///   The block should contain the relative direction and distance from
  ///   the initial position in which the vehicle should move, specified
  ///   in the world frame.
  ///   - `<direction>`: Relative direction (radians) in the world frame for
  ///     the vehicle to travel.
  ///   - `<length>`: Distance (meters) for the vehicle to travel.
  ///
  /// - `<circle>`: Element that indicates the model should travel in "circle"
  ///   mode. The block should contain the desired radius of the circle about
  ///   the vehicle's initial position
  ///   - `<radius>`: Radius (meters) of circular path to travel.
  ///
  /// Here are three examples:
  /// ```
  /// <plugin
  ///   filename="gz-sim-trajectory-follower-system"
  ///   name="gz::sim::systems::TrajectoryFollower">
  ///   <link_name>base_link</link_name>
  ///   <loop>true</loop>
  ///   <waypoints>
  ///     <waypoint>25 0</waypoint>
  ///     <waypoint>15 0</waypoint>
  ///   </waypoints>
  /// </plugin>
  /// <plugin
  ///   filename="gz-sim-trajectory-follower-system"
  ///   name="gz::sim::systems::TrajectoryFollower">
  ///   <link_name>base_link</link_name>
  ///   <loop>true</loop>
  ///   <line>
  ///     <direction>0</direction>
  ///     <length>5</length>
  ///   </line>
  /// </plugin>
  /// <plugin
  ///   filename="gz-sim-trajectory-follower-system"
  ///   name="gz::sim::systems::TrajectoryFollower">
  ///   <link_name>base_link</link_name>
  ///   <loop>true</loop>
  ///   <circle>
  ///     <radius>2</radius>
  ///   </circle>
  /// </plugin>
  /// ```
  class TrajectoryFollower
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: TrajectoryFollower();

    /// \brief Destructor
    public: ~TrajectoryFollower() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Check if an entity is enabled or not.
    /// \param[in] _entity Target entity
    /// \param[in] _ecm Entity component manager
    /// \return True if buoyancy should be applied.
    public: bool IsEnabled(Entity _entity,
        const EntityComponentManager &_ecm) const;

    /// \brief Private data pointer
    private: std::unique_ptr<TrajectoryFollowerPrivate> dataPtr;
  };
  }
}
}
}

#endif
