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
#ifndef IGNITION_GAZEBO_SYSTEMS_TRACKCONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_TRACKCONTROLLER_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/PhysicsEvents.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class TrackControllerPrivate;

  /// \brief Controller of a track on either a conveyor belt or a tracked
  /// vehicle. The system should be attached to a model. If implementing a
  /// tracked vehicle, use also TrackedVehicle system.
  ///
  /// The system is implemented along the lines of M. Pecka, K. Zimmermann and
  /// T. Svoboda, "Fast simulation of vehicles with non-deformable tracks,"
  /// 2017 IEEE/RSJ International Conference on Intelligent Robots and
  /// Systems (IROS), 2017, pp. 6414-6419, doi: 10.1109/IROS.2017.8206546. It
  /// does not provide 100% plausible track drivetrain simulation, but provides
  /// one that is provably better than a set of wheels instead of the track.
  /// Only velocity control is supported - no effort controller is available.
  /// The basic idea of the implementation is utilizing the so called "contact
  /// surface motion" parameter of each contact point between the track and the
  /// environment. Instead of telling the physics engine to push velocity
  /// towards zero in contact points (up to friction), it tells it to maintain
  /// the desired track velocity in the contact point (up to friction). For
  /// better behavior when turning with tracked vehicles, it also accepts the
  /// position of the center of rotation of the whole vehicle so that it can
  /// adjust the direction of friction along the desired circle.
  ///
  /// # Examples
  ///
  /// See example usage in worlds example/conveyor.sdf and
  /// example/tracked_vehicle_simple.sdf .
  ///
  /// # System Parameters
  ///
  /// `<link>` Name of the link the controller controls.
  ///
  /// `<debug>` If 1, the system will output debugging info and visualizations.
  ///
  /// `<track_orientation>` Orientation of the track relative to the link.
  ///   It is assumed that the track moves along the +x direction of the
  ///   transformed coordinate system.
  ///
  /// `<velocity_topic>` Name of the topic on which the system accepts velocity
  ///   commands (defaults to`/model/${model}/link/${link}/track_cmd_vel`).
  ///
  /// `<center_of_rotation_topic>` The topic on which the track accepts center
  ///   of rotation commands (defaults to
  ///   `/model/${model}/link/${link}/track_cmd_center_of_rotation`).
  ///
  /// `<min_velocity>`/`<max_velocity>` Min/max velocity of the track (m/s).
  ///
  /// `<min_acceleration>`/`<max_acceleration>` Min/max acceleration of the
  ///   track (m/s^2).
  ///
  /// `<min_jerk>`/`<max_jerk>` Min/max jerk of the track (m/s^3).
  class TrackController
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: TrackController();

    /// \brief Destructor
    public: ~TrackController() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TrackControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
