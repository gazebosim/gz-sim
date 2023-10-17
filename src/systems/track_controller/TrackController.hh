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
#ifndef GZ_SIM_SYSTEMS_TRACKCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_TRACKCONTROLLER_HH_

#include <memory>
#include <gz/sim/System.hh>
#include "gz/sim/physics/Events.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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
  /// Systems (IROS), 2017, pp. 6414-6419, doi: 10.1109/IROS.2017.8206546.
  ///
  /// It does not provide 100% plausible track drivetrain simulation, but
  /// provides one that is provably better than a set of wheels instead of the
  /// track.
  ///
  /// Only velocity control is supported - no effort controller is available.
  ///
  /// The basic idea of the implementation is utilizing the so called "contact
  /// surface motion" parameter of each contact point between the track and the
  /// environment. Instead of telling the physics engine to push velocity
  /// towards zero in contact points (up to friction), it tells it to maintain
  /// the desired track velocity in the contact point (up to friction).
  ///
  /// For better behavior when turning with tracked vehicles, it also accepts
  /// the position of the center of rotation of the whole vehicle so that it can
  /// adjust the direction of friction along the desired circle.
  ///
  /// This system does not simulate the effect of grousers. The best way to
  /// achieve a similar effect is to set a very high `<mu1>` for the track
  /// links.
  ///
  /// ## System Parameters
  ///
  /// - `<link>`: Name of the link the controller controls. Required parameter.
  ///
  /// - `<debug>`: If 1, the system will output debugging info and
  ///   visualizations. The default value is 0.
  ///
  /// - `<track_orientation>`: Orientation of the track relative to the link.
  ///   It is assumed that the track moves along the +x direction of the
  ///   transformed coordinate system. Defaults to no rotation (`0 0 0`).
  ///
  /// - `<velocity_topic>`: Name of the topic on which the system accepts
  ///   velocity commands.
  ///   Defaults to `/model/${model_name}/link/${link_name}/track_cmd_vel`.
  ///
  /// - `<center_of_rotation_topic>`: The topic on which the track accepts
  ///   center of rotation commands. Defaults to
  ///   `/model/${model_name}/link/${link_name}/track_cmd_center_of_rotation`.
  ///
  /// - `<max_command_age>`: If this parameter is set, each velocity or center
  ///   of rotation command will only act for the given number of seconds and
  ///   the track will be stopped if no command arrives before this timeout.
  ///
  /// - `<odometry_topic>`: The topic on which the track odometry (i.e. position
  ///   and instantaneous velocity) is published. This can be used e.g. to
  ///   simulate a conveyor with encoder feedback.
  ///   Defaults to `/model/${model_name}/link/${link_name}/odometry`.
  ///
  /// - `<odometry_publish_frequency>`: the frequency (in Hz) at which the
  ///   odometry messages are published. Defaults to 50 Hz.
  ///
  /// - `<min_velocity>`/`<max_velocity>`: Min/max velocity of the track (m/s).
  ///   If not specified, the velocity is not limited (however the physics will,
  ///   in the end, have some implicit limit).
  ///
  /// - `<min_acceleration>`/`<max_acceleration>`: Min/max acceleration of the
  ///   track (m/s^2). If not specified, the acceleration is not limited
  ///   (however the physics will, in the end, have some implicit limit).
  ///
  /// - `<min_jerk>`/`<max_jerk>`: Min/max jerk of the track (m/s^3). If not
  ///   specified, the acceleration is not limited (however the physics will,
  ///   in the end, have some implicit limit).
  ///
  /// ## Examples
  ///
  /// See example usage in worlds `example/conveyor.sdf` and
  /// `example/tracked_vehicle_simple.sdf`.
  class TrackController
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
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
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TrackControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
