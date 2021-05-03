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
#ifndef IGNITION_GAZEBO_SYSTEMS_ODOMETRYPUBLISHER_HH_
#define IGNITION_GAZEBO_SYSTEMS_ODOMETRYPUBLISHER_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class OdometryPublisherPrivate;

  /// \brief Odometry Publisher which can be attached to any entity in
  /// order to periodically publish 2D odometry data in the form of
  /// ignition::msgs::Odometry messages.
  ///
  /// # System Parameters
  ///
  /// `<odom_frame>`: Name of the world-fixed coordinate frame for the
  //  odometry message. This element is optional, and the default value
  /// is `{name_of_model}/odom`.
  ///
  /// `<robot_base_frame>`: Name of the coordinate frame rigidly attached
  /// to the mobile robot base. This element is optional, and the default
  /// value is `{name_of_model}/base_footprint`.
  ///
  /// `<odom_publish_frequency>`: Odometry publication frequency. This
  /// element is optional, and the default value is 50Hz.
  ///
  /// `<odom_topic>`: Custom topic on which this system will publish odometry
  /// messages. This element is optional, and the default value is
  /// `/model/{name_of_model}/odometry`.
  class IGNITION_GAZEBO_VISIBLE OdometryPublisher
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: OdometryPublisher();

    /// \brief Destructor
    public: ~OdometryPublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<OdometryPublisherPrivate> dataPtr;
  };
  }
}
}
}

#endif
