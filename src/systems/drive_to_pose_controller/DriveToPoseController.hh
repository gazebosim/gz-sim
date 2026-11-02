/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_SIM_SYSTEMS_DRIVETOPOSECONTROLLER_HH_
#define GZ_SIM_SYSTEMS_DRIVETOPOSECONTROLLER_HH_

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

#include <sdf/Element.hh>

#include <memory>
#include <string>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
  // Forward declaration
  class DriveToPoseControllerPrivate;

  /// \brief DriveToPoseController is a simple proportional controller that
  /// is attached to a model to move it by giving a pose in Gazebo's
  /// world coordinate system. This is not a standalone plugin, and requires
  /// the DiffDrive and OdometryPublisher plugins respectively.
  ///
  /// The plugin has the following tags:
  ///
  /// - `<linear_p_gain>`: (Optional) Proportional gain for the linear
  ///                      velocity controller | Default: 1.0
  ///
  /// - `<angular_p_gain>`: (Optional) Proportional gain for the angular
  ///                       velocity controller | Default: 2.0
  ///
  /// - `<linear_deviation>`: (Optional) Allowable linear deviation (in meters)
  ///                         from the desired coordinate | Default: 0.1
  ///
  /// - `<angular_deviation>`: (Optional) Allowable angular deviation (in rad)
  ///                          from the desired orientation | Default: 0.05
  ///
  /// - `<cmd_pose_topic>`: (Optional) Topic for receiving target pose messages.
  ///                       Default: `/model/{model_name}/cmd_pose`.
  ///                       With a namespace: `{namespace}/cmd_pose`.
  ///                       Relative custom topics are prefixed with the
  ///                       namespace, while absolute topics remain unchanged.
  ///
  /// - `<cmd_vel_topic>`: (Optional) Topic for publishing velocity commands.
  ///                      Default: `/model/{model_name}/cmd_vel`.
  ///                      With a namespace: `{namespace}/cmd_vel`.
  ///                      Relative custom topics are prefixed with the
  ///                      namespace, while absolute topics remain unchanged.
  ///
  /// - `<odom_topic>`: (Optional) Topic for receiving odometry pose messages.
  ///                   Default: `/model/{model_name}/pose`.
  ///                   With a namespace: `{namespace}/pose`.
  ///                   Relative custom topics are prefixed with the namespace,
  ///                   while absolute topics remain unchanged.
  ///
  /// - `<reached_pose_topic>`: (Optional) Topic for publishing reached poses.
  ///                           Default: `/model/{model_name}/reached_pose`.
  ///                           With a namespace: `{namespace}/reached_pose`.
  ///                           Relative custom topics are prefixed with the
  ///                           namespace, while absolute topics remain
  ///                           unchanged.
  class DriveToPoseController
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Resolved topic names
    public: struct TopicNames
    {
      /// \brief Command velocity topic name
      public: std::string cmdVelTopic;

      /// \brief Command pose topic name
      public: std::string cmdPoseTopic;

      /// \brief Odometry topic name
      public: std::string odomTopic;

      /// \brief Reached pose topic name
      public: std::string reachedPoseTopic;
    };

    /// \brief Constructor
    public: DriveToPoseController();

    /// \brief Destructor
    public: ~DriveToPoseController() override;

    // Documentation inherited
    public: void Configure(
                  const Entity& _entity,
                  const std::shared_ptr<const sdf::Element>& _sdf,
                  EntityComponentManager& _ecm,
                  EventManager& _eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                  const UpdateInfo& _info,
                  const EntityComponentManager& _ecm) override;

    /// \brief Get the resolved topic names
    public: TopicNames ResolvedTopicNames() const;

    /// \brief Private data pointer
    private: std::unique_ptr<DriveToPoseControllerPrivate> dataPtr;
  };
}
}
}
}

#endif
