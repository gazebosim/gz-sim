/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_LOGVIDEORECORDER_SYSTEM_HH_
#define IGNITION_GAZEBO_LOGVIDEORECORDER_SYSTEM_HH_

#include <memory>
#include <vector>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  class LogVideoRecorderPrivate;

  /** \class LogVideoRecorder LogVideoRecorder.hh \
   * ignition/gazebo/systems/LogVideoRecorder.hh
  **/
  /// \brief System which recordings videos from log playback
  /// There are two ways to specify what entities in the log playback to follow
  /// and record videos for: 1) by entity name and 2) by region. See the
  /// following parameters:
  ///   <entity>         Name of entity to record.
  ///   <region>         Axis-aligned box where entities are at start of log
  ///       <min>        Min corner position of box region.
  ///       <max>        Max corner position of box region.
  ///   <start_time>     Sim time when recording should start
  ///   <end_time>       Sim time when recording should end
  ///   <exit_on_finish> Exit ign-gazebo when log playback recording ends
  /// When recording is finished. An `end` string will be published to the
  /// `/log_video_recorder/status` topic and the videos are saved to a
  /// timestamped directory
  class LogVideoRecorder:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: LogVideoRecorder();

    /// \brief Destructor
    public: ~LogVideoRecorder() final = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<LogVideoRecorderPrivate> dataPtr;
  };
}
}
}
}
#endif

