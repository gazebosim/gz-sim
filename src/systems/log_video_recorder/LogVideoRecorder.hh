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
#ifndef GZ_SIM_LOGVIDEORECORDER_SYSTEM_HH_
#define GZ_SIM_LOGVIDEORECORDER_SYSTEM_HH_

#include <memory>
#include <vector>
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
  class LogVideoRecorderPrivate;

  /** \class LogVideoRecorder LogVideoRecorder.hh \
   * gz/sim/systems/LogVideoRecorder.hh
  **/
  /// \brief System which recordings videos from log playback
  /// There are two ways to specify what entities in the log playback to follow
  /// and record videos for: 1) by entity name and 2) by region. See the
  /// system parameters.
  ///
  /// When recording is finished. An `end` string will be published to the
  /// `/log_video_recorder/status` topic and the videos are saved to a
  /// timestamped directory
  ///
  /// ## System Parameters
  ///
  /// - `<entity>`         Name of entity to record.
  /// - `<region>`         Axis-aligned box where entities are at start of log
  ///   + `<min>` Min corner position of box region.
  ///   + `<max>`  Max corner position of box region.
  /// - `<start_time>`     Sim time when recording should start
  /// - `<end_time>`       Sim time when recording should end
  /// - `<exit_on_finish>` Exit gz-sim when log playback recording ends
  class LogVideoRecorder final:
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
