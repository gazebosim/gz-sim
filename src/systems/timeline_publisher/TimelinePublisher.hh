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
#ifndef GZ_SIM_SYSTEMS_TIMELINEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_TIMELINEPUBLISHER_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class TimelinePublisherPrivate;

  /// \brief Publishes messages and calls services at fixed points in
  /// simulation time, optionally looping. It plays a scripted sequence, such
  /// as joint position targets or velocity commands, without an external
  /// script. Keyframes follow simulation time, so pausing or stepping the
  /// simulation also pauses or steps the timeline, and the timeline starts
  /// over when the world is reset or the simulation time goes back. The
  /// timeline starts on the first unpaused step after the system is loaded,
  /// so a model spawned during the simulation starts its timeline when it
  /// appears.
  ///
  /// ## System Parameters
  ///
  /// - `<loop>`: Whether the timeline starts over after `<period>` seconds.
  ///   Defaults to false.
  ///
  /// - `<period>`: Length of the timeline in seconds. Defaults to the time
  ///   of the last keyframe, so the last keyframe and the first keyframe of
  ///   the next cycle fire on the same step. Must be greater than zero when
  ///   looping. If the last keyframe is later than the period, the period is
  ///   extended to it. If a step covers more than one cycle, for example
  ///   because the period is shorter than the step, only the latest cycle
  ///   fires.
  ///
  /// - `<keyframe>`: A point in the timeline. The `time` attribute is the
  ///   time in seconds since the start of the timeline. This element can be
  ///   repeated, and keyframes may be listed in any order. Keyframes with the
  ///   same time fire in the order they are listed. Each keyframe contains
  ///   any number of:
  ///
  ///   - `<output>`: Message to publish. The `topic` attribute is the topic
  ///     to publish on and the `type` attribute is the message type (e.g.
  ///     `gz.msgs.Double`). The content of the element is the message in
  ///     the protobuf text format.
  ///
  ///   - `<service>`: Service to call. The `name` attribute is the service
  ///     name, `reqType` and `repType` are the request and response message
  ///     types, `timeout` is the timeout in milliseconds (defaults to 1000)
  ///     and `reqMsg` is the request in the protobuf text format. Instead of
  ///     the `reqMsg` attribute, a long request can be the content of the
  ///     element. Service calls are made in order from a worker thread, so
  ///     they never block the simulation, but their effect isn't tied to an
  ///     exact step. When a new cycle starts, calls still waiting from
  ///     before the previous cycle are dropped, so a slow service can't make
  ///     the calls fall further and further behind. Calls still waiting when
  ///     the world is reset are dropped. Removing the system waits for the
  ///     call in progress, up to its timeout.
  ///
  /// ## Example
  ///
  /// Move a joint back and forth, repeating every 4 seconds:
  ///
  /// ```
  /// <plugin filename="gz-sim-timeline-publisher-system"
  ///         name="gz::sim::systems::TimelinePublisher">
  ///   <loop>true</loop>
  ///   <period>4</period>
  ///   <keyframe time="0">
  ///     <output topic="/model/arm/joint/j1/0/cmd_pos" type="gz.msgs.Double">
  ///       data: 1.0
  ///     </output>
  ///   </keyframe>
  ///   <keyframe time="2">
  ///     <output topic="/model/arm/joint/j1/0/cmd_pos" type="gz.msgs.Double">
  ///       data: -1.0
  ///     </output>
  ///   </keyframe>
  /// </plugin>
  /// ```
  class TimelinePublisher
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemReset
  {
    /// \brief Constructor
    public: TimelinePublisher();

    /// \brief Destructor
    public: ~TimelinePublisher() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TimelinePublisherPrivate> dataPtr;
  };
  }
}
}
}

#endif
