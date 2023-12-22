/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef MARITIME_PUBLISHERPLUGIN_HH_
#define MARITIME_PUBLISHERPLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace maritime
{
  /// \brief A system to publish user specified messages at a given time.
  /// It's also possible to publish the message repeatedly.
  ///
  /// ## Required system parameters
  ///
  /// * `<message>` is the tag containing the message information.
  ///               It's possible to specify multiple messages.
  ///
  ///   * Required attributes:
  ///     - `<type>` is the message type (eg. `gz.msgs.Boolean`).
  ///     - `<topic>` is the topic name where the message will be published.
  ///   * Optional attributes:
  ///     - `<at>` Simulation time to start publishing the message (seconds).
  ///              Default value is to publish the message when simulation
  ///              starts.
  ///     - `<every>` Continue publishing the message after <every> seconds.
  ///                 Default value is 0, which means only publish once.
  ///   * Value: String used to construct the protobuf message . This is
  ///     the human-readable representation of a protobuf message as used by
  ///     `gz topic` for publishing messages.
  ///
  /// ## Example
  /// <plugin filename="libPublisherPlugin.so" name="maritime::PublisherPlugin">
  ///   <message type="gz.msgs.Param" topic="/maritime/wavefield/parameters"
  ///            at="0.0" every="2.0">
  ///     params {
  ///       key: "amplitude"
  ///       value {
  ///         type: DOUBLE
  ///         double_value: 0
  ///       }
  ///     }
  ///   </message>
  ///   <message type="gz.msgs.StringMsg" topic="/foo"
  ///            at="5.0" every="1.0">
  ///     data: "hello boat"
  ///   </message>
  /// </plugin>
  class PublisherPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: PublisherPlugin();

    /// \brief Destructor.
    public: ~PublisherPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
