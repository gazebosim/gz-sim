/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_TRIGGEREDPUBLISHER_HH_
#define IGNITION_GAZEBO_SYSTEMS_TRIGGEREDPUBLISHER_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/triggered-publisher-system/Export.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class InputMatcher;

  /// \brief The triggered publisher system publishes a user specified message
  /// on an output topic in response to an input message that matches user
  /// specified criteria.
  ///
  /// ## System Parameters
  ///
  /// `<input>` The tag contains the input message type, topic and matcher
  /// information.
  ///   * Attributes:
  ///     * `type`: Input message type (eg. `ignition.msgs.Boolean`)
  ///     * `topic`: Input message topic name
  ///
  /// `<input><match>`: Contains configuration for matchers. Multiple <match>
  /// tags are possible. An output message is triggered if all Matchers match.
  ///   * Attributes:
  ///     * `logic_type`("positive" or "negative"): Specifies whether a
  ///         comparison must succeed or fail in order to trigger an output
  ///         message. A "positive" value triggers a match when a comparison
  ///         succeeds. A "negative" value triggers a match when a comparson
  ///         fails. The default value is "positive"
  ///     * `tol`: Tolerance for floating point comparisons.
  ///     * `field`: If specified, only this field inside the input
  ///         message is compared for a match.
  ///   * Value: String used to construct the protobuf message against which
  ///       input messages are matched. This is the human-readable
  ///       representation of a protobuf message as used by `ign topic` for
  ///       publishing messages
  ///
  /// `<output>`: Contains configuration for output messages: Multiple <output>
  /// tags are possible. A message will be published on each output topic for
  /// each input that matches.
  ///   * Attributes:
  ///     * `type`: Output message type (eg. `ignition.msgs.Boolean`)
  ///     * `topic`: Output message topic name
  ///   * Value: String used to construct the output protobuf message . This is
  ///     the human-readable representation of a protobuf message as used by
  ///     `ign topic` for publishing messages
  ///
  /// Examples:
  /// 1. Any receipt of a Boolean messages on the input topic triggers an output
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="ignition.msgs.Boolean" topic="/input_topic"/>
  ///      <output type="ignition.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// 2. Full match: An output is triggered when a Boolean message with a value
  ///    of "true" is received
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="ignition.msgs.Boolean" topic="/input_topic">
  ///        <match>
  ///            data: true
  ///        </match>
  ///      </input>
  ///      <output type="ignition.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// 3. Field match: An output is triggered when a specific field matches
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="ignition.msgs.Vector2d" topic="/input_topic">
  ///        <match field="x">1.0</match>
  ///        <match field="y">2.0</match>
  ///      </input>
  ///      <output type="ignition.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// The `logic_type` attribute can be used to negate a match. That is, to
  /// trigger an output when the input does not equal the value in <match>
  /// For example, the following will trigger an ouput when the input does not
  /// equal 1 AND does not equal 2.
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="ignition.msgs.Int32" topic="/input_topic">
  ///        <match logic_type="negative">1</match>
  ///        <match logic_type="negative">2</match>
  ///      </input>
  ///      <output type="ignition.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// ### Repeated Fields
  /// When a field matcher is used with repeated fields, the content of the
  /// repeated field is treated as a set and the comparison operator is set
  /// containment. For example, the `data` field of `ignition.msgs.Int32_V` is a
  /// repeated Int32 message. To match an input that contains the values 1 and 2
  /// the following matcher can be used:
  /// \code{.xml}
  ///  <plugin>
  ///    <input type="ignition.msgs.Int32_V" topic="/input_topic">
  ///      <match field="data">1</match>
  ///      <match field="data">2</match>
  ///    </input>
  ///    <output type="ignition.msgs.Empty" topic="/output_topic"/>
  ///  </plugin>
  /// \endcode
  /// To match an Int32_V message with the exact contents {1, 2}, the full
  /// matcher is used instead
  /// \code{.xml}
  /// <plugin>
  ///   <input type="ignition.msgs.Int32_V" topic="/input_topic">
  ///     <match>
  ///        data: 1
  ///        data: 2
  ///     </match>
  ///   </input>
  ///   <output type="ignition.msgs.Empty" topic="/output_topic"/>
  /// </plugin>
  /// \endcode
  ///
  /// ### Limitations
  /// The current implementation of this system does not support specifying a
  /// subfield of a repeated field in the "field" attribute. i.e, if
  /// `field="f1.f2"`, `f1` cannot be a repeated field.
  class IGNITION_GAZEBO_TRIGGERED_PUBLISHER_SYSTEM_VISIBLE TriggeredPublisher : public System,
                                                     public ISystemConfigure
  {
    /// \brief Constructor
    public: TriggeredPublisher() = default;

    /// \brief Destructor
    public: ~TriggeredPublisher() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// \brief Thread that handles publishing output messages
    public: void DoWork();

    /// \brief Helper function that calls Match on every InputMatcher available
    /// \param[in] _inputMsg Input message
    /// \return True if all of the matchers return true
    public: bool MatchInput(const transport::ProtoMsg &_inputMsg);

    /// \brief Input message type (eg. ignition.msgs.Boolean)
    private: std::string inputMsgType;

    /// \brief Input message topic
    private: std::string inputTopic;

    /// \brief Class that holds necessary bits for each specified output.
    private: struct OutputInfo
    {
      /// \brief Output message type
      std::string msgType;

      /// \brief Output message topic
      std::string topic;

      /// \brief Protobuf message of the output parsed from the human-readable
      /// string specified in the <output> element of the plugin's configuration
      transport::ProtoMsgPtr msgData;

      /// \brief Transport publisher
      transport::Node::Publisher pub;
    };

    /// \brief List of InputMatchers
    private: std::vector<std::unique_ptr<InputMatcher>> matchers;

    /// \brief List of outputs
    private: std::vector<OutputInfo> outputInfo;

    /// \brief Ignition communication node.
    private: transport::Node node;

    /// \brief Counter that tells the publisher how many times to publish
    private: std::size_t publishCount{0};

    /// \brief Mutex to synchronize access to publishCount
    private: std::mutex publishCountMutex;

    /// \brief Condition variable to signal that new matches have occured
    private: std::condition_variable newMatchSignal;

    /// \brief Thread handle for worker thread
    private: std::thread workerThread;

    /// \brief Flag for when the system is done and the worker thread should
    /// stop
    private: std::atomic<bool> done{false};
  };
  }
}
}
}

#endif
