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
#ifndef GZ_SIM_SYSTEMS_TRIGGEREDPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_TRIGGEREDPUBLISHER_HH_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <gz/transport/Node.hh>
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class InputMatcher;

  /// \brief The triggered publisher system publishes a user specified message
  /// on an output topic in response to an input message that matches user
  /// specified criteria. It can also call a user specified service as an
  /// output in response to an input message. It currently supports blocking
  /// service call. An optional simulation time delay can be used delay message
  /// publication.
  ///
  /// ## System Parameters
  ///
  /// - `<input>`: The tag contains the input message type, topic and matcher
  /// information.
  ///   * Attributes:
  ///     * `type`: Input message type (e.g. `gz.msgs.Boolean`)
  ///     * `topic`: Input message topic name
  ///
  /// - `<input><match>`: Contains configuration for matchers. Multiple
  /// `<match>` tags are possible. An output message is triggered if all
  /// Matchers match.
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
  ///       representation of a protobuf message as used by `gz topic` for
  ///       publishing messages
  ///
  /// - `<output>`: Contains configuration for output messages: Multiple
  /// `<output>` tags are possible. A message will be published on each output
  /// topic for each input that matches.
  ///   * Attributes:
  ///     * `type`: Output message type (e.g. `gz.msgs.Boolean`)
  ///     * `topic`: Output message topic name
  ///   * Value: String used to construct the output protobuf message . This is
  ///     the human-readable representation of a protobuf message as used by
  ///     `gz topic` for publishing messages
  ///
  /// - `<delay_ms>`: Integer number of milliseconds, in simulation time,  to
  /// delay publication.
  ///
  /// - `<service>`: Contains configuration for service to call: Multiple
  /// `<service>` tags are possible. A service will be called for each input
  /// that matches.
  ///   * Attributes:
  ///     * `name`: Service name (e.g. `/world/triggered_publisher/set_pose`)
  ///     * `timeout`: Service timeout
  ///     * `reqType`: Service request message type (e.g. gz.msgs.Pose)
  ///     * `repType`: Service response message type (e.g. gz.msgs.Empty)
  ///     * `reqMsg`: String used to construct the service protobuf message.
  ///
  /// ## Examples
  ///
  /// 1. Any receipt of a Boolean messages on the input topic triggers an output
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="gz.msgs.Boolean" topic="/input_topic"/>
  ///      <output type="gz.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// 2. Full match: An output is triggered when a Boolean message with a value
  ///    of "true" is received
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="gz.msgs.Boolean" topic="/input_topic">
  ///        <match>
  ///            data: true
  ///        </match>
  ///      </input>
  ///      <output type="gz.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// 3. Field match: An output is triggered when a specific field matches
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="gz.msgs.Vector2d" topic="/input_topic">
  ///        <match field="x">1.0</match>
  ///        <match field="y">2.0</match>
  ///      </input>
  ///      <output type="gz.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// The `logic_type` attribute can be used to negate a match. That is, to
  /// trigger an output when the input does not equal the value in `<match>`
  /// For example, the following will trigger an ouput when the input does not
  /// equal 1 AND does not equal 2.
  /// \code{.xml}
  ///    <plugin>
  ///      <input type="gz.msgs.Int32" topic="/input_topic">
  ///        <match logic_type="negative">1</match>
  ///        <match logic_type="negative">2</match>
  ///      </input>
  ///      <output type="gz.msgs.Empty" topic="/output_topic"/>
  ///    </plugin>
  /// \endcode
  ///
  /// ### Repeated Fields
  /// When a field matcher is used with repeated fields, the content of the
  /// repeated field is treated as a set and the comparison operator is set
  /// containment. For example, the `data` field of `gz.msgs.Int32_V` is a
  /// repeated Int32 message. To match an input that contains the values 1 and 2
  /// the following matcher can be used:
  /// \code{.xml}
  ///  <plugin>
  ///    <input type="gz.msgs.Int32_V" topic="/input_topic">
  ///      <match field="data">1</match>
  ///      <match field="data">2</match>
  ///    </input>
  ///    <output type="gz.msgs.Empty" topic="/output_topic"/>
  ///  </plugin>
  /// \endcode
  /// To match an Int32_V message with the exact contents {1, 2}, the full
  /// matcher is used instead
  /// \code{.xml}
  /// <plugin>
  ///   <input type="gz.msgs.Int32_V" topic="/input_topic">
  ///     <match>
  ///        data: 1
  ///        data: 2
  ///     </match>
  ///   </input>
  ///   <output type="gz.msgs.Empty" topic="/output_topic"/>
  /// </plugin>
  /// \endcode
  ///
  /// ## Limitations
  /// The current implementation of this system does not support specifying a
  /// subfield of a repeated field in the "field" attribute. i.e, if
  /// `field="f1.f2"`, `f1` cannot be a repeated field.
  class TriggeredPublisher : public System,
                             public ISystemConfigure,
                             public ISystemPreUpdate
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

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Thread that handles publishing output messages
    public: void DoWork();

    /// \brief Method that calls a service
    private: void CallService(std::size_t pendingSrv);

    /// \brief Method that publishes a message
    private: void PublishMsg(std::size_t pending);

    /// \brief Helper function that calls Match on every InputMatcher available
    /// \param[in] _inputMsg Input message
    /// \return True if all of the matchers return true
    public: bool MatchInput(const transport::ProtoMsg &_inputMsg);

    /// \brief Input message type (eg. gz.msgs.Boolean)
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

    /// \brief Class that holds necessary bits for each specified service output
    private: struct SrvOutputInfo
    {
      /// \brief Service name
      std::string srvName;

      /// \brief Service request type
      std::string reqType;

      /// \brief Service reply type
      std::string repType;

      /// \brief Service request message
      std::string reqMsg;

      /// \brief Serivce timeout
      int timeout;
    };

    /// \brief List of InputMatchers
    private: std::vector<std::unique_ptr<InputMatcher>> matchers;

    /// \brief List of outputs
    private: std::vector<OutputInfo> outputInfo;

    /// \brief List of service outputs
    private: std::vector<SrvOutputInfo> srvOutputInfo;

    /// \brief Gazebo communication node.
    private: transport::Node node;

    /// \brief Counter that tells how manny times to call the service
    private: std::size_t serviceCount{0};

    /// \brief Counter that tells the publisher how many times to publish
    private: std::size_t publishCount{0};

    /// \brief Mutex to synchronize access to publishCount
    private: std::mutex publishCountMutex;

    /// \brief Mutex to synchronize access to serviceCount
    private: std::mutex triggerSrvMutex;

    /// \brief Condition variable to signal that new matches have occured
    private: std::condition_variable newMatchSignal;

    /// \brief Thread handle for worker thread
    private: std::thread workerThread;

    /// \brief Flag for when the system is done and the worker thread should
    /// stop
    private: std::atomic<bool> done{false};

    /// \brief Publish delay time. This is in simulation time.
    private: std::chrono::steady_clock::duration delay{0};

    /// \brief Queue of publication times.
    private: std::vector<std::chrono::steady_clock::duration> publishQueue;

    /// \brief Mutex to synchronize access to publishQueue
    private: std::mutex publishQueueMutex;
  };
  }
}
}
}

#endif
