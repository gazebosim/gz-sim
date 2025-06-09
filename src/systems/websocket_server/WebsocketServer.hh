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

// This file was ported from:
// https://github.com/gazebosim/gz-launch/blob/main/plugins/websocket_server
// and converted to a gz-sim system.

#ifndef GZ_SIM_SYSTEMS_WEBSOCKETSERVER_HH_
#define GZ_SIM_SYSTEMS_WEBSOCKETSERVER_HH_

#include <chrono>
#include <cstddef>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/MessageInfo.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Util.hh>
#include <libwebsockets.h>

#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
    /// A websocket server for gzweb
    ///
    /// # Plugin parameters
    ///
    /// * <publication_hz> : An integer that is the maximum publication
    /// hertz rate.
    ///
    /// * <port> : An integer that is websocket port.
    ///
    /// * <authorization_key> : A key used for authentication. If this is
    /// set, then a connection must provide the matching key using an "auth"
    /// call on the websocket. If the <admin_authorization_key> is set, then
    /// the connection can provide that key.
    ///
    /// * <admin_authorization_key> : An admin key used for authentication. If
    /// this is set, then a connection must provide the matching key using an
    /// "auth" call on the websocket. If the <authorization_key> is set, then
    /// the connection can provide that key.
    ///
    /// * <max_connections> : An integer that specifies the maximum number
    /// of active websocket connections. A value less than zero indicates an
    /// unlimited number, this is the default. A websocket client error
    /// code of 1008 along with a reason set to "max_connections" will be
    /// returned if a new connection is rejected due to the max connection
    /// threshold.
    ///
    /// * <ssl> : Element that contains SSL configuration. For testing
    ///           purposes you can create self-signed SSL certificates. Run
    ///
    /** \code{.sh}
        openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout server.key \
            -out server.cert
    */
    ///  Use "localhost" for the  "Common Name" question. If you are testing
    ///  with a browser, first navigate to "https://localhost:<port>" and
    ///  accept the self-signed certificate.
    ///
    ///     * <cert_file>: Child element of <ssl> that contains the path to
    ///                     the SSL certificate file.
    ///     * <private_key_file>: Child element of <ssl> that contains the path
    ///                           to SSL private key file.
    ///
    /// # Websocket Server Interface
    ///
    /// The websocket server listens for incoming requests and sends
    /// messages based on the requests.
    ///
    /// All messages on the websocket, incoming and outgoing, are structured
    /// in a frame that consists of four comma separated components:
    ///     1. `operation`: string,
    ///     2. `topic_name`: string,
    ///     3. `message_type`: string, and
    ///     4. `payload`: serialized data.
    ///
    /// The `operation` component is mandatory and must be one of:
    ///     1. "sub": Subscribe to the topic in the `topic_name` component,
    ///     2. "pub": Publish a message from the Gazebo Transport topic in
    ///               the `topic_name` component,
    ///     3. "topics": Get the list of available topics,
    ///     4. "topics-types": Get the list of available topics and their
    ///                        message types,
    ///     5. "protos": Get a string containing all the protobuf
    ///                  definitions, and
    ///     6. "particle_emitters": Get the list of particle emitters.
    ///                  definitions.
    ///     7. "unsub": Unsubscribe from the topic in the `topic_name` component
    ///     8. "asset": Get a file as a byte array from a running Gazebo
    ///                 server. Set the payload to the file URI that is
    ///                 being requested.
    ///     9. "worlds": Get world info.
    ///     10. "scene": Get scene info.
    ///     11. "image": Subscribe to an image in the `topic_name` component.
    ///     12. "throttle": Throttle a topic in the `topic_name` component by
    ///                 the rate in the `payload` component.
    ///     13. "req": Request a service, passing in the optional request
    ///                message. The payload should be a serialized
    ///                protobuf message. The response payload holds the
    ///                serialized protobuf response, if any.
    ///
    /// The `topic_name` component is mandatory for the "sub", "pub", "unsub",
    /// and "req" operations. If present, it must be the name of a Gazebo
    /// Transport topic.
    ///
    /// The `message_type` component is mandatory for the "pub" and "req"
    /// operations. If present it names the Gazebo Message type, such as
    /// "gz.msgs.Clock".
    ///
    /// The `payload` component is mandatory for the "pub" and "req"
    /// operations. If present, it contains a serialized string of a
    /// Gazebo Message.
    ///
    /// ## Example frames
    ///
    /// 1. Get the list of topics: `topics,,,`
    ///
    /// 2. Get the protobuf definitions: `protos,,,`
    ///
    /// 3. Subscribe to the "/clock" topic: `sub,/clock,,`
    ///
    /// 4. Websocket server publishing data on the "/clock" topic:
    ///    `pub,/clock,gz.msgs.Clock,<serialized_data>`
    ///
    /// # Example usage
    ///
    /// ## Websocket Server
    ///
    /// 1. Add the following to the top of an SDF file to include the websocket
    /// server system when launching a world.
    ///
    ///  <plugin name="gz::sim::WebsocketServer"
    ///      filename="gz-sim-websocket-server-system">
    ///
    ///    <!-- Publication Hz -->
    ///    <publication_hz>30</publication_hz>
    /// </plugin>
    ///
    /// 2. Launch your SDF world file, e.g.
    ///
    /// `gz sim -v 4 -s websocket.sdf`
    ///
    /// 3. Connect gzweb to the websocket server for web visualization.
    ///    An example is provided in `examples/scripts/websocket_server`
    ///
    class WebsocketServer
        : public System,
          public ISystemConfigure
    {
      /// \brief Constructor
      public: WebsocketServer() = default;

      /// \brief Destructor
      public: virtual ~WebsocketServer();

      /// Documentation inherited
      public: void Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_eventMgr) final;

      public: void Run();

      private: void OnWebsocketSubscribedMessage(const char *_data,
                   const size_t _size,
                   const gz::transport::MessageInfo &_info);

      /// \brief Callback when an image is received on a topic
      /// \param[in] _msg Image msg
      /// \param[in] _info gz transport message info
      private: void OnWebsocketSubscribedImageMessage(
          const gz::msgs::Image &_msg,
          const gz::transport::MessageInfo &_info);

      /// \brief Callback that is triggered when a new connection is
      /// established.
      /// \param[in] _socketId ID of the socket.
      public: void OnConnect(int _socketId);

      /// \brief Callback that is triggered when a connection ended.
      /// \param[in] _socketId ID of the socket.
      public: void OnDisconnect(int _socketId);

      /// \brief Handles incoming websocket messages
      /// \param[in] _socketId Id of the socket associated with the message.
      /// \param[in] _msg The incoming message.
      public: void OnMessage(int _socketId, const std::string _msg);

      /// \brief Check and update subscription count for a message type. If
      /// a client has more subscriptions to a topic of a specified type than
      /// the subscription limit, this will block subscription. On the other
      /// hand, for an unsubscription operation, the count is decremented.
      /// \param[in] _topic Topic to subscribe to or unsubscribe from
      /// \param[in] _socketId Connection socket id
      /// \param[in] _subscribe True for subscribe operation, false for
      /// unsubscribe operation
      /// \return True if the subscription count is incremented or decremented,
      /// and false to indicate the subscription limit has reached.
      public: bool UpdateMsgTypeSubscriptionCount(const std::string &_topic,
          int _socketId, bool _subscribe);

      /// \brief Handles asset requests.
      /// \param[in] _socketId Id of the socket associated with the message.
      /// \param[in] _frameParts The request message in frame parts.
      private: void OnAsset(int _socketId,
                   const std::vector<std::string> &_frameParts);

      /// \brief Handles service requests.
      /// \param[in] _socketId Id of the socket associated with the message.
      /// \param[in] _frameParts The request message in frame parts.
      private: void OnRequest(int _socketId,
                   const std::vector<std::string> &_frameParts);

      private: gz::transport::Node node;

      private: bool run = true;
      private: std::thread *thread = nullptr;
      private: struct lws_context *context = nullptr;

      private: std::vector<struct lws_protocols> protocols;
      private: class Connection
      {
        public: std::chrono::system_clock::time_point creationTime;
        public: std::list<std::unique_ptr<char []>> buffer;
        public: std::list<int> len;
        public: std::mutex mutex;

        public: bool authorized{false};

        /// \brief A map of topic name to outbound publish rate
        /// A value of 0 means unthrottled
        public: std::map<std::string, std::chrono::nanoseconds>
            topicPublishPeriods;

        /// \brief A map of topic name to timestamp of last published message
        /// for this connection
        public: std::map<std::string,
            std::chrono::time_point<std::chrono::steady_clock>> topicTimestamps;

        /// \brief The number of subscriptions of a msg type this connection
        /// has. The key is the msg type, e.g. gz.msgs.Image, and the
        /// value is the subscription count
        public: std::map<std::string, int> msgTypeSubscriptionCount;
      };

      private: void QueueMessage(Connection *_connection,
                   const char *_data, const size_t _size);

      public: std::mutex mutex;

      /// \brief A mutex used in the OnWebsocketSubscribedMessage
      /// function.
      public: std::mutex subscriptionMutex;

      /// \brief All of the websocket connections.
      public: std::map<int, std::unique_ptr<Connection>> connections;

      /// \brief All of the subscribed Gazebo topics.
      /// The key is the topic name, and the value is the set of websocket
      /// connections that have subscribed to the topic.
      public: std::map<std::string, std::set<int>> topicConnections;

      /// \brief The limit placed on the number of subscriptions per msg type
      /// for each connection. The key is the msg type, e.g.
      /// gz.msgs.Image, and the value is the subscription limit
      public: std::map<std::string, int> msgTypeSubscriptionLimit;

      /// \brief Run loop mutex.
      public: std::mutex runMutex;

      /// \brief Run loop condition variable.
      public: std::condition_variable runConditionVariable;

      /// \brief Number of pending messages. This is used to throttle the
      /// run loop.
      public: int messageCount{0};

      /// \brief The maximum number of connections. A negative number
      /// indicates no limit.
      public: int maxConnections{-1};

      /// \brief Time of last publication for each subscribed topic. The key
      /// is the topic name and the value the time of last publication.
      /// \sa publishPeriod.
      private: std::map<std::string,
               std::chrono::time_point<std::chrono::steady_clock>>
                 topicTimestamps;

      /// \brief The message queue size per connection. A negative number
      /// indicates no limit.
      public: int queueSizePerConnection{-1};

      /// \brief The set of valid operations. This enum must align with the
      /// `operations` member variable.
      private: enum Operation
               {
                 /// \brief Subscribe to a topic.
                 SUBSCRIBE = 0,

                 /// \brief Publish a message from a topic.
                 PUBLISH = 1,

                 /// \brief Get the list of topics.
                 TOPICS = 2,

                 /// \brief Get the protobuf definitions.
                 PROTOS = 3,

                 /// \brief Get an asset as a byte array.
                 ASSET = 4,

                 /// \brief Request a service
                 REQUEST = 5,
               };

      /// \brief The set of valid operations, in string form. These values
      /// can be sent in websocket message frames.
      /// These values must align with the `Operation` enum.
      private: std::vector<std::string> operations{
                 "sub", "pub", "topics", "protos", "asset", "req"};

      /// \brief Store publish headers for topics. This is here to improve
      /// performance. Keys are topic names and values are frame headers.
      private: std::map<std::string, std::string> publishHeaders;

      /// \brief Period at which messages will be published on the websocket
      /// for each subscribed topic.
      /// \sa topicTimestamps.
      private: std::chrono::nanoseconds publishPeriod;

      /// \brief Authorization key used to validate a web-socket connection.
      private: std::string authorizationKey;

      /// \brief Administrator authorization key used to validate a web-socket
      /// connection.
      private: std::string adminAuthorizationKey;
    };
}
}
}
}

#endif
