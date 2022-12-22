/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_SIM_MSGMANAGER_HH_
#define GZ_SIM_MSGMANAGER_HH_

#include <gz/msgs/dataframe.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include <gz/transport/Node.hh>
#include <gz/utils/ImplPtr.hh>
#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace comms
{
/// \brief A queue of message pointers.
using DataQueue = std::deque<msgs::DataframeSharedPtr>;

/// \brief A map where the key is the topic subscribed to an address and
/// the value is a publisher to reach that topic.
using SubscriptionHandler =
  std::unordered_map<std::string, transport::Node::Publisher>;

/// \brief All the information associated to an address.
struct AddressContent
{
  /// \brief Queue of inbound messages.
  public: DataQueue inboundMsgs;

  /// \brief Queue of outbound messages.
  public: DataQueue outboundMsgs;

  /// \brief Subscribers.
  public: SubscriptionHandler subscriptions;

  /// \brief Model name associated to this address.
  public: std::string modelName;

  /// \brief Entity of the model associated to this address.
  public: sim::Entity entity;
};

/// \brief A map where the key is an address and the value is all the
/// information associated to each address (subscribers, queues, ...).
using Registry = std::unordered_map<std::string, AddressContent>;

/// \brief Class to handle messages and subscriptions.
class GZ_SIM_VISIBLE MsgManager
{
  /// \brief Default constructor.
  public: MsgManager();

  /// \brief Add a new subscriber. It's possible to associate multiple topics
  /// to the same address/model pair. However, the same address cannot be
  /// attached to multiple models. When all the subscribers are removed, it's
  /// posible to bind to this address using a different model.
  /// \param[in] _address The subscriber address.
  /// \param[in] _modelName The model name.
  /// \param[in] _topic The subscriber topic.
  /// \return True if the subscriber was successfully added or false otherwise.
  public: bool AddSubscriber(const std::string &_address,
                             const std::string &_modelName,
                             const std::string &_topic);

  /// \brief Add a new message to the inbound queue.
  /// \param[in] _address The destination address.
  /// \param[in] _msg The message.
  public: void AddInbound(const std::string &_address,
                          const msgs::DataframeSharedPtr &_msg);

  /// \brief Add a new message to the outbound queue.
  /// \param[in] _address The sender address.
  /// \param[in] _msg The message.
  public: void AddOutbound(const std::string &_address,
                           const msgs::DataframeSharedPtr &_msg);

  /// \brief Remove an existing subscriber.
  /// \param[in] _address The subscriber address.
  /// \param[in] _topic The Subscriber topic.
  /// \return True if the subscriber was removed or false otherwise.
  public: bool RemoveSubscriber(const std::string &_address,
                                const std::string &_topic);

  /// \brief Remove a message from the inbound queue.
  /// \param[in] _address The destination address.
  /// \param[in] _Msg Message pointer to remove.
  /// \return True if the message was removed or false otherwise.
  public: bool RemoveInbound(const std::string &_address,
                             const msgs::DataframeSharedPtr &_msg);

  /// \brief Remove a message from the outbound queue.
  /// \param[in] _address The sender address.
  /// \param[in] _msg Message pointer to remove.
  /// \return True if the message was removed or false otherwise.
  public: bool RemoveOutbound(const std::string &_address,
                              const msgs::DataframeSharedPtr &_msg);

  /// \brief This function delivers all the messages in the inbound queue to
  /// the appropriate subscribers. This function also clears the inbound queue.
  public: void DeliverMsgs();

  /// \brief Get an inmutable reference to the data containing subscriptions and
  /// data queues.
  /// \return A const reference to the data.
  public: const Registry &DataConst() const;

  /// \brief Get a mutable reference to the data containing subscriptions and
  /// data queues.
  /// \return A mutable reference to the data.
  public: Registry &Data();

  /// \brief Get a copy of the data structure containing subscriptions and data
  /// queues.
  /// \return A copy of the data.
  public: Registry Copy() const;

  /// \brief Set the data structure containing subscriptions and data queues.
  /// \param[in] _newContent New content to be set.
  public: void Set(const Registry &_newContent);

  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
}
}
}

#endif
