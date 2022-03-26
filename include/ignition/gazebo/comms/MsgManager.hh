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

#ifndef IGNITION_GAZEBO_MSGMANAGER_HH_
#define IGNITION_GAZEBO_MSGMANAGER_HH_

#include <deque>
#include <map>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>
#include <ignition/utils/ImplPtr.hh>
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace msgs
{
  // Forward declarations.
  class Dataframe;
}
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace comms
{

/// \brief ToDo.
struct MsgContent
{
  /// \brief Queue of inbound messages.
  public: std::deque<std::shared_ptr<msgs::Dataframe>> inboundMsgs;

  /// \brief Queue of outbound messages.
  public: std::deque<std::shared_ptr<msgs::Dataframe>> outboundMsgs;

  /// \brief A map where the key is the topic subscribed to this address and
  /// the value is a publisher to reach that topic.
  public: std::map<std::string,
                   ignition::transport::Node::Publisher> subscriptions;

  /// \brief Model name associated to this address.
  public: std::string modelName;
};

// class MsgManagerData
// {
//   /// \brief Default constructor.
//   public: MsgManagerData();

//   /// \brief Destructor.
//   public: virtual ~MsgManagerData();

// public:

//   /// \brief Private data pointer.
//   IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
// };

/// \brief ToDo.
class MsgManager
{
  /// \brief Default constructor.
  public: MsgManager();

  /// \brief Destructor.
  public: virtual ~MsgManager();

  /// \brief Add a new subscriber
  /// \param[in] _address The subscriber address.
  /// \param[in] _modelName The model name.
  /// \param[in] _topic The subscriber topic.
  public: void AddSubscriber(const std::string &_address,
                             const std::string &_modelName,
                             const std::string &_topic);

  /// \brief Add a new message to the inbound queue.
  /// \param[in] _address The destination address.
  /// \param[in] _msg The message.
  public: void AddInbound(const std::string &_address,
                          const std::shared_ptr<msgs::Dataframe> &_msg);

  /// \brief Add a new message to the outbound queue.
  /// \param[in] _address The sender address.
  /// \param[in] _msg The message.
  public: void AddOutbound(const std::string &_address,
                           const std::shared_ptr<msgs::Dataframe> &_msg);

  /// \brief Remove an existing subscriber.
  /// \param The subscriber address.
  /// \param The Subscriber topic.
  /// \return True if the subscriber was removed or false otherwise.
  public: bool RemoveSubscriber(const std::string &_address,
                                const std::string &_topic);

  /// \brief Remove a message from the inbound queue.
  /// \param[in] _address The destination address.
  /// \param[in] _Msg Message pointer to remove.
  public: void RemoveInbound(const std::string &_address,
                             const std::shared_ptr<msgs::Dataframe> &_msg);

  /// \brief Remove a message from the outbound queue.
  /// \param[in] _address The sender address.
  /// \param[in] _msg Message pointer to remove.
  public: void RemoveOutbound(const std::string &_address,
                              const std::shared_ptr<msgs::Dataframe> &_msg);

  /// \brief This function delivers all the messages in the inbound queue to
  /// the appropriate subscribers. This function also clears the inbound queue.
  public: void DeliverMsgs();

  /// \brief Get a mutable reference to the data containing subscriptions and
  /// data queues.
  /// \return A mutable reference to the data.
  public: std::map<std::string, MsgContent> &Data();

  /// \brief Get a copy of the data structure containing subscriptions and data
  /// queues.
  /// \return A copy of the data.
  public: std::map<std::string, MsgContent> Copy();

  /// \brief Set the data structure containing subscriptions and data queues.
  /// \param[in] _newContent New content to be set.
  public: void Set(std::map<std::string, MsgContent> &_newContent);

  /// \brief Private data pointer.
  IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
}
}
}

#endif
