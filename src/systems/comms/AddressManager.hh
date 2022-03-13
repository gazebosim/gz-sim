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

#ifndef IGNITION_GAZEBO_SYSTEMS_ADDRESSMANAGER_HH_
#define IGNITION_GAZEBO_SYSTEMS_ADDRESSMANAGER_HH_

#include <ignition/msgs/datagram.pb.h>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utils/ImplPtr.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

struct AddressContent
{
  /// \brief Queue of inbound messages.
  public: std::deque<std::shared_ptr<msgs::Datagram>> inboundMsgs;

  /// \brief Queue of outbound messages.
  public: std::deque<std::shared_ptr<msgs::Datagram>> outboundMsgs;

  /// \brief A map where the key is the topic subscribed to this address and
  /// the value is a publisher to reach that topic.
  public: std::map<std::string,
                   ignition::transport::Node::Publisher> subscriptions;
};

/// \brief
class AddressManager
{
  /// \brief Default constructor.
  public: AddressManager();

  /// \brief Destructor.
  public: virtual ~AddressManager();

  /// \brief
  public: void AddSubscriber(const std::string &_address,
                             const std::string &_topic);

  /// \brief
  // public: std::unordered_set<std::string> Subscribers(
  //   const std::string &_address);

  /// \brief
  public: void AddInbound(const std::string &_address,
                          const std::shared_ptr<msgs::Datagram> &_msg);

  /// \brief
  public: void AddOutbound(const std::string &_address,
                           const std::shared_ptr<msgs::Datagram> &_msg);

  /// \brief
  public: bool RemoveSubscriber(const std::string &_address,
                                const std::string &_topic);

  /// \brief
  public: void RemoveInbound(const std::string &_address,
                             const std::shared_ptr<msgs::Datagram> &_msg);

  /// \brief
  public: void RemoveOutbound(const std::string &_address,
                              const std::shared_ptr<msgs::Datagram> &_msg);

  /// \brief ToDo.
  public: void DeliverMsgs();

  /// \brief ToDo.
  public: std::map<std::string, AddressContent> &Data();

  /// \brief ToDo.
  public: std::map<std::string, AddressContent> Copy();

  /// \brief ToDo.
  public: void Set(std::map<std::string, AddressContent> &_newContent);

  /// \brief Private data pointer.
  IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
}
}
}

#endif
