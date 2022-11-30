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

#include <gz/msgs/dataframe.pb.h>

#include <algorithm>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/utils/ImplPtr.hh>
#include "gz/sim/config.hh"
#include "gz/sim/comms/MsgManager.hh"

/// \brief Private MsgManager data class.
class gz::sim::comms::MsgManager::Implementation
{
  /// \brief Buffer to store the content associated to each address.
  /// The key is an address. The value contains all the information associated
  /// to the address.
  public: Registry data;

  /// \brief A Gazebo Transport node for communications.
  public: std::unique_ptr<gz::transport::Node> node;
};

using namespace gz;
using namespace sim;
using namespace comms;

//////////////////////////////////////////////////
MsgManager::MsgManager()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->node = std::make_unique<gz::transport::Node>();
}

//////////////////////////////////////////////////
bool MsgManager::AddSubscriber(const std::string &_address,
                               const std::string &_modelName,
                               const std::string &_topic)
{
  auto it = this->dataPtr->data.find(_address);
  if (it != this->dataPtr->data.end())
  {
    if (!it->second.modelName.empty() && it->second.modelName != _modelName)
    {
      gzerr << "AddSubscriber() error: Address already attached to a different"
             << " model" << std::endl;
      return false;
    }
  }
  this->dataPtr->data[_address].modelName = _modelName;

  gz::transport::Node::Publisher publisher =
    this->dataPtr->node->Advertise<gz::msgs::Dataframe>(_topic);

  this->dataPtr->data[_address].subscriptions[_topic] = publisher;
  return true;
}

//////////////////////////////////////////////////
void MsgManager::AddInbound(const std::string &_address,
                            const msgs::DataframeSharedPtr &_msg)
{
  this->dataPtr->data[_address].inboundMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void MsgManager::AddOutbound(const std::string &_address,
                             const msgs::DataframeSharedPtr &_msg)
{
  this->dataPtr->data[_address].outboundMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
bool MsgManager::RemoveSubscriber(const std::string &_address,
                                  const std::string &_topic)
{
  auto it = this->dataPtr->data.find(_address);
  if (it == this->dataPtr->data.end())
  {
    gzerr << "RemoveSubscriber() error: Unable to find address ["
           << _address << "]" << std::endl;
    return false;
  }

  auto res = it->second.subscriptions.erase(_topic) > 0;

  // It there are no subscribers we clear the model name. This way the address
  // can be bound to a separate model. We also clear the queues.
  if (it->second.subscriptions.empty())
    it->second.modelName = "";

  return res;
}

//////////////////////////////////////////////////
bool MsgManager::RemoveInbound(const std::string &_address,
                               const msgs::DataframeSharedPtr &_msg)
{
  auto it = this->dataPtr->data.find(_address);
  if (it == this->dataPtr->data.end())
  {
    gzerr << "RemoveInbound() error: Unable to find address ["
           << _address << "]" << std::endl;
    return false;
  }

  auto &q = it->second.inboundMsgs;
  q.erase(std::remove(q.begin(), q.end(), _msg), q.end());
  return true;
}

//////////////////////////////////////////////////
bool MsgManager::RemoveOutbound(const std::string &_address,
                                const msgs::DataframeSharedPtr &_msg)
{
  auto it = this->dataPtr->data.find(_address);
  if (it == this->dataPtr->data.end())
  {
    gzerr << "RemoveOutbound() error: Unable to find address ["
           << _address << "]" << std::endl;
    return false;
  }

  auto &q = it->second.outboundMsgs;
  q.erase(std::remove(q.begin(), q.end(), _msg), q.end());
  return true;
}

//////////////////////////////////////////////////
void MsgManager::DeliverMsgs()
{
  for (auto & [address, content] : this->dataPtr->data)
  {
    // Reference to the inbound queue for this address.
    auto &inbound = content.inboundMsgs;

    // All these messages need to be delivered.
    for (auto &msg : inbound)
    {
      // Use the publisher associated to the destination address.
      for (auto & [topic, publisher] : content.subscriptions)
        publisher.Publish(*msg);
    }

    content.inboundMsgs.clear();
  }
}

//////////////////////////////////////////////////
const Registry &MsgManager::DataConst() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
Registry &MsgManager::Data()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
Registry MsgManager::Copy() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
void MsgManager::Set(const Registry &_newContent)
{
  this->dataPtr->data = _newContent;
}
