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

#include <ignition/msgs/dataframe.pb.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include <ignition/transport/Node.hh>
#include <ignition/utils/ImplPtr.hh>
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/comms/MsgManager.hh"

/// \brief Private MsgManager data class.
class ignition::gazebo::comms::MsgManager::Implementation
{
  /// \brief Buffer to store the content associated to each address.
  /// The key is an address. The value contains all the information associated
  /// to the address.
  public: Registry data;

  /// \brief An Ignition Transport node for communications.
  public: ignition::transport::Node node;
};

using namespace ignition;
using namespace gazebo;
using namespace comms;

//////////////////////////////////////////////////
MsgManager::MsgManager()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
MsgManager::~MsgManager()
{
  // cannot use default destructor because of dataPtr
}

//////////////////////////////////////////////////
void MsgManager::AddSubscriber(const std::string &_address,
                               const std::string &_modelName,
                               const std::string &_topic)
{
  this->dataPtr->data[_address].modelName = _modelName;

  ignition::transport::Node::Publisher publisher =
    this->dataPtr->node.Advertise<ignition::msgs::Dataframe>(_topic);

  this->dataPtr->data[_address].subscriptions[_topic] = publisher;
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
  if (this->dataPtr->data.find(_address) == this->dataPtr->data.end())
    return false;

  return this->dataPtr->data[_address].subscriptions.erase(_topic) > 0;
}

//////////////////////////////////////////////////
void MsgManager::RemoveInbound(const std::string &_address,
                               const msgs::DataframeSharedPtr &_msg)
{
  if (this->dataPtr->data.find(_address) == this->dataPtr->data.end())
    return;

  auto &q = this->dataPtr->data[_address].inboundMsgs;
  q.erase(std::remove(q.begin(), q.end(), _msg), q.end());
}

//////////////////////////////////////////////////
void MsgManager::RemoveOutbound(const std::string &_address,
                                const msgs::DataframeSharedPtr &_msg)
{
  if (this->dataPtr->data.find(_address) == this->dataPtr->data.end())
    return;

  auto &q = this->dataPtr->data[_address].outboundMsgs;
  q.erase(std::remove(q.begin(), q.end(), _msg), q.end());
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
const Registry &MsgManager::DataConst()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
Registry &MsgManager::Data()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
Registry MsgManager::Copy()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
void MsgManager::Set(Registry &_newContent)
{
  this->dataPtr->data = _newContent;
}
