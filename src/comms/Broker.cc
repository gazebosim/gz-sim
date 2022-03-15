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

#include <ignition/msgs/datagram.pb.h>

#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_set>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/comms/CommonTypes.hh"
#include "ignition/gazebo/comms/MsgManager.hh"
#include "ignition/gazebo/Util.hh"

/// \brief Private Broker data class.
class ignition::gazebo::comms::Broker::Implementation
{
  /// \brief An Ignition Transport node for communications.
  public: ignition::transport::Node node;

  /// \brief The message manager.
  public: MsgManager data;

  /// \brief Protect data from races.
  public: std::mutex mutex;
};

using namespace ignition;
using namespace gazebo;
using namespace comms;

//////////////////////////////////////////////////
Broker::Broker()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Broker::~Broker()
{
  // cannot use default destructor because of dataPtr
}

//////////////////////////////////////////////////
void Broker::Start()
{
  // Advertise the service for binding addresses.
  if (!this->dataPtr->node.Advertise(kAddrBindSrv, &Broker::OnBind, this))
  {
    ignerr << "Error advertising srv [" << kAddrBindSrv << "]" << std::endl;
    return;
  }

  // Advertise the service for unbinding addresses.
  if (!this->dataPtr->node.Advertise(kAddrUnbindSrv, &Broker::OnUnbind, this))
  {
    ignerr << "Error advertising srv [" << kAddrUnbindSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the topic for receiving data messages.
  if (!this->dataPtr->node.Subscribe(kBrokerTopic, &Broker::OnMsg, this))
  {
    ignerr << "Error subscribing to topic [" << kBrokerTopic << "]"
           << std::endl;
  }
}

//////////////////////////////////////////////////
void Broker::OnBind(const ignition::msgs::StringMsg_V &_req)
{
  auto count = _req.data_size();
  if (count != 2)
    ignerr << "Receive incorrect number of arguments. "
           << "Expecting 2 and receive " << count << std::endl;

  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);
  this->dataPtr->data.AddSubscriber(_req.data(0), _req.data(1));

  ignmsg << "Address [" << _req.data(0) << "] bound on topic ["
         << _req.data(1) << "]" << std::endl;
}

//////////////////////////////////////////////////
void Broker::OnUnbind(const ignition::msgs::StringMsg_V &_req)
{
  auto count = _req.data_size();
  if (count != 2)
    ignerr << "Receive incorrect number of arguments. "
           << "Expecting 2 and receive " << count << std::endl;

  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);
  this->dataPtr->data.RemoveSubscriber(_req.data(0), _req.data(1));

  ignmsg << "Address [" << _req.data(0) << "] unbound on topic ["
         << _req.data(1) << "]" << std::endl;
}

//////////////////////////////////////////////////
void Broker::OnMsg(const ignition::msgs::Datagram &_msg)
{
  // Place the message in the outbound queue of the sender.
  auto msgPtr = std::make_shared<ignition::msgs::Datagram>(_msg);

  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);
  this->dataPtr->data.AddOutbound(_msg.src_address(), msgPtr);
}

//////////////////////////////////////////////////
void Broker::DeliverMsgs()
{
  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);
  this->dataPtr->data.DeliverMsgs();
}

//////////////////////////////////////////////////
MsgManager &Broker::Data()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
void Broker::Lock()
{
  this->dataPtr->mutex.lock();
}

//////////////////////////////////////////////////
void Broker::Unlock()
{
  this->dataPtr->mutex.unlock();
}
