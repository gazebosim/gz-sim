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

#include <deque>
#include <mutex>
// #include <ignition/msgs/frame.pb.h>
// #include <map>
// #include <ignition/common/Profiler.hh>
// #include <ignition/plugin/Register.hh>
// #include <sdf/sdf.hh>

// #include "ignition/gazebo/Util.hh"

#include "Broker.hh"
#include "CommonTypes.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Helper class for managing bidirectional mapping of client IDs and
/// addresses. The class is not thread-safe, so callers must ensure that none
/// of the public methods get called simultaneously.
struct ClientIDs
{
  /// \brief Number of active clients for each address. This structure can be
  /// accessed by outer code for reading, but not for modification.
  std::unordered_map<std::string, size_t> numActiveClients;

  /// \brief Map of client IDs to addresses. This structure can be accessed by
  /// outer code for reading, but not for modification.
  std::unordered_map<ClientID, std::string> idToAddress;

  /// \brief Add a new client and generate its ID.
  /// \param _address Address of the client.
  /// \return ID of the client. This method should always succeed and return
  /// an ID different from invalidClientID.
  ClientID Add(const std::string& _address)
  {
    const auto clientId = this->NextID();
    this->idToAddress[clientId] = _address;
    if (this->numActiveClients.find(_address) == this->numActiveClients.end())
      this->numActiveClients[_address] = 0;
    this->numActiveClients[_address]++;
    return clientId;
  }

  /// \brief Unregister a client.
  /// \param _id ID of the client.
  /// \return Success of the unregistration. The method can fail e.g. when
  /// trying to unregister a client which has not been registered.
  bool Remove(const ClientID _id)
  {
    if (!this->Valid(_id))
      return false;
    this->numActiveClients[this->idToAddress[_id]]--;
    this->idToAddress.erase(_id);
    return true;
  }

  /// \brief Clear/reset the structure to be able to work as new.
  /// \note This cancels all registrations of all clients and resets the client
  /// ID numbering, so it is not valid to mix IDs of clients obtained before and
  /// after a Clear() call.
  void Clear()
  {
    this->numActiveClients.clear();
    this->idToAddress.clear();
    this->lastId = invalidClientId;
  }

  /// \brief Check validity of a client ID.
  /// \param _id ID to check.
  /// \return Whether a client with the given ID has been registered.
  bool Valid(const ClientID _id) const
  {
    return _id != invalidClientId &&
      this->idToAddress.find(_id) != this->idToAddress.end();
  }

  /// \brief Return an ID for a new client.
  /// \return The ID.
  private: ClientID NextID()
  {
    return ++this->lastId;
  }

  /// \brief Last ID given to a client.
  private: ClientID lastId {invalidClientId};
};

class ignition::gazebo::systems::BrokerPrivate
{
  /// \brief An Ignition Transport node for communications.
  public: ignition::transport::Node node;

  /// \brief Queue to store the incoming messages received from the clients.
  public: std::deque<msgs::Datagram> incomingMsgs;

  /// \brief Protect data from races.
  public: std::mutex mutex;

  /// \brief Information about the members of the team.
  public: TeamMembershipPtr team;

  /// \brief IDs of registered clients.
  public: ClientIDs clientIDs;

  /// \brief Buffer to store incoming data packets.
  // std::map<CommsAddress, ignition::msgs::Frame> incomingBuffer;

  /// \brief Buffer to store outgoing data packets.
  // std::map<CommsAddress, ignition::msgs::Frame> outgoingBuffer;
};

//////////////////////////////////////////////////
Broker::Broker()
    // : team(std::make_shared<TeamMembership_M>()),
    :  dataPtr(std::make_unique<BrokerPrivate>())
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
  // Advertise the service for registering addresses.
  if (!this->dataPtr->node.Advertise(kAddrRegistrationSrv,
                                     &Broker::OnAddrRegistration, this))
  {
    std::cerr << "Error advertising srv [" << kAddrRegistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the service for unregistering addresses.
  if (!this->dataPtr->node.Advertise(kAddrUnregistrationSrv,
                                     &Broker::OnAddrUnregistration, this))
  {
    std::cerr << "Error advertising srv [" << kAddrUnregistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the service for registering end points.
  if (!this->dataPtr->node.Advertise(kEndPointRegistrationSrv,
                                     &Broker::OnEndPointRegistration, this))
  {
    std::cerr << "Error advertising srv [" << kEndPointRegistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the service for unregistering end points.
  if (!this->dataPtr->node.Advertise(kEndPointUnregistrationSrv,
                                     &Broker::OnEndPointUnregistration, this))
  {
    std::cerr << "Error advertising srv [" << kEndPointUnregistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise a oneway service for centralizing all message requests.
  if (!this->dataPtr->node.Advertise(kBrokerSrv, &Broker::OnMessage, this))
  {
    std::cerr << "Error advertising srv [" << kBrokerSrv << "]" << std::endl;
    return;
  }

  std::cout << "Started communication broker in Ignition partition "
            << this->IgnPartition() << std::endl;
}

//////////////////////////////////////////////////
std::string Broker::IgnPartition() const
{
  return this->dataPtr->node.Options().Partition();
}

//////////////////////////////////////////////////
ClientID Broker::Register(const std::string &_clientAddress)
{
  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);
  auto kvp = this->dataPtr->team->find(_clientAddress);
  if (kvp == this->dataPtr->team->end())
  {
    auto newMember = std::make_shared<TeamMember>();

    // Name and address are the same.
    newMember->address = _clientAddress;
    newMember->name = _clientAddress;

    (*this->dataPtr->team)[_clientAddress] = newMember;
  }

  const auto clientId = this->dataPtr->clientIDs.Add(_clientAddress);

  return clientId;
}

//////////////////////////////////////////////////
bool Broker::Unregister(const ClientID _clientId)
{
  if (!this->dataPtr->clientIDs.Valid(_clientId))
  {
    std::cerr << "Broker::Unregister() error: Client ID [" << _clientId
              << "] is invalid." << std::endl;
    return false;
  }

  bool success = true;

  // std::unordered_set<subt::communication_broker::EndpointID> endpointIds;
  // {
  //   // make a copy because Unbind() calls will alter the structure
  //   std::lock_guard<std::mutex> lk(this->mutex);
  //   endpointIds = this->dataPtr->endpointIDs.clientIdToEndpointIds[_clientId];
  // }

  // for (const auto endpointId : endpointIds)
  //   success = success && this->Unbind(endpointId);

  {
    std::lock_guard<std::mutex> lk(this->dataPtr->mutex);

    const auto& clientAddress = this->dataPtr->clientIDs.idToAddress[_clientId];
    success = success && this->dataPtr->clientIDs.Remove(_clientId);

    if (this->dataPtr->clientIDs.numActiveClients[clientAddress] == 0u)
      this->dataPtr->team->erase(clientAddress);
  }

  return success;
}

//////////////////////////////////////////////////
bool Broker::DispatchMessages()
{
  return true;
}

/////////////////////////////////////////////////
bool Broker::OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                ignition::msgs::UInt32 &_rep)
{
  const auto &address = _req.data();

  const ClientID result = this->Register(address);

  _rep.set_data(result);

  return result != invalidClientId;
}

/////////////////////////////////////////////////
bool Broker::OnAddrUnregistration(const ignition::msgs::UInt32 &_req,
                                  ignition::msgs::Boolean &_rep)
{
  uint32_t clientId = _req.data();

  bool result;

  result = this->Unregister(clientId);

  _rep.set_data(result);

  return result;
}

/////////////////////////////////////////////////
void Broker::OnMessage(const ignition::msgs::Datagram &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->dataPtr->mutex);

  // Save the message.
  this->dataPtr->incomingMsgs.push_back(_req);
}

// //////////////////////////////////////////////////
// CommsBroker::CommsBroker()
//   : dataPtr(std::make_unique<CommsBrokerPrivate>())
// {
// }

// //////////////////////////////////////////////////
// void CommsBroker::Configure(const Entity &_entity,
//     const std::shared_ptr<const sdf::Element> &_sdf,
//     EntityComponentManager &/*_ecm*/,
//     EventManager &/*_eventMgr*/)
// {
// }

// //////////////////////////////////////////////////
// void CommsBroker::PreUpdate(
//     const ignition::gazebo::UpdateInfo &_info,
//     ignition::gazebo::EntityComponentManager &_ecm)
// {
//   IGN_PROFILE("CommsBroker::PreUpdate");
// }

// IGNITION_ADD_PLUGIN(CommsBroker,
//                     ignition::gazebo::System,
//                     CommsBroker::ISystemConfigure,
//                     CommsBroker::ISystemPreUpdate)

// IGNITION_ADD_PLUGIN_ALIAS(CommsBroker,
//                           "ignition::gazebo::systems::CommsBroker")
