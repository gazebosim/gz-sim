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

/// \file Broker.hh
/// \brief Broker for handling message delivery among robots.
#ifndef IGNITION_GAZEBO_SYSTEMS_BROKER_HH_
#define IGNITION_GAZEBO_SYSTEMS_BROKER_HH_

#include <ignition/msgs/datagram.pb.h>

#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "CommonTypes.hh"

// #include <subt_communication_broker/protobuf/endpoint_registration.pb.h>
// #include <subt_communication_broker/protobuf/neighbor_m.pb.h>
// #include <subt_communication_model/subt_communication_model.h>
// #include <subt_rf_interface/subt_rf_interface.h>

//#include "CommonTypes"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

typedef
std::function<std::tuple<bool, ignition::math::Pose3<double>, double>(
  const std::string& name)> pose_update_function;

  /// \brief Stores information about a client broker.
  struct BrokerClientInfo
  {
    /// \brief Address of the client. E.g.: 192.168.2.2
    std::string address;
  };

  /// \def EndPoints_M
  /// \brief Map of endpoints
  using EndPoints_M = std::map<std::string, std::vector<BrokerClientInfo>>;

  struct BrokerPrivate;

  /// \brief Store messages, and exposes an API for registering new clients,
  /// bind to a particular address, push new messages or get the list of
  /// messages already stored in the queue.
  class Broker
  {
    /// \brief Constructor.
    public: Broker();

    /// \brief Destructor.
    public: virtual ~Broker();

    /// \brief Start handling services
    ///
    /// This function allows us to wait to advertise capabilities to
    /// clients until the broker has been entirely initialized. I.e.,
    /// after SetDefaultRadioConfiguration() has been called.
    public: void Start();

    /// \brief Get the Ignition partition this broker is running in.
    /// \return The partition name.
    public: std::string IgnPartition() const;

    /// \brief Register a new client for message handling. Multiple clients for
    /// the same address are allowed even from a single process.
    /// \param[in] _clientAddress Address of the client.
    /// \return ID of the client (should be later used for unregistration).
    /// If the returned ID is invalidClientId, the registration failed.
    public: ClientID Register(const std::string &_clientAddress);

    /// \brief Unregister a client and unbind from all its endpoints.
    /// \param[in] _clientId The ID received from the Register() call.
    /// \return True if the operation succeeded or false otherwise (if there is
    /// no client registered for this ID or the ID is invalid).
    public: bool Unregister(ClientID _clientId);

    /// \brief Dispatch all incoming messages.
    public: bool DispatchMessages();

    /// \brief Callback executed when a new registration request is received.
    /// \param[in] _req The address contained in the request.
    /// \param[out] _rep An ID of the registered client. This ID should be used
    /// when unregistering this client. If registration failed, invalidClientId
    /// value is returned.
    private: bool OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                     ignition::msgs::UInt32 &_rep);

    /// \brief Callback executed when a new unregistration request is received.
    /// \param[in] _req ID of the client to unregister.
    /// \param[out] _rep The result of the service. True when the unregistration
    /// went OK or false otherwise (e.g.: the ID wasn't registered).
    private: bool OnAddrUnregistration(const ignition::msgs::UInt32 &_req,
                                       ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new endpoint registration request is
    /// received.
    /// \param[in] _req The endpoint to register together with ID of the client
    /// to which this registration belongs.
    /// \param[out] _rep An ID of the endpoint. This ID should be used
    /// when unregistering this endpoint. If registration failed,
    /// invalidEndpointId value is returned.
    private: bool OnEndPointRegistration(
      const ignition::msgs::EndpointRegistration &_req,
      ignition::msgs::UInt32 &_rep);

    /// \brief Callback executed when a new endpoint unregistration request is
    /// received.
    /// \param[in] _req ID of the endpoint to unregister.
    /// \param[out] _rep The result of the service. True when the unregistration
    /// went OK or false otherwise (e.g.: the ID wasn't registered).
    private: bool OnEndPointUnregistration(
      const ignition::msgs::UInt32 &_req,
      ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new request is received.
    /// \param[in] _req The datagram contained in the request.
    private: void OnMessage(const ignition::msgs::Datagram &_req);

    /// \brief Handle reset.
    // public: void Reset();

    // /// \brief Get a mutator to the team.
    // /// \return A mutator to the team.
    // public: TeamMembershipPtr Team();

    // /// \brief Send a message to each member
    // /// with its updated neighbors list.
    // public: void NotifyNeighbors();

    /// \brief Dispatch all incoming messages.
    // public: bool DispatchMessages();

   //  /// \brief This method associates an endpoint with a broker client and its
   //  /// address. An endpoint is constructed as an address followed by ':',
   //  /// followed by the port. E.g.: "192.168.1.5:8000" is a valid endpoint.
   //  /// \param[in] _clientId ID of a registered client.
   //  /// \param[in] _endpoint End point requested to bind.
   //  /// \return ID that should be used for unbinding the endpoint. If
   //  /// invalidEndpointId is returned, the binding request failed (e.g. due to
   //  /// wrong endpoint specification, or if the _clientId is wrong).
   //  public: EndpointID Bind(ClientID _clientId, const std::string &_endpoint);

   //  /// \brief This method cancels the association between a client and an
   //  /// endpoint. When all equal endpoints on the same client are unbound,
   //  /// the client will stop receiving messages for the given endpoint.
   //  /// \param[in] _endpointId ID of the endpoint to unbind. It has to be an ID
   //  /// received from a previous call to Bind().
   //  /// \return Whether the unbind was successful. It may fail e.g. when the
   //  /// given ID is invalid and the broker doesn't know it.
   //  public: bool Unbind(EndpointID _endpointId);

   //  /// \brief Register a new client for message handling. Multiple clients for
   //  /// the same address are allowed even from a single process.
   //  /// \param[in] _clientAddress Address of the client.
   //  /// \return ID of the client (should be later used for unregistration).
   //  /// If the returned ID is invalidClientId, the registration failed.
   //  public: ClientID Register(const std::string &_clientAddress);

   //  /// \brief Unregister a client and unbind from all its endpoints.
   //  /// \param[in] _clientId The ID received from the Register() call.
   //  /// \return True if the operation succeeded or false otherwise (if there is
   //  /// no client registered for this ID or the ID is invalid).
   //  public: bool Unregister(ClientID _clientId);

   //  /// \brief Set the radio configuration for address
   //  /// \param[in] address
   //  /// \param[in] radio_configuration
   //  public: void SetRadioConfiguration(const std::string& address,
   //     communication_model::radio_configuration config);

   //  /// \brief Set the radio configuration
   //  /// \param[in] radio_configuration
   //  public: void SetDefaultRadioConfiguration(
   //      communication_model::radio_configuration config);

   //  /// \brief Set the communication function handle to use
   //  /// \param[in] f Function that evaluates transmission of bytes across the
   //  ///            communication channel.
   //  public: void SetCommunicationFunction(
   //      communication_model::communication_function f);

   //  /// \brief Set the function to be used for updating pose
   //  /// \param[in] f Function that finds pose based on name
   //  public: void SetPoseUpdateFunction(pose_update_function f);

   //  /// \brief Callback executed when a new registration request is received.
   //  /// \param[in] _req The address contained in the request.
   //  /// \param[out] _rep An ID of the registered client. This ID should be used
   //  /// when unregistering this client. If registration failed, invalidClientId
   //  /// value is returned.
   //  private: bool OnAddrRegistration(const ignition::msgs::StringMsg &_req,
   //                                   ignition::msgs::UInt32 &_rep);

   //  /// \brief Callback executed when a new unregistration request is received.
   //  /// \param[in] _req ID of the client to unregister.
   //  /// \param[out] _rep The result of the service. True when the unregistration
   //  /// went OK or false otherwise (e.g.: the ID wasn't registered).
   //  private: bool OnAddrUnregistration(const ignition::msgs::UInt32 &_req,
   //                                     ignition::msgs::Boolean &_rep);

   //  /// \brief Callback executed when a new endpoint registration request is
   //  /// received.
   //  /// \param[in] _req The endpoint to register together with ID of the client
   //  /// to which this registration belongs.
   //  /// \param[out] _rep An ID of the endpoint. This ID should be used
   //  /// when unregistering this endpoint. If registration failed,
   //  /// invalidEndpointId value is returned.
   //  private: bool OnEndPointRegistration(
   //    const subt::msgs::EndpointRegistration &_req,
   //    ignition::msgs::UInt32 &_rep);

   //  /// \brief Callback executed when a new endpoint unregistration request is
   //  /// received.
   //  /// \param[in] _req ID of the endpoint to unregister.
   //  /// \param[out] _rep The result of the service. True when the unregistration
   //  /// went OK or false otherwise (e.g.: the ID wasn't registered).
   //  private: bool OnEndPointUnregistration(
   //    const ignition::msgs::UInt32 &_req,
   //    ignition::msgs::Boolean &_rep);

   //  /// \brief Callback executed when a new request is received.
   //  /// \param[in] _req The datagram contained in the request.
   //  private: void OnMessage(const subt::msgs::Datagram &_req);

   //  /// \brief Queue to store the incoming messages received from the clients.
   //  protected: std::deque<msgs::Datagram> incomingMsgs;

   //  /// \brief List of bound endpoints. The key is an endpoint and the
   //  /// value is the vector of clients bounded on that endpoint.
   //  protected: EndPoints_M endpoints;

   //  /// \brief Information about the members of the team.
   //  protected: TeamMembershipPtr team;

   //  /// \brief An Ignition Transport node for communications.
   //  private: ignition::transport::Node node;

   //  /// \brief The publisher for notifying neighbor updates.
   //  private: ignition::transport::Node::Publisher neighborPub;

   //  /// \brief Protect data from races.
   //  private: std::mutex mutex;

   //  /// \brief Function handle for evaluating communication
   //  private: subt::communication_model::communication_function
   //  communication_function;

   // private:
   //  /// \brief Default radio configuration
   //  subt::communication_model::radio_configuration default_radio_configuration;

   //  /// \brief Pose update function
   // private: pose_update_function pose_update_f;

   /// \brief Private definitions and data
   private: std::unique_ptr<BrokerPrivate> dataPtr;
  };
  }
}
}
}

#endif
