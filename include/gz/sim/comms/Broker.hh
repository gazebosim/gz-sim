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

#ifndef GZ_SIM_BROKER_HH_
#define GZ_SIM_BROKER_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/config.hh"

namespace gz
{
namespace msgs
{
  // Forward declarations.
  class Boolean;
  class Dataframe;
  class StringMsg_V;
}
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace comms
{
  // Forward declarations.
  class MsgManager;

  /// \brief A class to store messages to be delivered using a comms model.
  /// This class should be used in combination with a specific comms model that
  /// implements the ICommsModel interface.
  /// \sa ICommsModel.hh
  /// The broker maintains two queues: inbound and outbound. When a client
  /// sends a communication request, we'll store it in the outbound queue of
  /// the sender's address. When the comms model decides that a message needs
  /// to be delivered to one of the destination, it'll be stored in the inbound
  /// queue of the destination's address.
  ///
  /// The main goal of this class is to receive the comms requests, stamp the
  /// time, and place them in the appropriate outbound queue, as well as deliver
  /// the messages that are in the inbound queues.
  ///
  /// The instance of the comms model is responsible for moving around the
  /// messages from the outbound queues to the inbound queues.
  ///
  /// The broker can be configured with the following SDF parameters:
  ///
  /// * Optional parameters:
  /// <broker> Element used to capture the broker parameters. This block can
  ///          contain any of the next parameters:
  ///    <messages_topic>: Topic name where the broker receives all the incoming
  ///                      messages. The default value is "/broker/msgs"
  ///    <bind_service>: Service name used to bind an address.
  ///                    The default value is "/broker/bind"
  ///    <unbind_service>: Service name used to unbind from an address.
  ///                      The default value is "/broker/unbind"
  ///
  /// Here's an example:
  /// <plugin
  ///   filename="gz-sim-perfect-comms-system"
  ///   name="gz::sim::systems::PerfectComms">
  ///   <broker>
  ///     <messages_topic>/broker/inbound</messages_topic>
  ///     <bind_service>/broker/bind_address</bind_service>
  ///     <unbind_service>/broker/unbind_address</unbind_service>
  ///   </broker>
  /// </plugin>
  class GZ_SIM_VISIBLE Broker
  {
    /// \brief Constructor.
    public: Broker();

    /// \brief Configure the broker via SDF.
    /// \param[in] _sdf The SDF Element associated with the broker parameters.
    public: void Load(std::shared_ptr<const sdf::Element> _sdf);

    /// \brief Start handling comms services.
    ///
    /// This function allows us to wait to advertise capabilities to
    /// clients until the broker has been entirely initialized.
    public: void Start();

    /// \brief Get the current time.
    /// \return Current time.
    public: std::chrono::steady_clock::duration Time() const;

    /// \brief Set the current time.
    /// \param[in] _time Current time.
    public: void SetTime(const std::chrono::steady_clock::duration &_time);

    /// \brief This method associates an address with a client topic used as
    /// callback for receiving messages. This is a client requirement to
    /// start receiving messages.
    /// \param[in] _req Bind request containing the following content:
    ///   _req[0] Client address.
    ///   _req[1] Model name associated to the address.
    ///   _req[2] Client subscription topic.
    /// \param[out] _rep Unused
    /// \return True when the bind service succeeded or false otherwise.
    public: bool OnBind(const gz::msgs::StringMsg_V &_req,
                        gz::msgs::Boolean &_rep);

    /// \brief Unbind a given client address. The client associated to this
    /// address will not receive any more messages.
    /// \param[in] _req Bind request containing the following content:
    ///   _req[0] Client address.
    ///   _req[1] Client subscription topic.
    public: void OnUnbind(const gz::msgs::StringMsg_V &_req);

    /// \brief Callback executed to process a communication request from one of
    /// the clients.
    /// \param[in] _msg The message from the client.
    public: void OnMsg(const gz::msgs::Dataframe &_msg);

    /// \brief Process all the messages in the inbound queue and deliver them
    /// to the destination clients.
    public: void DeliverMsgs();

    /// \brief Get a mutable reference to the message manager.
    /// \return The mutable reference.
    public: MsgManager &DataManager();

    /// \brief Lock the mutex to access the message manager.
    public: void Lock();

    /// \brief Unlock the mutex to access the message manager.
    public: void Unlock();

   /// \brief Private data pointer.
   GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}

#endif
