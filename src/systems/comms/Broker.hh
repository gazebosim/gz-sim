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

#include <mutex>
#include <ignition/msgs/datagram.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/utils/ImplPtr.hh>

#include "AddressManager.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Store messages, and exposes an API for registering new clients,
  /// bind to a particular address, push new messages or get the list of
  /// messages already stored in the queue.
  class Broker
  {
    /// \brief Constructor.
    public: Broker();

    /// \brief Destructor.
    public: virtual ~Broker();

    /// \brief Start.
    public: void Start();

    /// \brief ToDo.
    public: void OnBind(const ignition::msgs::StringMsg_V &_req);

    /// \brief ToDo.
    public: void OnUnbind(const ignition::msgs::StringMsg_V &_req);

    /// \brief ToDo.
    public: void OnMsg(const ignition::msgs::Datagram &_msg);

    /// \brief ToDo.
    public: void DeliverMsgs();

    /// \brief ToDo.
    public: AddressManager &Data();

    /// \brief ToDo.
    public: void Lock();

    /// \brief ToDo.
    public: void Unlock();

   /// \brief Private data pointer.
   IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
}
}

#endif
