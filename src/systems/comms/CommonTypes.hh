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
#ifndef IGNITION_GAZEBO_SYSTEMS_COMMONTYPES_HH_
#define IGNITION_GAZEBO_SYSTEMS_COMMONTYPES_HH_

#include <string>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

// /// \brief Address used to send a message to all the members of the team
// /// listening on a specific port.
// const std::string kBroadcast = "broadcast";

// /// \brief Address used to bind to a multicast group. Note that we do not
// /// support multiple multicast groups, only one.
// const std::string kMulticast = "multicast";

// /// \brief Topic used to centralize all messages sent from the agents.
const std::string kBrokerTopic = "/broker";

/// \brief Service used to validate an address.
const std::string kAddrBindSrv = "/broker/bind";

/// \brief Service used to invalidate an address.
const std::string kAddrUnbindSrv = "/broker/unbind";
}
}
}
}

#endif