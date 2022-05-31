/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "PeerInfo.hh"

#include "gz/common/Uuid.hh"
#include "gz/transport/NetUtils.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
PeerInfo::PeerInfo(const NetworkRole &_role):
  id(gz::common::Uuid().String()),
  hostname(gz::transport::hostname()),
  role(_role)
{
}

/////////////////////////////////////////////////
std::string PeerInfo::Namespace() const
{
  std::string ns = "";
  if (this->role == NetworkRole::SimulationSecondary)
  {
    ns = "/" + this->id.substr(0, 8);
  }
  return ns;
}

/////////////////////////////////////////////////
gz::sim::private_msgs::PeerInfo gz::sim::toProto(
    const PeerInfo &_info)
{
  gz::sim::private_msgs::PeerInfo proto;
  proto.set_id(_info.id);
  proto.set_hostname(_info.hostname);

  switch (_info.role)
  {
    case NetworkRole::ReadOnly:
      proto.set_role(gz::sim::private_msgs::PeerInfo::READ_ONLY);
      break;
    case NetworkRole::SimulationPrimary:
      proto.set_role(
          gz::sim::private_msgs::PeerInfo::SIMULATION_PRIMARY);
      break;
    case NetworkRole::SimulationSecondary:
      proto.set_role(
          gz::sim::private_msgs::PeerInfo::SIMULATION_SECONDARY);
      break;
    case NetworkRole::None:
    default:
      proto.set_role(gz::sim::private_msgs::PeerInfo::NONE);
  }
  return proto;
}

/////////////////////////////////////////////////
PeerInfo gz::sim::fromProto(
    const gz::sim::private_msgs::PeerInfo& _proto)
{
  PeerInfo info;
  info.id = _proto.id();
  info.hostname = _proto.hostname();

  switch (_proto.role())
  {
    case gz::sim::private_msgs::PeerInfo::READ_ONLY:
      info.role = NetworkRole::ReadOnly;
      break;
    case gz::sim::private_msgs::PeerInfo::SIMULATION_PRIMARY:
      info.role = NetworkRole::SimulationPrimary;
      break;
    case gz::sim::private_msgs::PeerInfo::SIMULATION_SECONDARY:
      info.role = NetworkRole::SimulationSecondary;
      break;
    case gz::sim::private_msgs::PeerInfo::NONE:
    default:
      info.role = NetworkRole::None;
      break;
  }
  return info;
}
