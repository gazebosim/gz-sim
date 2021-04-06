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

#include "ignition/common/Uuid.hh"
#include "ignition/transport/NetUtils.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
PeerInfo::PeerInfo(const NetworkRole &_role):
  id(ignition::common::Uuid().String()),
  hostname(ignition::transport::hostname()),
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
ignition::gazebo::private_msgs::PeerInfo ignition::gazebo::toProto(
    const PeerInfo &_info)
{
  ignition::gazebo::private_msgs::PeerInfo proto;
  proto.set_id(_info.id);
  proto.set_hostname(_info.hostname);

  switch (_info.role)
  {
    case NetworkRole::ReadOnly:
      proto.set_role(ignition::gazebo::private_msgs::PeerInfo::READ_ONLY);
      break;
    case NetworkRole::SimulationPrimary:
      proto.set_role(
          ignition::gazebo::private_msgs::PeerInfo::SIMULATION_PRIMARY);
      break;
    case NetworkRole::SimulationSecondary:
      proto.set_role(
          ignition::gazebo::private_msgs::PeerInfo::SIMULATION_SECONDARY);
      break;
    case NetworkRole::None:
    default:
      proto.set_role(ignition::gazebo::private_msgs::PeerInfo::NONE);
  }
  return proto;
}

/////////////////////////////////////////////////
PeerInfo ignition::gazebo::fromProto(
    const ignition::gazebo::private_msgs::PeerInfo& _proto)
{
  PeerInfo info;
  info.id = _proto.id();
  info.hostname = _proto.hostname();

  switch (_proto.role())
  {
    case ignition::gazebo::private_msgs::PeerInfo::READ_ONLY:
      info.role = NetworkRole::ReadOnly;
      break;
    case ignition::gazebo::private_msgs::PeerInfo::SIMULATION_PRIMARY:
      info.role = NetworkRole::SimulationPrimary;
      break;
    case ignition::gazebo::private_msgs::PeerInfo::SIMULATION_SECONDARY:
      info.role = NetworkRole::SimulationSecondary;
      break;
    case ignition::gazebo::private_msgs::PeerInfo::NONE:
    default:
      info.role = NetworkRole::None;
      break;
  }
  return info;
}
