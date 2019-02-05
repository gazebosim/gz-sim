/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "ignition/gazebo/network/NetworkManager.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"

#include "NetworkManagerPrivate.hh"
#include "NetworkManagerPrimary.hh"
#include "NetworkManagerSecondary.hh"
#include "NetworkManagerReadOnly.hh"


using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
std::unique_ptr<NetworkManager> NetworkManager::Create(
    const NetworkConfig &_config)
{
  std::unique_ptr<NetworkManager> ret;
  switch (_config.role)
  {
    case NetworkRole::SimulationPrimary:
      ret = std::make_unique<NetworkManagerPrimary>(_config);
      break;
    case NetworkRole::SimulationSecondary:
      ret = std::make_unique<NetworkManagerSecondary>(_config);
      break;
    case NetworkRole::ReadOnly:
      ret = std::make_unique<NetworkManagerReadOnly>(_config);
      break;
    case NetworkRole::None:
    default:
      ignwarn << "Cannot create NetworkManager, unrecognized role"
        << std::endl;
  }

  return ret;
}

//////////////////////////////////////////////////
NetworkManager::NetworkManager(const NetworkConfig &_config):
  dataPtr(new NetworkManagerPrivate)
{
  this->dataPtr->config = _config;
  this->dataPtr->peerInfo = PeerInfo(this->dataPtr->config.role);
  this->dataPtr->tracker = std::make_unique<PeerTracker>(
      this->dataPtr->peerInfo);
}

//////////////////////////////////////////////////
NetworkManager::~NetworkManager() = default;
