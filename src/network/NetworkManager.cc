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

using namespace ignition;
using namespace gazebo;


bool validateConfig(const NetworkConfig &_config)
{
  bool valid = true;
  switch (_config.role)
  {
    case NetworkRole::SimulationPrimary:
      if (_config.numSecondariesExpected <= 0)
      {
        valid = false;
      }
      break;
    case NetworkRole::SimulationSecondary:
      break;
    case NetworkRole::ReadOnly:
    case NetworkRole::None:
    default:
      valid = false;
  }

  return valid;
}

//////////////////////////////////////////////////
std::unique_ptr<NetworkManager> NetworkManager::Create(
    EventManager *_eventMgr, const NetworkConfig &_config,
    const NodeOptions &_options)
{
  std::unique_ptr<NetworkManager> ret;

  if (!validateConfig(_config))
  {
    return nullptr;
  }

  switch (_config.role)
  {
    case NetworkRole::SimulationPrimary:
      ret = std::make_unique<NetworkManagerPrimary>(
          _eventMgr, _config, _options);
      break;
    case NetworkRole::SimulationSecondary:
      ret = std::make_unique<NetworkManagerSecondary>(
          _eventMgr, _config, _options);
      break;
    case NetworkRole::ReadOnly:
      // \todo(mjcarroll): Enable ReadOnly
      ignwarn << "ReadOnly role not currently supported" << std::endl;
    case NetworkRole::None:
      break;
    default:
      ignwarn << "Cannot create NetworkManager, unrecognized role"
        << std::endl;
  }

  return ret;
}

//////////////////////////////////////////////////
NetworkManager::NetworkManager(
    EventManager *_eventMgr, const NetworkConfig &_config,
    const NodeOptions &_options):
  dataPtr(new NetworkManagerPrivate)
{
  this->dataPtr->config = _config;
  this->dataPtr->peerInfo = PeerInfo(this->dataPtr->config.role);
  this->dataPtr->eventMgr = _eventMgr;
  this->dataPtr->tracker = std::make_unique<PeerTracker>(
      this->dataPtr->peerInfo, _eventMgr, _options);
}

//////////////////////////////////////////////////
NetworkManager::~NetworkManager() = default;


//////////////////////////////////////////////////
NetworkRole NetworkManager::Role() const
{
  return this->dataPtr->config.role;
}

//////////////////////////////////////////////////
bool NetworkManager::IsPrimary() const
{
  return this->dataPtr->config.role == NetworkRole::SimulationPrimary;
}

//////////////////////////////////////////////////
bool NetworkManager::IsSecondary() const
{
  return this->dataPtr->config.role == NetworkRole::SimulationSecondary;
}

//////////////////////////////////////////////////
bool NetworkManager::IsReadOnly() const
{
  return this->dataPtr->config.role == NetworkRole::ReadOnly;
}
