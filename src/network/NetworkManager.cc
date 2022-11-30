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


#include <algorithm>
#include <string>

#include "gz/common/Console.hh"
#include "gz/common/Util.hh"
#include "gz/sim/Events.hh"

#include "NetworkManager.hh"
#include "NetworkManagerPrivate.hh"
#include "NetworkManagerPrimary.hh"
#include "NetworkManagerSecondary.hh"

using namespace gz;
using namespace sim;


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
    const std::function<void(const UpdateInfo &_info)> &_stepFunction,
    EntityComponentManager &_ecm, EventManager *_eventMgr,
    const NetworkConfig &_config,
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
          _stepFunction, _ecm, _eventMgr, _config, _options);
      break;
    case NetworkRole::SimulationSecondary:
      ret = std::make_unique<NetworkManagerSecondary>(
          _stepFunction, _ecm, _eventMgr, _config, _options);
      break;
    case NetworkRole::ReadOnly:
      // \todo(mjcarroll): Enable ReadOnly
      gzwarn << "ReadOnly role not currently supported" << std::endl;
    case NetworkRole::None:
      break;
    default:
      gzwarn << "Cannot create NetworkManager, unrecognized role"
        << std::endl;
  }

  return ret;
}

//////////////////////////////////////////////////
NetworkManager::NetworkManager(
    const std::function<void(const UpdateInfo &_info)> &_stepFunction,
    EntityComponentManager &_ecm, EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  dataPtr(new NetworkManagerPrivate)
{
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->stepFunction = _stepFunction;
  this->dataPtr->config = _config;
  this->dataPtr->peerInfo = PeerInfo(this->dataPtr->config.role);
  this->dataPtr->eventMgr = _eventMgr;
  this->dataPtr->tracker = std::make_unique<PeerTracker>(
      this->dataPtr->peerInfo, _eventMgr, _options);

  if (_eventMgr)
  {
    // Set a flag when the executable is stopping to cleanly exit.
    this->dataPtr->stoppingConn = _eventMgr->Connect<events::Stop>([this]()
    {
      this->dataPtr->stopReceived = true;
    });

    this->dataPtr->peerRemovedConn = _eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info)
    {
      if (_info.Namespace() != this->Namespace())
      {
        gzmsg << "Peer [" << _info.Namespace()
               << "] removed, stopping simulation" << std::endl;
        this->dataPtr->eventMgr->Emit<events::Stop>();
      }
    });

    this->dataPtr->peerStaleConn = _eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info)
    {
      if (_info.Namespace() != this->Namespace())
      {
        gzerr << "Peer [" << _info.Namespace()
               << "] went stale, stopping simulation" << std::endl;
        this->dataPtr->eventMgr->Emit<events::Stop>();
      }
    });
  }
  else
  {
    gzwarn << "NetworkManager started without EventManager. "
      << "Distributed environment may not terminate correctly" << std::endl;
  }
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

//////////////////////////////////////////////////
NetworkConfig NetworkManager::Config() const
{
  return this->dataPtr->config;
}
