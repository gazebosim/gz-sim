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
#include "ignition/gazebo/network/NetworkManager.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"
#include "ignition/transport/NetUtils.hh"
#include "NetworkManagerPrivate.hh"


using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManager::NetworkManager():
  dataPtr(new NetworkManagerPrivate)
{
  std::string role;
  if (common::env("IGN_GAZEBO_NETWORK_ROLE", role))
  {
    std::transform(role.begin(), role.end(), role.begin(), ::toupper);
    if (role == "PRIMARY" || role == "SIMULATION_PRIMARY")
    {
      this->dataPtr->config.role = NetworkRole::SimulationPrimary;
    }
    else if (role == "SECONDARY" || role == "SIMULATION_SECONDARY")
    {
      this->dataPtr->config.role = NetworkRole::SimulationSecondary;
    }
    else if (role == "READONLY" || role == "READ_ONLY")
    {
      this->dataPtr->config.role = NetworkRole::ReadOnly;
    }
    else
    {
      this->dataPtr->config.role = NetworkRole::None;
      ignwarn << "Invalid setting for IGN_GAZEBO_NETWORK_ROLE: " << role
              << "(expected: PRIMARY, SECONDARY, READONLY)"
              << ", distributed sim disabled" << std::endl;
    }
  }
  else
  {
      ignwarn << "IGN_GAZEBO_NETWORK_ROLE not set"
              << ", distributed sim disabled" << std::endl;
  }

  // If this is configured as a primary, we need to know number of secondaries
  if (this->dataPtr->config.role == NetworkRole::SimulationPrimary)
  {
    std::string secondaries;
    if (common::env("IGN_GAZEBO_NETWORK_SECONDARIES", secondaries))
    {
      try {
        this->dataPtr->config.numSecondariesExpected = std::stoul(secondaries);
      }
      catch (const std::invalid_argument& e)
      {
        ignwarn << "IGN_GAZEBO_NETWORK_SECONDARIES set to invalid value: "
                << secondaries << ", distributed sim disabled" << std::endl;
      }
    }
    else
    {
      this->dataPtr->config.role = NetworkRole::None;
      ignwarn << "Detected IGN_GAZEBO_NETWORK_ROLE=PRIMARY, but "
        << "IGN_GAZEBO_NETWORK_SECONDARIES not set, "
        << "no distributed sim available" << std::endl;
    }
  }

  this->dataPtr->peerInfo = PeerInfo(this->dataPtr->config.role);
  this->dataPtr->tracker = std::make_unique<PeerTracker>(
      this->dataPtr->peerInfo);
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

//////////////////////////////////////////////////
bool NetworkManager::Valid() const
{
  bool valid = this->dataPtr->config.role != NetworkRole::None;

  if (this->IsPrimary())
  {
    valid &= (this->dataPtr->config.numSecondariesExpected != 0);
  }
  return valid;
}

//////////////////////////////////////////////////
bool NetworkManager::Ready() const
{
  bool ready = Valid();

  if (this->IsPrimary())
  {
    auto secondaries = this->dataPtr->tracker->NumSecondary();
    igndbg << "Found: " << secondaries << " network secondaries."
           << " (Expected: " << this->dataPtr->config.numSecondariesExpected
           << ")" << std::endl;
    ready &= (secondaries == this->dataPtr->config.numSecondariesExpected);
  }
  else if (this->IsSecondary())
  {
    auto primaries = this->dataPtr->tracker->NumPrimary();
    igndbg << "Found: " << primaries << " network primaries."
           << " (Expected: 1)"  << std::endl;
    ready &= (primaries == 1);
  }

  return ready;
}

//////////////////////////////////////////////////
std::string NetworkManager::Namespace() const
{
  return this->dataPtr->peerInfo.Namespace();
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
