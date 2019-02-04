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
#include "NetworkManagerSecondary.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"

#include "NetworkManagerPrivate.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerSecondary::NetworkManagerSecondary(const NetworkConfig &_config):
  NetworkManager(_config)
{
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Valid() const
{
  return this->dataPtr->config.role == NetworkRole::SimulationSecondary;
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Ready() const
{
  bool ready = Valid();
  auto primaries = this->dataPtr->tracker->NumPrimary();
  igndbg << "Found: " << primaries << " network primaries."
         << " (Expected: 1)"  << std::endl;
  ready &= (primaries == 1);
  return ready;
}

//////////////////////////////////////////////////
std::string NetworkManagerSecondary::Namespace() const
{
  return this->dataPtr->peerInfo.id.substr(0, 8);
}
