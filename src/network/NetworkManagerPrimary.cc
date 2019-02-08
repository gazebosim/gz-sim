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
#include "NetworkManagerPrimary.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"

#include "NetworkManagerPrivate.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(const NetworkConfig &_config):
  NetworkManager(_config)
{
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Valid() const
{
  bool valid = this->dataPtr->config.role == NetworkRole::SimulationPrimary;
  valid &= (this->dataPtr->config.numSecondariesExpected != 0);
  return valid;
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Ready() const
{
  bool ready = this->Valid();
  auto secondaries = this->dataPtr->tracker->NumSecondary();
  igndbg << "Found: " << secondaries << " network secondaries."
         << " (Expected: " << this->dataPtr->config.numSecondariesExpected
         << ")" << std::endl;
  ready &= (secondaries == this->dataPtr->config.numSecondariesExpected);
  return ready;
}

//////////////////////////////////////////////////
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}
