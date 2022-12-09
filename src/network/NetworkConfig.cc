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
#include "NetworkConfig.hh"

#include <algorithm>

#include "gz/common/Console.hh"
#include "gz/common/Util.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
NetworkConfig NetworkConfig::FromValues(const std::string &_role,
    unsigned int _secondaries)
{
  NetworkConfig config;

  if (!_role.empty())
  {
    std::string role = _role;
    std::transform(role.begin(), role.end(), role.begin(), ::toupper);

    if (role == "PRIMARY" || role == "SIMULATION_PRIMARY")
    {
      config.role = NetworkRole::SimulationPrimary;
    }
    else if (role == "SECONDARY" || role == "SIMULATION_SECONDARY")
    {
      config.role = NetworkRole::SimulationSecondary;
    }
    else if (role == "READONLY" || role == "READ_ONLY")
    {
      config.role = NetworkRole::ReadOnly;
    }
    else
    {
      config.role = NetworkRole::None;
      gzwarn << "Invalid setting for network role: " << role
              << "(expected: PRIMARY, SECONDARY, READONLY)"
              << ", distributed sim disabled" << std::endl;
    }
  }
  else
  {
      gzwarn << "Network role not set"
              << ", distributed sim disabled" << std::endl;
  }

  // If this is configured as a primary, we need to know number of secondaries
  if (config.role == NetworkRole::SimulationPrimary)
  {
    config.numSecondariesExpected = _secondaries;
    if (config.numSecondariesExpected == 0)
    {
      config.role = NetworkRole::None;
      gzwarn << "Detected network role as PRIMARY, but "
        << "network secondaries not set, "
        << "no distributed sim available" << std::endl;
    }
  }

  return config;
}
