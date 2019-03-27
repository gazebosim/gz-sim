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

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "NetworkConfig.hh"

using namespace ignition::gazebo;

TEST(NetworkManager, ValueConstructor)
{
  ignition::common::Console::SetVerbosity(4);
  {
    // Primary without number of secondaries is invalid
    auto config = NetworkConfig::FromValues("PRIMARY", 0);
    assert(config.role == NetworkRole::None);
    assert(config.numSecondariesExpected == 0);
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    auto config = NetworkConfig::FromValues("PRIMARY", 3);
    assert(config.role == NetworkRole::SimulationPrimary);
    assert(config.numSecondariesExpected == 3);
  }

  {
    // Secondary is always valid
    auto config = NetworkConfig::FromValues("SECONDARY", 0);
    assert(config.role == NetworkRole::SimulationSecondary);
  }

  {
    // Readonly is always valid
    auto config = NetworkConfig::FromValues("READONLY");
    assert(config.role == NetworkRole::ReadOnly);
  }

  {
    // Anything else is invalid
    auto config = NetworkConfig::FromValues("READ_WRITE");
    assert(config.role == NetworkRole::None);
  }
}

//////////////////////////////////////////////////
// Quick test to establish correct behavior for reading
// environment variables.
// Only on Linux for the moment.
#ifdef  __linux__
TEST(NetworkManager, EnvConstructor)
{
  ignition::common::Console::SetVerbosity(4);

  {
    // Primary without number of secondaries is invalid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::None);
    assert(config.numSecondariesExpected == 0);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
    setenv("IGN_GAZEBO_NETWORK_SECONDARIES", "3", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::SimulationPrimary);
    assert(config.numSecondariesExpected == 3);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
    unsetenv("IGN_GAZEBO_NETWORK_SECONDARIES");
  }

  {
    // Primary with non-number of secondaries is invalid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
    setenv("IGN_GAZEBO_NETWORK_SECONDARIES", "foo", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::None);
    assert(config.numSecondariesExpected == 0);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
    unsetenv("IGN_GAZEBO_NETWORK_SECONDARIES");
  }

  {
    // Secondary is always valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "SECONDARY", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::SimulationSecondary);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }

  {
    // Readonly is always valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "READONLY", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::ReadOnly);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }

  {
    // Anything else is invalid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "READ_WRITE", 1);
    auto config = NetworkConfig::FromEnv();
    assert(config.role == NetworkRole::None);
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }
}
#endif
