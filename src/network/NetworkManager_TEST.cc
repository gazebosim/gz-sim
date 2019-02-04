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

#include <cstdlib>
#include <ignition/common/Console.hh>

#include "ignition/gazebo/network/NetworkManager.hh"

using namespace ignition::gazebo;

//////////////////////////////////////////////////
TEST(NetworkManager, ConfigConstructor)
{
  ignition::common::Console::SetVerbosity(4);

  {
    // Primary without number of secondaries is invalid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    NetworkManager nm(conf);
    ASSERT_FALSE(nm.Valid());
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    conf.numSecondariesExpected = 5;
    NetworkManager nm(conf);
    ASSERT_TRUE(nm.Valid());
  }

  {
    // Secondary is always valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationSecondary;
    NetworkManager nm(conf);
    ASSERT_TRUE(nm.Valid());
  }

  {
    // Readonly is always valid
    NetworkConfig conf;
    conf.role = NetworkRole::ReadOnly;
    NetworkManager nm(conf);
    ASSERT_TRUE(nm.Valid());
  }
}

//////////////////////////////////////////////////
// Only on Linux for the moment
#ifdef  __linux__
TEST(NetworkManager, EnvConstructor)
{
  ignition::common::Console::SetVerbosity(4);

  {
    // Primary without number of secondaries is invalid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
    NetworkManager nm;
    ASSERT_FALSE(nm.Valid());
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
    setenv("IGN_GAZEBO_NETWORK_SECONDARIES", "3", 1);
    NetworkManager nm;
    ASSERT_TRUE(nm.Valid());
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
    unsetenv("IGN_GAZEBO_NETWORK_SECONDARIES");
  }

  {
    // Secondary is always valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "SECONDARY", 1);
    NetworkManager nm;
    ASSERT_TRUE(nm.Valid());
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }

  {
    // Readonly is always valid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "READONLY", 1);
    NetworkManager nm;
    ASSERT_TRUE(nm.Valid());
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }

  {
    // Anything else is invalid
    setenv("IGN_GAZEBO_NETWORK_ROLE", "READ_WRITE", 1);
    NetworkManager nm;
    ASSERT_FALSE(nm.Valid());
    unsetenv("IGN_GAZEBO_NETWORK_ROLE");
  }
}
#endif

//////////////////////////////////////////////////
TEST(NetworkManager, EstablishComms)
{
  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  NetworkManager nmPrimary(confPrimary);
  ASSERT_TRUE(nmPrimary.Valid());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  NetworkManager nmSecondary1(confSecondary1);
  ASSERT_TRUE(nmSecondary1.Valid());

  NetworkConfig confSecondary2;
  confSecondary2.role = NetworkRole::SimulationSecondary;
  NetworkManager nmSecondary2(confSecondary2);
  ASSERT_TRUE(nmSecondary2.Valid());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // All participants should be "ready" in that the correct
  // number of peers are discovered for their respective role.
  ASSERT_TRUE(nmPrimary.Ready());
  ASSERT_TRUE(nmSecondary1.Ready());
  ASSERT_TRUE(nmSecondary2.Ready());
}


