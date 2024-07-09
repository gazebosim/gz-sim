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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>

#include "NetworkConfig.hh"

using namespace gz::sim;

TEST(NetworkManager, ValueConstructor)
{
  gz::common::Console::SetVerbosity(4);
  {
    // Primary without number of secondaries is invalid
    auto config = NetworkConfig::FromValues("PRIMARY", 0);
    ASSERT_EQ(config.role, NetworkRole::None);
    ASSERT_EQ(config.numSecondariesExpected, 0);
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    auto config = NetworkConfig::FromValues("PRIMARY", 3);
    ASSERT_EQ(config.role, NetworkRole::SimulationPrimary);
    ASSERT_EQ(config.numSecondariesExpected, 3);
  }

  {
    // Secondary is always valid
    auto config = NetworkConfig::FromValues("SECONDARY", 0);
    ASSERT_EQ(config.role, NetworkRole::SimulationSecondary);
  }

  {
    // Readonly is always valid
    auto config = NetworkConfig::FromValues("READONLY");
    ASSERT_EQ(config.role, NetworkRole::ReadOnly);
  }

  {
    // Anything else is invalid
    auto config = NetworkConfig::FromValues("READ_WRITE");
    ASSERT_EQ(config.role, NetworkRole::None);
  }
}
