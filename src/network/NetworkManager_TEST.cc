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

#include "NetworkManagerPrimary.hh"
#include "NetworkManagerReadOnly.hh"
#include "NetworkManagerSecondary.hh"

using namespace ignition::gazebo;

//////////////////////////////////////////////////
TEST(NetworkManager, ConfigConstructor)
{
  ignition::common::Console::SetVerbosity(4);

  {
    // Primary without number of secondaries is invalid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    auto nm = NetworkManager::Create(conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nm.get()));
    EXPECT_FALSE(nm->Valid());
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    conf.numSecondariesExpected = 5;
    auto nm = NetworkManager::Create(conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nm.get()));
    EXPECT_TRUE(nm->Valid());
  }

  {
    // Secondary is always valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationSecondary;
    auto nm = NetworkManager::Create(conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerSecondary *>(nm.get()));
    EXPECT_TRUE(nm->Valid());
  }

  {
    // Readonly is always valid
    NetworkConfig conf;
    conf.role = NetworkRole::ReadOnly;
    auto nm = NetworkManager::Create(conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerReadOnly *>(nm.get()));
    EXPECT_TRUE(nm->Valid());
  }
}

//////////////////////////////////////////////////
TEST(NetworkManager, EstablishComms)
{
  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  auto nmPrimary = NetworkManager::Create(confPrimary);
  ASSERT_NE(nullptr, nmPrimary);
  EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nmPrimary.get()));
  EXPECT_TRUE(nmPrimary->IsPrimary());
  EXPECT_TRUE(nmPrimary->Valid());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary1 = NetworkManager::Create(confSecondary1);
  ASSERT_NE(nullptr, nmSecondary1);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary1.get()));
  EXPECT_TRUE(nmSecondary1->IsSecondary());
  EXPECT_TRUE(nmSecondary1->Valid());

  NetworkConfig confSecondary2;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary2 = NetworkManager::Create(confSecondary1);
  ASSERT_NE(nullptr, nmSecondary2);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary2.get()));
  EXPECT_TRUE(nmSecondary2->IsSecondary());
  EXPECT_TRUE(nmSecondary2->Valid());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // All participants should be "ready" in that the correct
  // number of peers are discovered for their respective role.
  EXPECT_TRUE(nmPrimary->Ready());
  EXPECT_TRUE(nmSecondary1->Ready());
  EXPECT_TRUE(nmSecondary2->Ready());
}


