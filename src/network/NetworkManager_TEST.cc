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
#include <gz/common/Console.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "NetworkManager.hh"
#include "NetworkManagerPrimary.hh"
#include "NetworkManagerSecondary.hh"

using namespace gz::sim;

void step(const UpdateInfo &)
{
}

//////////////////////////////////////////////////
TEST(NetworkManager, ConfigConstructor)
{
  gz::common::Console::SetVerbosity(4);

  EntityComponentManager ecm;

  {
    // Primary without number of secondaries is invalid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    auto nm = NetworkManager::Create(step, ecm, nullptr, conf);
    ASSERT_EQ(nullptr, nm);
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    conf.numSecondariesExpected = 5;
    auto nm = NetworkManager::Create(step, ecm, nullptr, conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nm.get()));
    EXPECT_TRUE(nm->IsPrimary());
    EXPECT_FALSE(nm->IsSecondary());
    EXPECT_FALSE(nm->IsReadOnly());
  }

  {
    // Secondary is always valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationSecondary;
    auto nm = NetworkManager::Create(step, ecm, nullptr, conf);
    ASSERT_NE(nullptr, nm);
    EXPECT_NE(nullptr, static_cast<NetworkManagerSecondary *>(nm.get()));
    EXPECT_FALSE(nm->IsPrimary());
    EXPECT_TRUE(nm->IsSecondary());
    EXPECT_FALSE(nm->IsReadOnly());
  }

  {
    // Readonly is always invalid
    NetworkConfig conf;
    conf.role = NetworkRole::ReadOnly;
    auto nm = NetworkManager::Create(step, ecm, nullptr, conf);
    ASSERT_EQ(nullptr, nm);
  }

  {
    // None is always invalid
    NetworkConfig conf;
    conf.role = NetworkRole::None;
    auto nm = NetworkManager::Create(step, ecm, nullptr, conf);
    ASSERT_EQ(nullptr, nm);
  }
}

//////////////////////////////////////////////////
TEST(NetworkManager, EstablishComms)
{
  gz::common::Console::SetVerbosity(4);

  EntityComponentManager ecm;

  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  auto nmPrimary = NetworkManager::Create(step, ecm, nullptr, confPrimary);
  ASSERT_NE(nullptr, nmPrimary);
  EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nmPrimary.get()));
  EXPECT_TRUE(nmPrimary->IsPrimary());
  // Primary namespace is an empty string.
  EXPECT_EQ(0u, nmPrimary->Namespace().length());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary1 = NetworkManager::Create(step, ecm, nullptr,
      confSecondary1);
  ASSERT_NE(nullptr, nmSecondary1);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary1.get()));
  EXPECT_TRUE(nmSecondary1->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary1->Namespace().length());

  NetworkConfig confSecondary2;
  confSecondary2.role = NetworkRole::SimulationSecondary;
  auto nmSecondary2 = NetworkManager::Create(step, ecm, nullptr,
      confSecondary2);
  ASSERT_NE(nullptr, nmSecondary2);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary2.get()));
  EXPECT_TRUE(nmSecondary2->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary2->Namespace().length());
  // Secondary namespace should be unique.
  EXPECT_TRUE(nmSecondary1->Namespace() != nmSecondary2->Namespace());

  // Give time for messages to propagate
  for (int sleep = 0; sleep < 50 &&
      (!nmPrimary->Ready() || !nmSecondary1->Ready() || !nmSecondary2->Ready());
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // All participants should be "ready" in that the correct
  // number of peers are discovered for their respective role.
  EXPECT_TRUE(nmPrimary->Ready());
  EXPECT_TRUE(nmSecondary1->Ready());
  EXPECT_TRUE(nmSecondary2->Ready());
}

//////////////////////////////////////////////////
TEST(NetworkManager, Step)
{
  gz::common::Console::SetVerbosity(4);

  EntityComponentManager ecm;

  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  auto nmPrimary = NetworkManager::Create(step, ecm, nullptr, confPrimary);
  ASSERT_NE(nullptr, nmPrimary);
  EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nmPrimary.get()));
  EXPECT_TRUE(nmPrimary->IsPrimary());
  // Primary namespace is an empty string.
  EXPECT_EQ(0u, nmPrimary->Namespace().length());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary1 = NetworkManager::Create(step, ecm, nullptr,
      confSecondary1);
  ASSERT_NE(nullptr, nmSecondary1);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary1.get()));
  EXPECT_TRUE(nmSecondary1->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary1->Namespace().length());

  NetworkConfig confSecondary2;
  confSecondary2.role = NetworkRole::SimulationSecondary;
  auto nmSecondary2 = NetworkManager::Create(step, ecm, nullptr,
      confSecondary2);
  ASSERT_NE(nullptr, nmSecondary2);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary2.get()));
  EXPECT_TRUE(nmSecondary2->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary2->Namespace().length());
  // Secondary namespace should be unique.
  EXPECT_TRUE(nmSecondary1->Namespace() != nmSecondary2->Namespace());

  for (int sleep = 0; sleep < 50 &&
      (!nmPrimary->Ready() || !nmSecondary1->Ready() || !nmSecondary2->Ready());
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // All participants should be "ready" in that the correct
  // number of peers are discovered for their respective role.
  EXPECT_TRUE(nmPrimary->Ready());
  EXPECT_TRUE(nmSecondary1->Ready());
  EXPECT_TRUE(nmSecondary2->Ready());

  using namespace std::chrono_literals;

  std::atomic<bool> running {true};

  auto primaryThread = std::thread([&nmPrimary, &running]()
  {
    auto info = UpdateInfo();
    info.iterations = 0;
    info.dt = std::chrono::steady_clock::duration{2ms};
    info.simTime = std::chrono::steady_clock::duration{0};
    info.paused = false;

    nmPrimary->Handshake();

    auto primary = static_cast<NetworkManagerPrimary *>(nmPrimary.get());
    while (info.iterations <= 100)
    {
      // If step doesn't block, network is working
      EXPECT_TRUE(primary->Step(info));

      info.iterations++;
      info.simTime += info.dt;
    }

    running = false;
  });

  auto secondaryThread1 = std::thread([&nmSecondary1, &running]()
  {
    nmSecondary1->Handshake();

    while (running)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  auto secondaryThread2 = std::thread([&nmSecondary2, &running]()
  {
    nmSecondary2->Handshake();

    while (running)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  while (running)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  primaryThread.join();
  secondaryThread1.join();
  secondaryThread2.join();

  EXPECT_FALSE(running);
}
