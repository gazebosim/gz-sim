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
    auto nm = NetworkManager::Create(nullptr, conf);
    ASSERT_EQ(nullptr, nm);
    // Expect console warning as well
  }

  {
    // Primary with number of secondaries is valid
    NetworkConfig conf;
    conf.role = NetworkRole::SimulationPrimary;
    conf.numSecondariesExpected = 5;
    auto nm = NetworkManager::Create(nullptr, conf);
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
    auto nm = NetworkManager::Create(nullptr, conf);
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
    auto nm = NetworkManager::Create(nullptr, conf);
    ASSERT_EQ(nullptr, nm);
  }

  {
    // None is always invalid
    NetworkConfig conf;
    conf.role = NetworkRole::None;
    auto nm = NetworkManager::Create(nullptr, conf);
    ASSERT_EQ(nullptr, nm);
  }
}

//////////////////////////////////////////////////
TEST(NetworkManager, EstablishComms)
{
  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  auto nmPrimary = NetworkManager::Create(nullptr, confPrimary);
  ASSERT_NE(nullptr, nmPrimary);
  EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nmPrimary.get()));
  EXPECT_TRUE(nmPrimary->IsPrimary());
  // Primary namespace is an empty string.
  EXPECT_EQ(0u, nmPrimary->Namespace().length());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary1 = NetworkManager::Create(nullptr, confSecondary1);
  ASSERT_NE(nullptr, nmSecondary1);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary1.get()));
  EXPECT_TRUE(nmSecondary1->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary1->Namespace().length());

  NetworkConfig confSecondary2;
  confSecondary2.role = NetworkRole::SimulationSecondary;
  auto nmSecondary2 = NetworkManager::Create(nullptr, confSecondary2);
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
  // Create a primary and two secondaries
  NetworkConfig confPrimary;
  confPrimary.role = NetworkRole::SimulationPrimary;
  confPrimary.numSecondariesExpected = 2;

  auto nmPrimary = NetworkManager::Create(nullptr, confPrimary);
  ASSERT_NE(nullptr, nmPrimary);
  EXPECT_NE(nullptr, static_cast<NetworkManagerPrimary *>(nmPrimary.get()));
  EXPECT_TRUE(nmPrimary->IsPrimary());
  // Primary namespace is an empty string.
  EXPECT_EQ(0u, nmPrimary->Namespace().length());

  NetworkConfig confSecondary1;
  confSecondary1.role = NetworkRole::SimulationSecondary;
  auto nmSecondary1 = NetworkManager::Create(nullptr, confSecondary1);
  ASSERT_NE(nullptr, nmSecondary1);
  EXPECT_NE(nullptr,
      static_cast<NetworkManagerSecondary *>(nmSecondary1.get()));
  EXPECT_TRUE(nmSecondary1->IsSecondary());
  // Secondary namespace is the first 8 digits of the secondary's UUID
  EXPECT_LT(0u, nmSecondary1->Namespace().length());

  NetworkConfig confSecondary2;
  confSecondary2.role = NetworkRole::SimulationSecondary;
  auto nmSecondary2 = NetworkManager::Create(nullptr, confSecondary2);
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

  auto primaryThread = std::thread([&nmPrimary, &running](){
    uint64_t primaryIter = 0;
    auto primaryStepSize = std::chrono::steady_clock::duration{2ms};
    auto primarySimTime = std::chrono::steady_clock::duration{0};

    nmPrimary->Initialize();
    nmPrimary->Step(primaryIter, primaryStepSize, primarySimTime);

    while (primaryIter <= 100)
    {
      while (!nmPrimary->Step(primaryIter, primaryStepSize, primarySimTime)) {}

      while (!nmPrimary->StepAck(primaryIter)) {}

      primaryIter++;
      primarySimTime += primaryStepSize;
    }

    running = false;
  });

  uint64_t secondaryIter1 = 0;
  uint64_t secondaryIter2 = 0;

  auto secondaryThread1 = std::thread([&nmSecondary1, &secondaryIter1](){
    auto secondaryStepSize = std::chrono::steady_clock::duration{0ms};
    auto secondarySimTime = std::chrono::steady_clock::duration{0};

    nmSecondary1->Initialize();

    while (secondaryIter1 < 100)
    {
      while (!nmSecondary1->Step(
            secondaryIter1, secondaryStepSize, secondarySimTime)) {}
      while (!nmSecondary1->StepAck(secondaryIter1)) {}
    }
  });

  auto secondaryThread2 = std::thread([&nmSecondary2, &secondaryIter2](){
    auto secondaryStepSize = std::chrono::steady_clock::duration{0ms};
    auto secondarySimTime = std::chrono::steady_clock::duration{0};

    nmSecondary2->Initialize();

    while (secondaryIter2 < 100)
    {
      while (!nmSecondary2->Step(
            secondaryIter2, secondaryStepSize, secondarySimTime)) {}
      while (!nmSecondary2->StepAck(secondaryIter2)) {}
    }
  });

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  primaryThread.join();
  secondaryThread1.join();
  secondaryThread2.join();

  EXPECT_FALSE(running);
  EXPECT_EQ(100u, secondaryIter1);
  EXPECT_EQ(100u, secondaryIter2);
}
