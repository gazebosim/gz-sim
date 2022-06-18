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

#include <gtest/gtest.h>

#include <cstdlib>
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "PeerTracker.hh"
#include "gz/sim/EventManager.hh"

using namespace gz::sim;

//////////////////////////////////////////////////
TEST(PeerTracker, PeerTracker)
{
  gz::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  std::atomic<int> peers = 0;
  auto added = eventMgr.Connect<PeerAdded>([&peers](PeerInfo _peer)
  {
    (void) _peer;
    peers++;
  });

  auto removed = eventMgr.Connect<PeerRemoved>([&peers](PeerInfo _peer)
  {
    (void) _peer;
    peers--;
  });

  auto tracker1 = std::make_shared<PeerTracker>(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr);
  EXPECT_EQ(0, peers);

  auto tracker2 = std::make_shared<PeerTracker>(
    PeerInfo(NetworkRole::SimulationSecondary));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(1, peers);

  auto tracker3 = std::make_shared<PeerTracker>(
    PeerInfo(NetworkRole::SimulationSecondary));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(2, peers);

  auto tracker4 = std::make_shared<PeerTracker>(
    PeerInfo(NetworkRole::ReadOnly));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(3, peers);

  auto tracker5 = std::make_shared<PeerTracker>(
    PeerInfo(NetworkRole::ReadOnly));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(4, peers);

  auto tracker6 = std::make_shared<PeerTracker>(
    PeerInfo(NetworkRole::None));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(5, peers);

  // Allow all the heartbeats to propagate
  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep &&
      (tracker1->NumPeers() < 5 ||
      tracker2->NumPeers() < 5 ||
      tracker3->NumPeers() < 5 ||
      tracker4->NumPeers() < 5 ||
      tracker5->NumPeers() < 5); ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // All counts exclude self.
  EXPECT_EQ(5u, tracker1->NumPeers());
  EXPECT_EQ(5u, tracker2->NumPeers());
  EXPECT_EQ(5u, tracker3->NumPeers());
  EXPECT_EQ(5u, tracker4->NumPeers());
  EXPECT_EQ(5u, tracker5->NumPeers());

  EXPECT_EQ(0u, tracker1->NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker1->NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker1->NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker1->NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker2->NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(1u, tracker2->NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker2->NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker2->NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker3->NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(1u, tracker3->NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker3->NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker3->NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker4->NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker4->NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(1u, tracker4->NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker4->NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker5->NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker5->NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(1u, tracker5->NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker5->NumPeers(NetworkRole::None));

  tracker6.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(4, peers);

  tracker5.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(3, peers);

  tracker4.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(2, peers);

  tracker3.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(1, peers);

  tracker2.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(0, peers);

  tracker1.reset();
}

//////////////////////////////////////////////////
TEST(PeerTracker, GZ_UTILS_TEST_DISABLED_ON_MAC(PeerTrackerStale))
{
  gz::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  // Tracker with artificially short timeout.
  auto tracker1 = std::make_shared<PeerTracker>(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr);
  tracker1->SetHeartbeatPeriod(std::chrono::milliseconds(10));
  tracker1->SetStaleMultiplier(1);

  auto info2 = PeerInfo(NetworkRole::SimulationSecondary);

  std::atomic<int> stalePeers = 0;
  auto stale = eventMgr.Connect<PeerStale>([&](PeerInfo _peer)
  {
    EXPECT_EQ(_peer.id, info2.id);
    stalePeers++;
  });

  auto tracker2 = std::make_shared<PeerTracker>(info2);
  tracker2->SetHeartbeatPeriod(std::chrono::milliseconds(100));

  int maxSleep{100};
  int sleep{0};
  for (; sleep < maxSleep && (tracker2->NumPeers() == 0 || stalePeers == 0);
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  EXPECT_LT(sleep, maxSleep);

  EXPECT_EQ(1, stalePeers);

  // Expect while tracker1 can see tracker2, the opposite is
  // not true.
  EXPECT_EQ(0u, tracker1->NumPeers());
  EXPECT_EQ(1u, tracker2->NumPeers());

  // PeerTracker will print a debug message when DISCONNECTING message is
  // received from stale peer
}

//////////////////////////////////////////////////
TEST(PeerTracker, Partitioned)
{
  gz::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  auto options1 = gz::transport::NodeOptions();
  options1.SetPartition("p1");
  auto tracker1 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr, options1);

  auto options2 = gz::transport::NodeOptions();
  options2.SetPartition("p2");
  auto tracker2 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr, options2);

  // Allow all the heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Trackers should not detect peers in different partitions
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  auto tracker3 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr, options1);

  auto tracker4 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr, options2);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  for (int sleep = 0; sleep < 30 &&
      (tracker1.NumPeers() == 0 ||
       tracker2.NumPeers() == 0 ||
       tracker3.NumPeers() == 0 ||
       tracker4.NumPeers() == 0);
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());
}

//////////////////////////////////////////////////
TEST(PeerTracker, Namespaced)
{
  gz::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  auto options1 = gz::transport::NodeOptions();
  options1.SetNameSpace("ns1");
  auto tracker1 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr, options1);

  auto options2 = gz::transport::NodeOptions();
  options2.SetNameSpace("ns2");
  auto tracker2 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr, options2);

  // Allow some time for heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Trackers should not detect peers in different namespaces
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  auto tracker3 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr, options1);

  auto tracker4 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr, options2);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());
}

//////////////////////////////////////////////////
// Only on Linux for the moment
#ifdef  __linux__
TEST(PeerTracker, PartitionedEnv)
{
  gz::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  gz::common::setenv("GZ_PARTITION", "p1");
  auto tracker1 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr);

  gz::common::setenv("GZ_PARTITION", "p2");
  auto tracker2 = PeerTracker(
      PeerInfo(NetworkRole::SimulationPrimary), &eventMgr);

  // Allow some time for heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should not detect peers in different partitions
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  gz::common::setenv("GZ_PARTITION", "p1");
  auto tracker3 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr);

  gz::common::setenv("GZ_PARTITION", "p2");
  auto tracker4 = PeerTracker(
      PeerInfo(NetworkRole::SimulationSecondary), &eventMgr);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());

  gz::common::unsetenv("GZ_PARTITION");
}
#endif
