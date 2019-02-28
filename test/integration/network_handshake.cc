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
#include <chrono>
#include <condition_variable>

#include "ignition/msgs/world_control.pb.h"
#include "ignition/msgs/world_stats.pb.h"
#include "ignition/transport/Node.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

uint64_t kIterations;

/////////////////////////////////////////////////
// Send a world control message.
void worldControl(bool _paused, uint64_t _steps)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [&](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    EXPECT_TRUE(_result);
  };

  ignition::msgs::WorldControl req;
  req.set_pause(_paused);
  req.set_multi_step(_steps);
  transport::Node node;
  node.Request("/world/default/control", req, cb);
}

/////////////////////////////////////////////////
// Get the current paused state from the world stats message
void testPaused(bool _paused)
{
  std::condition_variable condition;
  std::mutex mutex;
  transport::Node node;
  bool paused = !_paused;

  std::function<void(const ignition::msgs::WorldStatistics &)> cb =
      [&](const ignition::msgs::WorldStatistics &_msg)
  {
    std::unique_lock<std::mutex> lock(mutex);
    paused = _msg.paused();
    condition.notify_all();
  };

  std::unique_lock<std::mutex> lock(mutex);
  node.Subscribe("/world/default/stats", cb);
  condition.wait(lock);
  EXPECT_EQ(_paused, paused);
}

/////////////////////////////////////////////////
// Get the current iteration count from the world stats message
uint64_t iterations()
{
  std::condition_variable condition;
  std::mutex mutex;
  transport::Node node;
  uint64_t iterations = 0;

  std::function<void(const ignition::msgs::WorldStatistics &)> cb =
      [&](const ignition::msgs::WorldStatistics &_msg)
  {
    std::unique_lock<std::mutex> lock(mutex);
    iterations = _msg.iterations();
    condition.notify_all();
  };

  std::unique_lock<std::mutex> lock(mutex);
  node.Subscribe("/world/default/stats", cb);
  condition.wait(lock);
  return iterations;
}

/////////////////////////////////////////////////
// Only on Linux for the moment
#ifdef  __linux__
TEST(NetworkHandshake, Handshake)
{
  setenv("IGN_GAZEBO_NETWORK_ROLE", "PRIMARY", 1);
  setenv("IGN_GAZEBO_NETWORK_SECONDARIES", "2", 1);
  ServerConfig serverConfig;
  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  Server serverPrimary(serverConfig);
  serverPrimary.SetUpdatePeriod(1us);

  setenv("IGN_GAZEBO_NETWORK_ROLE", "SECONDARY", 1);
  Server serverSecondary1(serverConfig);
  serverSecondary1.SetUpdatePeriod(1us);

  Server serverSecondary2(serverConfig);
  serverSecondary2.SetUpdatePeriod(1us);

  // Run the server asynchronously
  serverPrimary.Run(false);
  serverSecondary1.Run(false);
  serverSecondary2.Run(false);

  // The server should start paused.
  testPaused(true);
  EXPECT_EQ(0u, iterations());
}
#endif  // __linux__
