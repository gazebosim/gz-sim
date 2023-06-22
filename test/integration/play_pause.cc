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
#include <chrono>
#include <condition_variable>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <gz/transport/Node.hh>
#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

uint64_t kIterations;

/////////////////////////////////////////////////
// Send a world control message.
void worldControl(bool _paused, uint64_t _steps)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [&](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    EXPECT_TRUE(_result);
  };

  msgs::WorldControl req;
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

  std::function<void(const msgs::WorldStatistics &)> cb =
      [&](const msgs::WorldStatistics &_msg)
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

  std::function<void(const msgs::WorldStatistics &)> cb =
      [&](const msgs::WorldStatistics &_msg)
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

class PlayPause : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(PlayPause, PlayPause)
{
  common::Console::SetVerbosity(4);

  ServerConfig serverConfig;
  Server server(serverConfig);
  server.SetUpdatePeriod(1us);

  // Run the server asynchronously
  server.Run(false);

  // The server should start paused.
  testPaused(true);
  EXPECT_EQ(0u, iterations());

  // Unpause the server, and check
  worldControl(false, 0);
  testPaused(false);
  EXPECT_LT(0u, iterations());

  // Pause the server, and check
  worldControl(true, 0);
  testPaused(true);

  // Step forward 1 iteration
  uint64_t iters = iterations();
  worldControl(true, 1);
  EXPECT_EQ(iters + 1u, iterations());
  // The server should be paused after stepping.
  testPaused(true);

  // Step forward 10 iteration
  iters = iterations();
  worldControl(true, 10);
  EXPECT_EQ(iters + 10u, iterations());
  // The server should be paused after stepping.
  testPaused(true);

  // Unpause the server, and check
  worldControl(false, 0);
  testPaused(false);
  EXPECT_GT(iterations(), iters);

  // Stepping while unpaused should not change the pause state
  worldControl(false, 10);
  testPaused(false);
}
