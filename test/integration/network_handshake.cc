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

#include <gz/utils/ExtraTestMacros.hh>

#include <gz/msgs/pose_v.pb.h>
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
// Get the current paused state from the world stats message
uint64_t testPaused(bool _paused)
{
  std::condition_variable condition;
  std::mutex mutex;
  transport::Node node;
  bool paused = !_paused;
  uint64_t iterations = 0;

  std::function<void(const msgs::WorldStatistics &)> cb =
      [&](const msgs::WorldStatistics &_msg)
  {
    std::unique_lock<std::mutex> lock(mutex);
    paused = _msg.paused();
    iterations = _msg.iterations();
    condition.notify_all();
  };

  std::unique_lock<std::mutex> lock(mutex);
  node.Subscribe("/world/default/stats", cb);
  auto success = condition.wait_for(lock, std::chrono::seconds(1));
  EXPECT_EQ(std::cv_status::no_timeout, success);
  EXPECT_EQ(_paused, paused);
  return iterations;
}

/////////////////////////////////////////////////
class NetworkHandshake : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(NetworkHandshake, GZ_UTILS_TEST_DISABLED_ON_WIN32(Handshake))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  serverConfig.SetNetworkRole("primary");
  serverConfig.SetNetworkSecondaries(2);
  serverConfig.SetUseLevels(true);

  auto serverPrimary = std::make_unique<Server>(serverConfig);
  serverPrimary->SetUpdatePeriod(1us);

  serverConfig.SetNetworkRole("secondary");
  serverConfig.SetUseLevels(true);
  auto serverSecondary1 = std::make_unique<Server>(serverConfig);
  auto serverSecondary2 = std::make_unique<Server>(serverConfig);

  int terminated{0};

  auto testFcn = [&](Server *_server)
  {
    // Run for a finite number of iterations
    EXPECT_TRUE(_server->Run(true, 10, false));

    terminated++;
  };

  auto primaryThread = std::thread(testFcn, serverPrimary.get());
  auto secondaryThread1 = std::thread(testFcn, serverSecondary1.get());
  auto secondaryThread2 = std::thread(testFcn, serverSecondary2.get());

  // Primary ended all iterations, shut it down so secondaries also stop
  int maxSleep = 30;
  for (int sleep = 0; sleep < maxSleep && terminated == 0; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(1, terminated);

  primaryThread.join();
  serverPrimary.reset();

  // Terminate secondaries
  for (int sleep = 0; sleep < maxSleep && terminated < 3; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(3, terminated);

  secondaryThread1.join();
  secondaryThread2.join();

  serverSecondary1.reset();
  serverSecondary2.reset();
}

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(NetworkHandshake, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Updates))
{
  auto pluginElem = std::make_shared<sdf::Element>();
  pluginElem->SetName("plugin");
  pluginElem->AddAttribute("name", "string", "required_but_ignored", true);
  pluginElem->AddAttribute("filename", "string", "required_but_ignored", true);

  // Primary
  ServerConfig::PluginInfo primaryPluginInfo;
  primaryPluginInfo.SetEntityName("default");
  primaryPluginInfo.SetEntityType("world");
  sdf::Plugin plugin;
  plugin.SetFilename("gz-sim-scene-broadcaster-system");
  plugin.SetName("gz::sim::systems::SceneBroadcaster");
  plugin.InsertContent(pluginElem);
  primaryPluginInfo.SetPlugin(plugin);

  ServerConfig configPrimary;
  configPrimary.SetNetworkRole("primary");
  configPrimary.SetUseLevels(true);
  // Can only test one secondary running physics, because running 2 physics in
  // the same process causes a segfault, see
  // https://github.com/gazebosim/gz-sim/issues/18
  configPrimary.SetNetworkSecondaries(1);
  configPrimary.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/performers.sdf");
  configPrimary.AddPlugin(primaryPluginInfo);

  auto serverPrimary = std::make_unique<Server>(configPrimary);

  // Secondary
  ServerConfig::PluginInfo secondaryPluginInfo;
  secondaryPluginInfo.SetEntityName("default");
  secondaryPluginInfo.SetEntityType("world");
  sdf::Plugin secondPlugin;
  secondPlugin.SetFilename("gz-sim-physics-system");
  secondPlugin.SetName("gz::sim::systems::Physics");
  secondPlugin.InsertContent(pluginElem);
  secondaryPluginInfo.SetPlugin(secondPlugin);

  ServerConfig configSecondary;
  configSecondary.SetNetworkRole("secondary");
  configSecondary.SetUseLevels(true);
  configSecondary.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/performers.sdf");
  configSecondary.AddPlugin(secondaryPluginInfo);

  auto serverSecondary1 = std::make_unique<Server>(configSecondary);

  // Subscribe to pose updates, which should come from the primary
  transport::Node node;
  std::vector<double> zPos;
  std::function<void(const msgs::Pose_V &)> cb =
      [&](const msgs::Pose_V &_msg)
  {
    for (int i = 0; i < _msg.pose().size(); ++i)
    {
      const auto &poseMsg = _msg.pose(i);

      if (poseMsg.name() == "sphere")
      {
        zPos.push_back(poseMsg.position().z());
      }
    }
  };
  node.Subscribe("/world/default/pose/info", cb);

  // Run
  std::atomic<bool> testRunning{true};
  auto testFcn = [&](Server *_server)
  {
    _server->Run(false, 0, false);

    while (testRunning)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };

  auto primaryThread = std::thread(testFcn, serverPrimary.get());
  auto secondaryThread1 = std::thread(testFcn, serverSecondary1.get());

  // Wait a few simulation iterations
  int maxSleep = 30;
  for (int sleep = 0; sleep < maxSleep && zPos.size() < 100; sleep++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // We don't assert because we still need to join the threads if the test fails
  EXPECT_LE(100u, zPos.size());

  // Check model was falling (physics simulated by secondary)
  if (zPos.size() >= 100u)
  {
    EXPECT_GT(zPos[0], zPos[99]);
  }

  // Finish server threads
  testRunning = false;

  primaryThread.join();
  secondaryThread1.join();

  serverPrimary.reset();
  serverSecondary1.reset();
}
