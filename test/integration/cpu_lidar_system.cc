/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/CpuLidar.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/RaycastData.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

// Named constants for iteration counts used in test scenarios.
// These values control how many simulation iterations each test runs
// to ensure sufficient time for sensor discovery and data generation.
namespace kIterations
{
/// \brief Iterations for basic sensor discovery test
static constexpr int kDiscoveryIterations = 10;
/// \brief Iterations for raycast data validity test
static constexpr int kRaycastDataIterations = 10;
/// \brief Iterations for raycast results processing test
static constexpr int kRaycastResultsIterations = 200;
/// \brief Iterations for laser scan publication test
static constexpr int kLaserScanIterations = 200;
/// \brief Iteration threshold for sensor topic verification
static constexpr int kTopicVerificationIteration = 2;
/// \brief Iteration threshold for raycast data checking
static constexpr int kRaycastCheckIteration = 3;
/// \brief Iteration threshold for results processing
static constexpr int kResultsCheckIteration = 5;
/// \brief Timeout duration in milliseconds for message waiting
static constexpr int kMessageTimeoutMs = 5000;
/// \brief Iterations for point cloud publication test
static constexpr int kPointCloudIterations = 200;
}

class CpuLidarTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Test for sensor discovery - verifies that the CPU lidar sensor
// is properly discovered and configured with the correct name and topic.
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(Discovery))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool cpuLidarFound = false;

  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    _ecm.Each<components::CpuLidar,
              components::Name>(
      [&](const Entity &_entity,
          const components::CpuLidar *,
          const components::Name *_name) -> bool
      {
        cpuLidarFound = true;
        EXPECT_EQ("cpu_lidar", _name->Data());

        auto sensorComp = _ecm.Component<components::Sensor>(_entity);
        EXPECT_NE(nullptr, sensorComp);

        // Verify sensor topic is set after initial discovery
        if (_info.iterations > kIterations::kTopicVerificationIteration)
        {
          auto topicComp =
              _ecm.Component<components::SensorTopic>(_entity);
          EXPECT_NE(nullptr, topicComp);
          if (topicComp)
          {
            EXPECT_EQ("/test/cpu_lidar", topicComp->Data());
          }
        }
        return true;
      });
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, kIterations::kDiscoveryIterations, false);
  EXPECT_TRUE(cpuLidarFound);
}

/////////////////////////////////////////////////
// Test for raycast data validity - verifies that the CPU lidar
// sensor generates valid raycast data with proper structure.
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RaycastDataValidity))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool raycastDataFound = false;

  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    // Check for raycast data after sufficient iterations
    if (_info.iterations >= kIterations::kRaycastCheckIteration)
    {
      _ecm.Each<components::CpuLidar,
                components::RaycastData>(
        [&](const Entity &,
            const components::CpuLidar *,
            const components::RaycastData *_rayData) -> bool
        {
          raycastDataFound = true;
          EXPECT_EQ(640u, _rayData->Data().rays.size());

          // Verify each ray has valid start and end points
          for (const auto &ray : _rayData->Data().rays)
          {
            EXPECT_GT((ray.end - ray.start).Length(), 0.0);
          }
          return true;
        });
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, kIterations::kRaycastDataIterations, false);
  EXPECT_TRUE(raycastDataFound);
}

/////////////////////////////////////////////////
// Test for raycast results processing - verifies that the CPU lidar
// sensor properly processes and stores raycast results.
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RaycastResultsProcessed))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool resultsFound = false;

  // Subscribe to the sensor topic so HasConnections() returns true
  // and the system requests raycasts from Physics.
  transport::Node node;
  node.Subscribe("/test/cpu_lidar", [](const msgs::LaserScan &){});

  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    // Wait for sufficient iterations before checking results
    if (_info.iterations < kIterations::kResultsCheckIteration) return;

    _ecm.Each<components::CpuLidar,
              components::RaycastData>(
      [&](const Entity &,
          const components::CpuLidar *,
          const components::RaycastData *_rayData) -> bool
      {
        if (!_rayData->Data().results.empty())
        {
          resultsFound = true;
          EXPECT_EQ(_rayData->Data().rays.size(),
                    _rayData->Data().results.size());
        }
        return true;
      });
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, kIterations::kRaycastResultsIterations, false);
  EXPECT_TRUE(resultsFound);
}

/////////////////////////////////////////////////
// Test for laser scan publication - verifies that the CPU lidar
// sensor publishes laser scan messages correctly with valid range data.
// Uses condition_variable and mutex to wait for messages with timeout.
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(LaserScanPublished))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Synchronization primitives for thread-safe message waiting
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<msgs::LaserScan> received;
  bool messageReceived = false;

  transport::Node node;
  node.Subscribe("/test/cpu_lidar",
    [&](const msgs::LaserScan &_msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(_msg);
      messageReceived = true;
      cv.notify_one();
    });

  // Run server in background and wait for message with timeout
  std::thread serverThread([&]()
  {
    server.Run(true, kIterations::kLaserScanIterations, false);
  });

  // Wait for message with timeout using condition_variable
  {
    std::unique_lock<std::mutex> lock(mutex);
    auto timeout = std::chrono::milliseconds(kIterations::kMessageTimeoutMs);
    if (!cv.wait_for(lock, timeout, [&]{ return messageReceived; }))
    {
      // Timeout reached without receiving message
    }
  }

  // Ensure server thread completes
  serverThread.join();

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_GT(received.size(), 0u);

  auto &scan = received.back();
  EXPECT_EQ(640, scan.count());
  EXPECT_DOUBLE_EQ(0.08, scan.range_min());
  EXPECT_DOUBLE_EQ(10.0, scan.range_max());

  // Verify all range values are valid (not inf or nan and within bounds)
  bool anyHit = false;
  for (int i = 0; i < scan.ranges_size(); ++i)
  {
    if (!std::isinf(scan.ranges(i)) && !std::isnan(scan.ranges(i)))
    {
      anyHit = true;
      EXPECT_GT(scan.ranges(i), 0.08);
      EXPECT_LT(scan.ranges(i), 10.0);
    }
  }
  EXPECT_TRUE(anyHit);
}

/////////////////////////////////////////////////
// Test for point cloud publication - verifies that the CPU lidar
// sensor publishes PointCloudPacked messages with valid data.
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(PointCloudPublished))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  std::mutex mutex;
  std::condition_variable cv;
  std::vector<msgs::PointCloudPacked> received;
  bool messageReceived = false;

  transport::Node node;
  node.Subscribe("/test/cpu_lidar/points",
    [&](const msgs::PointCloudPacked &_msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(_msg);
      messageReceived = true;
      cv.notify_one();
    });

  std::thread serverThread([&]()
  {
    server.Run(true, kIterations::kPointCloudIterations, false);
  });

  {
    std::unique_lock<std::mutex> lock(mutex);
    auto timeout = std::chrono::milliseconds(kIterations::kMessageTimeoutMs);
    static_cast<void>(
      cv.wait_for(lock, timeout, [&]{ return messageReceived; }));
  }

  serverThread.join();

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_GT(received.size(), 0u);

  auto &msg = received.back();
  EXPECT_EQ(640u, msg.width());
  EXPECT_EQ(1u, msg.height());
  EXPECT_GT(msg.data().size(), 0u);
  EXPECT_GT(msg.point_step(), 0u);
}
