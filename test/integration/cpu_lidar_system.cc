/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <mutex>
#include <vector>

#include <gz/msgs/laserscan.pb.h>

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

class CpuLidarTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(CpuLidarTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(SensorDiscoveryAndRaycastData))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/cpu_lidar.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool cpuLidarFound = false;
  bool raycastDataFound = false;

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

        if (_info.iterations > 1)
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

    if (_info.iterations >= 3)
    {
      _ecm.Each<components::CpuLidar,
                components::RaycastData>(
        [&](const Entity &,
            const components::CpuLidar *,
            const components::RaycastData *_rayData) -> bool
        {
          raycastDataFound = true;
          EXPECT_EQ(640u, _rayData->Data().rays.size());

          for (const auto &ray : _rayData->Data().rays)
          {
            EXPECT_GT((ray.end - ray.start).Length(), 0.0);
          }
          return true;
        });
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 10, false);
  EXPECT_TRUE(cpuLidarFound);
  EXPECT_TRUE(raycastDataFound);
}

/////////////////////////////////////////////////
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

  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    if (_info.iterations < 5) return;

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
  server.Run(true, 20, false);
  EXPECT_TRUE(resultsFound);
}

/////////////////////////////////////////////////
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

  std::mutex mutex;
  std::vector<msgs::LaserScan> received;

  transport::Node node;
  std::function<void(const msgs::LaserScan &)> cb =
    [&](const msgs::LaserScan &_msg)
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(_msg);
    };
  node.Subscribe("/test/cpu_lidar", cb);

  server.Run(true, 200, false);

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_GT(received.size(), 0u);

  auto &scan = received.back();
  EXPECT_EQ(640, scan.count());
  EXPECT_DOUBLE_EQ(0.08, scan.range_min());
  EXPECT_DOUBLE_EQ(10.0, scan.range_max());

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
