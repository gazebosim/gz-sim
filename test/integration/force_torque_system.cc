/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/ForceTorque.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/Relay.hh"
#include "helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class ForceTorqueTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ForceTorqueTest, MeasureWeight)
{
  using namespace std::chrono_literals;
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile =
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/force_torque.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1us);

  const std::string sensorName = "force_torque_sensor";

  size_t iters = 1000u;

  std::vector<msgs::Wrench> wrenches;
  wrenches.reserve(iters);
  std::mutex wrenchMutex;
  std::condition_variable cv;
  auto wrenchCb = std::function<void(const msgs::Wrench &)>(
      [&wrenchMutex, &wrenches, &cv, iters](const auto &_msg)
      {
        std::lock_guard lock(wrenchMutex);
        wrenches.push_back(_msg);
        if (wrenches.size() >= iters)
        {
          cv.notify_all();
        }
      });

  transport::Node node;
  node.Subscribe("/force_torque", wrenchCb);

  // Run server
  server.Run(true, iters, false);
  ASSERT_EQ(iters, *server.IterationCount());

  {
    std::unique_lock lock(wrenchMutex);
    cv.wait_for(lock, 30s, [&] { return wrenches.size() >= iters; });
    ASSERT_EQ(iters, wrenches.size());

    const double kSensorMass = 0.2;
    const double kWeightMass = 10;
    const double kGravity = 9.8;
    const auto &wrench = wrenches.back();
    EXPECT_NEAR(kGravity * (kSensorMass + kWeightMass), wrench.force().z(),
                1e-3);
  }
}
