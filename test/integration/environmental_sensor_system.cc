/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <cmath>

#include <gz/common/Filesystem.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector2d.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include "gz/transport/Node.hh"
#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"
#include "../helpers/Subscription.hh"
#include "../helpers/Util.hh"

using namespace gz;
using namespace sim;

/// \brief Test EnvironmentPreload system
class EnvironmentSensorTest : public InternalFixture<::testing::Test>
{
  /////////////////////////////////////////////////
  public: EnvironmentSensorTest()
  {
    node.Subscribe("/world/environmental_sensor_test/"
      "model/model_with_sensor/link/link/sensor/custom_sensor/"
      "environmental_sensor/humidity",
      &EnvironmentSensorTest::OnReceiveMsg,
      this);

    node.Subscribe("/wind_speed2d", &EnvironmentSensorTest::OnWind2d, this);
    node.Subscribe("/wind_speed3d", &EnvironmentSensorTest::OnWind3d, this);
  }

  /////////////////////////////////////////////////
  public: void OnReceiveMsg(const gz::msgs::Double& _msg)
  {
    // Data set is such that sensor value == time for the first second.
    double nsec = _msg.header().stamp().nsec();
    double sec = static_cast<double>(_msg.header().stamp().sec());
    auto time = nsec * 1e-9 + sec;
    EXPECT_NEAR(time, _msg.data(), 1e-9);
    this->receivedData = true;
  }

  /////////////////////////////////////////////////
  public: void OnWind2d(const gz::msgs::Vector3d& _msg)
  {
    EXPECT_NEAR(_msg.x(), 1, 1e-6);
    EXPECT_NEAR(_msg.y(), 0, 1e-6);
    EXPECT_NEAR(_msg.z(), 0, 1e-6);
    this->receivedWind2dData = true;
  }

  /////////////////////////////////////////////////
  public: void OnWind3d(const gz::msgs::Vector3d& _msg)
  {
    // Robot is rotated 90 degrees in yaw and has transform type set to local.
    // Wind is in global +x and therefore in robot's local frame it will be -y
    EXPECT_NEAR(_msg.x(), 0, 1e-3);
    EXPECT_NEAR(_msg.y(), -1, 1e-3);
    EXPECT_NEAR(_msg.z(), 0, 1e-3);
    this->receivedWind3dData = true;
  }

  /////////////////////////////////////////////////
  public: std::atomic<bool> receivedData{false};

  /////////////////////////////////////////////////
  public: std::atomic<bool> receivedWind2dData{false};

  /////////////////////////////////////////////////
  public: std::atomic<bool> receivedWind3dData{false};

  /////////////////////////////////////////////////
  public: transport::Node node;

};

/// \brief Test EnvironmentalSensorSystem reset behavior without timestamp
/// assertions.
class EnvironmentSensorResetTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(EnvironmentSensorTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(CanPreload))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "test",
      "worlds", "environmental_sensor.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  // Run server
  server.Run(true, 1000, false);
  EXPECT_TRUE(this->receivedData.load());
  EXPECT_TRUE(this->receivedWind2dData.load());
  EXPECT_TRUE(this->receivedWind3dData.load());
}

/////////////////////////////////////////////////
TEST_F(EnvironmentSensorResetTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetRestoresEarlyReadings))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "test",
      "worlds", "environmental_sensor.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string humidityTopic =
      "/world/environmental_sensor_test/model/model_with_sensor/link/link/"
      "sensor/custom_sensor/environmental_sensor/humidity";

  transport::Node node;
  Subscription<msgs::Double> initialHumidity;
  Subscription<msgs::Vector3d> initialWind2d;
  Subscription<msgs::Vector3d> initialWind3d;
  initialHumidity.Subscribe(node, humidityTopic, 1);
  initialWind2d.Subscribe(node, "/wind_speed2d", 1);
  initialWind3d.Subscribe(node, "/wind_speed3d", 1);

  auto waitForHumidity =
      [&server](Subscription<msgs::Double> &_subscription, auto _predicate)
      {
        return test::StepUntil(server, 2000, [&]
        {
          return _subscription.Count() > 0u &&
              _predicate(_subscription.Last());
        });
      };
  auto waitForWind =
      [&server](Subscription<msgs::Vector3d> &_subscription, auto _predicate)
      {
        return test::StepUntil(server, 2000, [&]
        {
          return _subscription.Count() > 0u &&
              _predicate(_subscription.Last());
        });
      };

  ASSERT_TRUE(waitForHumidity(initialHumidity,
      [](const msgs::Double &_msg)
      {
        return _msg.data() < 0.2;
      }));
  const auto baselineHumidity = initialHumidity.Last();

  ASSERT_TRUE(waitForWind(initialWind2d,
      [](const msgs::Vector3d &_msg)
      {
        return std::abs(_msg.x() - 1.0) < 1e-6 &&
            std::abs(_msg.y()) < 1e-6 &&
            std::abs(_msg.z()) < 1e-6;
      }));
  ASSERT_TRUE(waitForWind(initialWind3d,
      [](const msgs::Vector3d &_msg)
      {
        return std::abs(_msg.x()) < 1e-3 &&
            std::abs(_msg.y() + 1.0) < 1e-3 &&
            std::abs(_msg.z()) < 1e-3;
      }));

  ASSERT_TRUE(waitForHumidity(initialHumidity,
      [](const msgs::Double &_msg)
      {
        return _msg.data() > 0.7;
      }));
  const auto preResetHumidity = initialHumidity.Last();
  EXPECT_GT(preResetHumidity.data(), baselineHumidity.data() + 0.6);

  server.ResetAll();

  // Re-subscribe after reset to avoid accepting late pre-reset transport data.
  transport::Node postResetNode;
  Subscription<msgs::Double> postResetHumidity;
  Subscription<msgs::Vector3d> postResetWind2d;
  Subscription<msgs::Vector3d> postResetWind3d;
  postResetHumidity.Subscribe(postResetNode, humidityTopic, 1);
  postResetWind2d.Subscribe(postResetNode, "/wind_speed2d", 1);
  postResetWind3d.Subscribe(postResetNode, "/wind_speed3d", 1);

  ASSERT_TRUE(waitForHumidity(postResetHumidity,
      [&preResetHumidity](const msgs::Double &_msg)
      {
        return _msg.data() < 0.2 &&
            _msg.data() < preResetHumidity.data() - 0.6;
      }));
  const auto postResetHumidityMsg = postResetHumidity.Last();
  EXPECT_NEAR(baselineHumidity.data(), postResetHumidityMsg.data(), 0.2);

  ASSERT_TRUE(waitForWind(postResetWind2d,
      [](const msgs::Vector3d &_msg)
      {
        return std::abs(_msg.x() - 1.0) < 1e-6 &&
            std::abs(_msg.y()) < 1e-6 &&
            std::abs(_msg.z()) < 1e-6;
      }));
  ASSERT_TRUE(waitForWind(postResetWind3d,
      [](const msgs::Vector3d &_msg)
      {
        return std::abs(_msg.x()) < 1e-3 &&
            std::abs(_msg.y() + 1.0) < 1e-3 &&
            std::abs(_msg.z()) < 1e-3;
      }));
}
