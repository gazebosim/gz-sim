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

#include <gz/msgs/navsat.pb.h>
#include <cmath>
#include <mutex>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/NavSat.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"
#include "../helpers/Subscription.hh"
#include "../helpers/Util.hh"

using namespace gz;
using namespace sim;

/// \brief Test NavSatTest system
class NavSatTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::NavSat> navSatMsgs;

/////////////////////////////////////////////////
void navsatCb(const msgs::NavSat &_msg)
{
  mutex.lock();
  navSatMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
bool earlyEpisodeNavSatReading(const msgs::NavSat &_baseline,
    const msgs::NavSat &_actual)
{
  return _baseline.frame_id() == _actual.frame_id() &&
      std::abs(_baseline.latitude_deg() - _actual.latitude_deg()) < 1e-9 &&
      std::abs(_baseline.longitude_deg() - _actual.longitude_deg()) < 1e-9 &&
      std::abs(_actual.velocity_east()) < 1e-6 &&
      std::abs(_actual.velocity_north()) < 1e-6 &&
      _actual.altitude() > -1.0 &&
      _actual.velocity_up() > -5.0;
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling navsat
TEST_F(NavSatTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelFalling))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "navsat.sdf"));

  auto topic = "world/navsat_sensor/"
      "model/navsat_model/link/link/sensor/navsat_sensor/navsat";

  bool checkedComponents{false};
  fixture.OnPostUpdate(
    [&](
      const sim::UpdateInfo &,
      const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Sensor, components::NavSat, components::Name,
                components::SensorTopic>(
          [&](const gz::sim::Entity &,
              const components::Sensor *,
              const components::NavSat *,
              const components::Name *_name,
              const components::SensorTopic *_topic) -> bool
          {
            EXPECT_EQ(_name->Data(), "navsat_sensor");
            EXPECT_EQ(topic, _topic->Data());

            checkedComponents = true;
            return true;
          });
    }).Finalize();

  // subscribe to navsat topic
  transport::Node node;
  node.Subscribe(topic, &navsatCb);

  // Run server
  fixture.Server()->Run(true, 300u, false);
  EXPECT_TRUE(checkedComponents);

  // Wait for messages to be received
  int sleep{0};
  int maxSleep{30};
  for (; navSatMsgs.size() < 10 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_LE(sleep, maxSleep);
  EXPECT_LE(10u, navSatMsgs.size());

  mutex.lock();
  auto lastMsg = navSatMsgs.back();
  mutex.unlock();

  EXPECT_EQ("navsat_model::link::navsat_sensor", lastMsg.frame_id());

  // Located at world origin
  EXPECT_DOUBLE_EQ(-22.9, lastMsg.latitude_deg());
  EXPECT_DOUBLE_EQ(-43.2, lastMsg.longitude_deg());

  // Not moving horizontally
  EXPECT_DOUBLE_EQ(0.0, lastMsg.velocity_east());
  EXPECT_DOUBLE_EQ(0.0, lastMsg.velocity_north());

  // Falling due to gravity
  EXPECT_GT(0.0, lastMsg.altitude());
  EXPECT_GT(0.0, lastMsg.velocity_up());
}

/////////////////////////////////////////////////
TEST_F(NavSatTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetRestoresEarlyReadings))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "navsat.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string topic = "world/navsat_sensor/"
      "model/navsat_model/link/link/sensor/navsat_sensor/navsat";

  transport::Node node;
  Subscription<msgs::NavSat> initialNavSat;
  initialNavSat.Subscribe(node, topic, 1);
  auto waitForNavSat =
      [&server](Subscription<msgs::NavSat> &_subscription, auto _predicate)
      {
        return test::StepUntil(server, 2000, [&]
        {
          return _subscription.Count() > 0u &&
              _predicate(_subscription.Last());
        });
      };

  ASSERT_TRUE(waitForNavSat(initialNavSat,
      [](const msgs::NavSat &) { return true; }));
  const auto baseline = initialNavSat.Last();

  // Let the falling model move away from the early-episode reading.
  server.Run(true, 1000, false);
  const auto preReset = initialNavSat.Last();
  EXPECT_LT(preReset.altitude(), baseline.altitude() - 1.0);
  EXPECT_LT(preReset.velocity_up(), baseline.velocity_up() - 1.0);
  server.ResetAll();

  transport::Node postResetNode;
  Subscription<msgs::NavSat> postResetNavSat;
  postResetNavSat.Subscribe(postResetNode, topic, 1);

  msgs::NavSat postReset;
  ASSERT_TRUE(waitForNavSat(postResetNavSat,
      [&baseline, &preReset, &postReset](const msgs::NavSat &_msg)
      {
        if (!earlyEpisodeNavSatReading(baseline, _msg))
        {
          return false;
        }
        if (_msg.altitude() <= preReset.altitude() + 1.0 ||
            _msg.velocity_up() <= preReset.velocity_up() + 1.0)
        {
          return false;
        }
        postReset = _msg;
        return true;
      }));
  EXPECT_EQ(baseline.frame_id(), postReset.frame_id());
  EXPECT_NEAR(baseline.latitude_deg(), postReset.latitude_deg(), 1e-9);
  EXPECT_NEAR(baseline.longitude_deg(), postReset.longitude_deg(), 1e-9);
  EXPECT_NEAR(0.0, postReset.velocity_east(), 1e-6);
  EXPECT_NEAR(0.0, postReset.velocity_north(), 1e-6);
}
