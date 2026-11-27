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

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/navsat.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "../plugins/MockSystem.hh"

#include "gz/sim/Link.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/NavSat.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

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
// The test checks the world pose and sensor readings of a falling navsat
TEST_F(NavSatTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelFalling))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "navsat.sdf"));
  Server server(serverConfig);

  auto topic = "world/navsat_sensor/"
      "model/navsat_model/link/link/sensor/navsat_sensor/navsat";

  // subscribe to navsat topic
  transport::Node node;
  node.Subscribe(topic, &navsatCb);

  // Run server
  server.Run(true, 300u, false);

  // Check the sensor entity. The world shares the sensor's name, so the
  // lookup also asks for the Sensor component to make sure we get the sensor.
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    const Entity sensor = ecm->EntityByComponents(
        components::Sensor(), components::Name("navsat_sensor"));
    ASSERT_NE(kNullEntity, sensor);
    EXPECT_NE(nullptr, ecm->Component<components::NavSat>(sensor));

    const auto *topicComp = ecm->Component<components::SensorTopic>(sensor);
    ASSERT_NE(nullptr, topicComp);
    EXPECT_EQ(topic, topicComp->Data());
  }

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

  // Horizontal velocity is zero within ENU conversion roundoff.
  EXPECT_NEAR(0.0, lastMsg.velocity_east(), 1e-12);
  EXPECT_NEAR(0.0, lastMsg.velocity_north(), 1e-12);

  // Falling due to gravity
  EXPECT_GT(0.0, lastMsg.altitude());
  EXPECT_GT(0.0, lastMsg.velocity_up());
}

/////////////////////////////////////////////////
// World +X points east at heading 0 and north at heading +90 degrees.
TEST_F(NavSatTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(VelocityHeading))
{
  struct HeadingCase
  {
    std::string worldName;
    double east;
    double north;
  };
  const HeadingCase cases[] = {
      {"navsat_heading", 1, 0}, {"navsat_heading_90", 0, 1}};
  for (const auto &heading : cases)
  {
    SCOPED_TRACE(heading.worldName);
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", heading.worldName + ".sdf"));
    Server server(serverConfig);

    auto setup = std::make_shared<MockSystem>();
    setup->preUpdateCallback =
        [](const UpdateInfo &_info, EntityComponentManager &_ecm)
        {
          if (_info.iterations != 1)
            return;
          const auto link = _ecm.EntityByComponents(
              components::Link(), components::Name("link"));
          ASSERT_NE(kNullEntity, link);
          Link(link).SetLinearVelocity(_ecm, {1, 0, 0});
        };
    ASSERT_TRUE(server.AddSystem(setup).value_or(false));

    struct Collector
    {
      std::condition_variable cv;
      std::mutex mutex;
      std::optional<msgs::NavSat> msg;
    };
    const auto collector = std::make_shared<Collector>();
    std::function<void(const msgs::NavSat &)> callback =
        [collector](const msgs::NavSat &_msg)
        {
          if (_msg.header().stamp().sec() < 1)
            return;
          std::lock_guard<std::mutex> lock(collector->mutex);
          collector->msg = _msg;
          collector->cv.notify_one();
        };
    transport::Node node;
    const auto topic = "world/" + heading.worldName +
        "/model/navsat_model/link/link/sensor/navsat_sensor/navsat";
    ASSERT_TRUE(node.Subscribe(topic, callback));
    ASSERT_TRUE(server.Run(true, 2000u, false));

    std::unique_lock<std::mutex> lock(collector->mutex);
    ASSERT_TRUE(collector->cv.wait_for(lock, std::chrono::seconds(3),
        [&] { return collector->msg.has_value(); }));
    const auto &msg = *collector->msg;
    EXPECT_NEAR(heading.east, msg.velocity_east(), 2e-3);
    EXPECT_NEAR(heading.north, msg.velocity_north(), 2e-3);
    EXPECT_NEAR(0, msg.velocity_up(), 2e-3);
  }
}
