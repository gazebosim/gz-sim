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
#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/LinearVelocity.hh"
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

  // Not moving horizontally
  EXPECT_DOUBLE_EQ(0.0, lastMsg.velocity_east());
  EXPECT_DOUBLE_EQ(0.0, lastMsg.velocity_north());

  // Falling due to gravity
  EXPECT_GT(0.0, lastMsg.altitude());
  EXPECT_GT(0.0, lastMsg.velocity_up());
}
