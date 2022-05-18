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

#include <ignition/msgs/navsat.pb.h>
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utils/ExtraTestMacros.hh>

#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/NavSat.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "gz/sim/test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

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
TEST_F(NavSatTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(ModelFalling))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "navsat.sdf"));

  auto topic = "world/navsat_sensor/"
      "model/navsat_model/link/link/sensor/navsat_sensor/navsat";

  bool checkedComponents{false};
  fixture.OnPostUpdate(
    [&](
      const gazebo::UpdateInfo &,
      const gazebo::EntityComponentManager &_ecm)
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
