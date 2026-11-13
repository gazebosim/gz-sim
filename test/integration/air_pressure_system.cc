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

#include <gz/msgs/fluid_pressure.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test AirPressureTest system
class AirPressureTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(AirPressureTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AirPressure))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/air_pressure.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "air_pressure_sensor";

  auto topic = "world/air_pressure_sensor/model/air_pressure_model/link/link/"
      "sensor/air_pressure_sensor/air_pressure";

  // Subscribe to air_pressure topic
  bool received{false};
  msgs::FluidPressure msg;
  msg.Clear();
  std::function<void(const msgs::FluidPressure &)>  cb =
      [&received, &msg](const msgs::FluidPressure &_msg)
  {
    // Only need one message
    if (received)
      return;

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(topic, cb);

  // Run server
  server.Run(true, 100, false);

  // Check the sensor entity. The world shares the sensor's name, so the
  // lookup also asks for the Sensor component to make sure we get the sensor.
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    const Entity sensor = ecm->EntityByComponents(
        components::Sensor(), components::Name(sensorName));
    ASSERT_NE(kNullEntity, sensor);
    EXPECT_NE(nullptr, ecm->Component<components::AirPressureSensor>(sensor));

    // The topic component is only created after the first update
    const auto *topicComp = ecm->Component<components::SensorTopic>(sensor);
    ASSERT_NE(nullptr, topicComp);
    EXPECT_EQ(topic, topicComp->Data());
  }

  // Wait for message to be received
  for (int sleep = 0; !received && sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(received);

  // check air pressure
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_EQ(0, msg.header().stamp().sec());
  EXPECT_LT(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(101325.0, msg.pressure());
  EXPECT_DOUBLE_EQ(0.0, msg.variance());
}
