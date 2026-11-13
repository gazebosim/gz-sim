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

#include <gz/msgs/magnetometer.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test MagnetometerTest system
class MagnetometerTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::Magnetometer> magnetometerMsgs;

/////////////////////////////////////////////////
void magnetometerCb(const msgs::Magnetometer &_msg)
{
  mutex.lock();
  magnetometerMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the detected field from a rotated magnetometer
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(MagnetometerTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(RotatedMagnetometer))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/magnetometer.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "magnetometer_sensor";

  auto topic = "world/magnetometer_sensor/model/magnetometer_model/link/link/"
      "sensor/magnetometer_sensor/magnetometer";

  // subscribe to magnetometer topic
  transport::Node node;
  node.Subscribe(topic, &magnetometerCb);

  // step world and verify magnetometer's detected field
  // Run server
  size_t iters = 200u;
  server.Run(true, iters, false);

  // Get the sensor's world pose. The world shares the sensor's name, so the
  // lookup also asks for the Sensor component.
  math::Pose3d sensorPose;
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    const Entity sensor = ecm->EntityByComponents(
        components::Sensor(), components::Name(sensorName));
    ASSERT_NE(kNullEntity, sensor);
    EXPECT_NE(nullptr, ecm->Component<components::Magnetometer>(sensor));

    const auto *topicComp = ecm->Component<components::SensorTopic>(sensor);
    ASSERT_NE(nullptr, topicComp);
    EXPECT_EQ(topic, topicComp->Data());

    const auto *worldPose = ecm->Component<components::WorldPose>(sensor);
    ASSERT_NE(nullptr, worldPose);
    sensorPose = worldPose->Data();
  }

  // Hardcoded SDF values
  math::Vector3d worldMagneticField(0.94, 0.76, -0.12);

  math::Vector3d field = sensorPose.Rot().Inverse().RotateVector(
        worldMagneticField);
  mutex.lock();
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->x(),
      field.X(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->y(),
      field.Y(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->z(),
      field.Z(), TOL);
  mutex.unlock();
}
