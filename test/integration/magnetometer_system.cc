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

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
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

  // Create a system that records magnetometer data
  test::Relay testSystem;

  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Magnetometer,
                  components::Name,
                  components::WorldPose>(
            [&](const Entity &_entity,
                const components::Magnetometer *,
                const components::Name *_name,
                const components::WorldPose *_worldPose) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);
              poses.push_back(_worldPose->Data());

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to magnetometer topic
  transport::Node node;
  node.Subscribe(topic, &magnetometerCb);

  // step world and verify magnetometer's detected field
  // Run server
  size_t iters = 200u;
  server.Run(true, iters, false);
  EXPECT_EQ(iters, poses.size());

  // Hardcoded SDF values
  math::Vector3d worldMagneticField(0.94, 0.76, -0.12);

  math::Vector3d field = poses.back().Rot().Inverse().RotateVector(
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
