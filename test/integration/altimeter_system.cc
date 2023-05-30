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

#include <gz/msgs/altimeter.pb.h>
#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test AltimeterTest system
class AltimeterTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::Altimeter> altMsgs;

/////////////////////////////////////////////////
void altimeterCb(const msgs::Altimeter &_msg)
{
  mutex.lock();
  altMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling altimeter
// See: https://github.com/gazebosim/gz-sim/issues/1175
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(AltimeterTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ModelFalling))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/altimeter.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "altimeter_sensor";

  auto topic = "world/altimeter_sensor/"
      "model/altimeter_model/link/link/sensor/altimeter_sensor/altimeter";

  // Create a system that records altimeter data
  test::Relay testSystem;
  std::vector<math::Pose3d> poses;
  std::vector<math::Vector3d> velocities;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Altimeter, components::Name,
                  components::WorldPose,
                  components::WorldLinearVelocity>(
            [&](const Entity &_entity,
                const components::Altimeter *,
                const components::Name *_name,
                const components::WorldPose *_worldPose,
                const components::WorldLinearVelocity *_worldLinearVel) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              poses.push_back(_worldPose->Data());
              velocities.push_back(_worldLinearVel->Data());

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

  // subscribe to altimeter topic
  transport::Node node;
  node.Subscribe(topic, &altimeterCb);

  // Run server
  size_t iters100 = 100u;
  server.Run(true, iters100, false);
  EXPECT_EQ(iters100, poses.size());

  // Wait for messages to be received
  size_t updateRate = 30;
  double stepSize = 0.001;
  size_t waitForMsgs =
      static_cast<size_t>(poses.size() * stepSize * updateRate + 1);
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    bool received = altMsgs.size() == waitForMsgs;
    mutex.unlock();

    if (received)
      break;
  }

  mutex.lock();
  EXPECT_EQ(altMsgs.size(), waitForMsgs);
  auto firstMsg = altMsgs.front();
  auto lastMsg = altMsgs.back();
  mutex.unlock();

  // check altimeter world pose
  // verify the altimeter model z pos is decreasing
  EXPECT_GT(poses.front().Pos().Z(), poses.back().Pos().Z());
  EXPECT_GT(poses.back().Pos().Z(), 0.0);
  // velocity should be negative in z
  EXPECT_GT(velocities.front().Z(), velocities.back().Z());
  EXPECT_LT(velocities.back().Z(), 0.0);

  // check altimeter sensor data
  // vertical position = world position - intial position
  // so since altimeter is falling, vertical position should be negative
  EXPECT_GT(firstMsg.vertical_position(),
      lastMsg.vertical_position());
  EXPECT_LT(lastMsg.vertical_position(), 0.0);
  // vertical velocity should be negative
  EXPECT_GT(firstMsg.vertical_velocity(),
      lastMsg.vertical_velocity());
  EXPECT_LT(lastMsg.vertical_velocity(), 0.0);

  // Run server for longer period of time so the altimeter falls then rests
  // on the ground plane
  size_t iters1000 = 1000u;
  server.Run(true, iters1000, false);
  EXPECT_EQ(iters100 + iters1000, poses.size());

  // Wait for messages to be received
  waitForMsgs = static_cast<size_t>(poses.size() * stepSize * updateRate + 1);
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    bool received = altMsgs.size() == waitForMsgs;
    mutex.unlock();

    if (received)
      break;
  }

  mutex.lock();
  EXPECT_EQ(altMsgs.size(), waitForMsgs);
  lastMsg = altMsgs.back();
  mutex.unlock();

  // check altimeter world pose
  // altimeter should be on the ground
  // note that altimeter's parent link is at an 0.05 offset
  // set high tol due to instablity
  EXPECT_NEAR(poses.back().Pos().Z(), 0.05, 1e-2);
  // velocity should now be zero as the model is resting on the ground
  EXPECT_NEAR(velocities.back().Z(), 0u, 1e-3);

  // check altimeter sensor data
  // altimeter vertical position = 0.05 (world pos) - 3.05 (initial position)
  EXPECT_LT(lastMsg.vertical_position(), -3.0);
  // velocity should be zero
  EXPECT_NEAR(lastMsg.vertical_velocity(), 0u, 1e-3);
}
