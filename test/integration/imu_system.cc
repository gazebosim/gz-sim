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

#include <ignition/msgs/imu.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test ImuTest system
class ImuTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

std::mutex mutex;
std::vector<msgs::IMU> imuMsgs;

/////////////////////////////////////////////////
void imuCb(const msgs::IMU &_msg)
{
  mutex.lock();
  imuMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling imu
TEST_F(ImuTest, ModelFalling)
{
  double z = 3;
  // TODO(anyone): get step size from sdf
  double stepSize = 0.001;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/imu.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "imu_sensor";

  auto topic =
      "world/imu_sensor/model/imu_model/link/link/sensor/imu_sensor/imu";

  // Create a system that records imu data
  test::Relay testSystem;
  math::Vector3d worldGravity;
  std::vector<math::Pose3d> poses;
  std::vector<math::Vector3d> accelerations;
  std::vector<math::Vector3d> angularVelocities;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Imu,
                  components::Name,
                  components::WorldPose,
                  components::AngularVelocity,
                  components::LinearAcceleration>(
            [&](const ignition::gazebo::Entity &_entity,
                const components::Imu *,
                const components::Name *_name,
                const components::WorldPose *_worldPose,
                const components::AngularVelocity *_angularVel,
                const components::LinearAcceleration *_linearAcc) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              poses.push_back(_worldPose->Data());
              accelerations.push_back(_linearAcc->Data());
              angularVelocities.push_back(_angularVel->Data());

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              return true;
            });

        _ecm.Each<components::Gravity>(
            [&](const ignition::gazebo::Entity &,
                const components::Gravity *_gravity) -> bool
            {
              // gtest is having a hard time with ASSERTs inside nested lambdas
              EXPECT_NE(nullptr, _gravity);
              if (nullptr != _gravity)
                worldGravity = _gravity->Data();
              return false;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to imu topic
  transport::Node node;
  node.Subscribe(topic, &imuCb);

  // step world and verify imu's linear acceleration is zero on free fall
  // Run server
  size_t iters200 = 200u;
  server.Run(true, iters200, false);
  EXPECT_EQ(iters200, accelerations.size());

  EXPECT_NEAR(worldGravity.X(), 0, TOL);
  EXPECT_NEAR(worldGravity.Y(), 0, TOL);
  EXPECT_NEAR(worldGravity.Z(), -5, TOL);

  EXPECT_NEAR(accelerations.back().X(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Y(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Z(), worldGravity.Z(), TOL);

  // Check we received messages

  EXPECT_GT(imuMsgs.size(), 0u);
  mutex.lock();
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->x(), 0, TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->y(), 0, TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->z(), 0, TOL);
  mutex.unlock();

  server.Run(true, 1u, false);
  EXPECT_NEAR(accelerations.back().X(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Y(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Z(), worldGravity.Z(), TOL);
  mutex.lock();
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->x(), 0, TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->y(), 0, TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->z(), 0, TOL);
  mutex.unlock();

  // Predict time of contact with ground plane.
  double tHit = sqrt((z-0.5) / (-worldGravity.Z()));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit + 0.5 - (iters200 + 1) * stepSize;
  double steps = ceil(dtHit / stepSize);
  ASSERT_GT(steps, 0);
  server.Run(true, steps, false);

  // Get the gravity vector for the final position of the box
  math::Vector3d gravity =
    poses.back().Rot().Inverse().RotateVector(worldGravity);

  EXPECT_NEAR(accelerations.back().X(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Y(), 0, TOL);
  EXPECT_NEAR(accelerations.back().Z(), 0, TOL);
  // Compare sensed gravity values against the gravity sensed for that position
  mutex.lock();
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->x(),
      -gravity.X(), TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->y(),
      -gravity.Y(), TOL);
  EXPECT_NEAR(imuMsgs.back().mutable_linear_acceleration()->z(),
      -gravity.Z(), TOL);
  mutex.unlock();

  // Verify reported name
  std::string scopedName = "imu_model::link::imu_sensor";
  mutex.lock();
  EXPECT_EQ(imuMsgs.back().entity_name(), scopedName);
  mutex.unlock();
}
