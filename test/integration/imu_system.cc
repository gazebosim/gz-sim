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

#include <gz/msgs/imu.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/LinearAcceleration.hh"
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

/// \brief Test ImuTest system
class ImuTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::IMU> imuMsgs;
msgs::IMU lastImuMsgENU;
msgs::IMU lastImuMsgNED;
msgs::IMU lastImuMsgNWU;
msgs::IMU lastImuMsgCUSTOM;
msgs::IMU lastImuMsgDEFAULT;

/////////////////////////////////////////////////
void imuENUCb(const msgs::IMU &_msg)
{
  lastImuMsgENU = _msg;
}

/////////////////////////////////////////////////
void imuNEDCb(const msgs::IMU &_msg)
{
  lastImuMsgNED = _msg;
}

/////////////////////////////////////////////////
void imuNWUCb(const msgs::IMU &_msg)
{
  lastImuMsgNWU = _msg;
}

/////////////////////////////////////////////////
void imuCUSTOMCb(const msgs::IMU &_msg)
{
  lastImuMsgCUSTOM = _msg;
}

/////////////////////////////////////////////////
void imuDEFULTCb(const msgs::IMU &_msg)
{
  lastImuMsgDEFAULT = _msg;
}

/////////////////////////////////////////////////
void clearLastImuMsgs()
{
  lastImuMsgCUSTOM.Clear();
  lastImuMsgENU.Clear();
  lastImuMsgNED.Clear();
  lastImuMsgNWU.Clear();
  lastImuMsgDEFAULT.Clear();
}

/////////////////////////////////////////////////
void imuCb(const msgs::IMU &_msg)
{
  mutex.lock();
  imuMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling imu
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ImuTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelFalling))
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
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Imu,
                  components::Name,
                  components::WorldPose,
                  components::AngularVelocity,
                  components::LinearAcceleration>(
            [&](const Entity &_entity,
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

        _ecm.Each<components::Gravity>(
            [&](const Entity &,
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
  int steps = static_cast<int>(ceil(dtHit / stepSize));
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

/////////////////////////////////////////////////
// The test checks to make sure orientation is not published if it is disabled
TEST_F(ImuTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OrientationDisabled))
{
  imuMsgs.clear();

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "imu_no_orientation.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topic =
      "world/imu_sensor/model/imu_model/link/link/sensor/imu_sensor/imu";

  // subscribe to imu topic
  transport::Node node;
  node.Subscribe(topic, &imuCb);

  // step world and verify imu's orientation is not published
  // Run server
  size_t iters200 = 200u;
  server.Run(true, iters200, false);

  // Check we received messages
  EXPECT_GT(imuMsgs.size(), 0u);
  mutex.lock();
  for (const auto &msg : imuMsgs)
  {
    EXPECT_FALSE(msg.has_orientation());
  }
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks if the orientation is published according to the
// localization tag
TEST_F(ImuTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(NamedFrames))
{
  imuMsgs.clear();
  clearLastImuMsgs();

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "imu_named_frame.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topicENU = "/imu_test_ENU";
  auto topicNED = "/imu_test_NED";
  auto topicNWU = "/imu_test_NWU";
  auto topicCUSTOM = "/imu_test_CUSTOM";
  auto topicDEFAULT = "/imu_test_DEFAULT";

  // subscribe to imu topic
  transport::Node node;
  node.Subscribe(topicENU, &imuENUCb);
  node.Subscribe(topicNED, &imuNEDCb);
  node.Subscribe(topicNWU, &imuNWUCb);
  node.Subscribe(topicCUSTOM, &imuCUSTOMCb);
  node.Subscribe(topicDEFAULT, &imuDEFULTCb);

  // Run server
  server.Run(true, 200u, false);

  // Check we received messages
  EXPECT_TRUE(lastImuMsgENU.has_orientation());
  EXPECT_TRUE(lastImuMsgNED.has_orientation());
  EXPECT_TRUE(lastImuMsgNWU.has_orientation());
  EXPECT_TRUE(lastImuMsgCUSTOM.has_orientation());
  EXPECT_TRUE(lastImuMsgDEFAULT.has_orientation());

  // For the DEFAULT msg, orientation is reported relative
  // to the original pose, which should be identity quaternion.
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().z(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().w(), 1, 1e-2);

  // For the ENU msg
  EXPECT_NEAR(lastImuMsgENU.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().z(), 1, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().w(), 0, 1e-2);

  // For the NED msg
  EXPECT_NEAR(lastImuMsgNED.orientation().x(), -0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().y(), 0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().z(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().w(), 0, 1e-2);

  // For the NWU msg
  EXPECT_NEAR(lastImuMsgNWU.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().z(), 0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().w(), 0.707, 1e-2);

  // For the CUSTOM msg
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().y(), 0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().z(), 0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().w(), 0, 1e-2);
}

/////////////////////////////////////////////////
// The test checks if the orientation is published according to the
// localization tag, with heading_deg also accounted for
TEST_F(ImuTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(NamedFramesWithHeading))
{
  imuMsgs.clear();
  clearLastImuMsgs();

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "imu_heading_deg.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topicENU = "/imu_test_ENU";
  auto topicNED = "/imu_test_NED";
  auto topicNWU = "/imu_test_NWU";
  auto topicCUSTOM = "/imu_test_CUSTOM";
  auto topicDEFAULT = "/imu_test_DEFAULT";

  // subscribe to imu topic
  transport::Node node;
  node.Subscribe(topicENU, &imuENUCb);
  node.Subscribe(topicNED, &imuNEDCb);
  node.Subscribe(topicNWU, &imuNWUCb);
  node.Subscribe(topicCUSTOM, &imuCUSTOMCb);
  node.Subscribe(topicDEFAULT, &imuDEFULTCb);

  // Run server
  server.Run(true, 200u, false);

  // Check we received messages
  EXPECT_TRUE(lastImuMsgENU.has_orientation());
  EXPECT_TRUE(lastImuMsgNED.has_orientation());
  EXPECT_TRUE(lastImuMsgNWU.has_orientation());
  EXPECT_TRUE(lastImuMsgCUSTOM.has_orientation());
  EXPECT_TRUE(lastImuMsgDEFAULT.has_orientation());

  // For the DEFAULT msg, orientation is reported relative
  // to the original pose, which should be identity quaternion.
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().z(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgDEFAULT.orientation().w(), 1, 1e-2);

  // For the ENU msg
  EXPECT_NEAR(lastImuMsgENU.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().z(), 0.707, 1e-2);
  EXPECT_NEAR(lastImuMsgENU.orientation().w(), 0.707, 1e-2);

  // For the NED msg
  EXPECT_NEAR(lastImuMsgNED.orientation().x(), -1, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().z(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNED.orientation().w(), 0, 1e-2);

  // For the NWU msg
  EXPECT_NEAR(lastImuMsgNWU.orientation().x(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().y(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().z(), 0, 1e-2);
  EXPECT_NEAR(lastImuMsgNWU.orientation().w(), 1, 1e-2);

  // For the CUSTOM msg
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().x(), -0.5, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().y(), 0.5, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().z(), 0.5, 1e-2);
  EXPECT_NEAR(lastImuMsgCUSTOM.orientation().w(), 0.5, 1e-2);
}

/////////////////////////////////////////////////
// The test checks if orientations are reported correctly for a rotating body.
// The world includes a sphere rolling down a plane, with axis of rotation
// as the "west" direction vector, using the right hand rule.
TEST_F(ImuTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(RotatingBody))
{
  imuMsgs.clear();
  clearLastImuMsgs();

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "imu_rotating_demo.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  msgs::IMU currentImuMsgDefault_1;
  msgs::IMU currentImuMsgDefault_2;
  msgs::IMU currentImuMsgDefault_3;

  std::function<void(const msgs::IMU &_msg)> defaultCb1 =
      [&](const msgs::IMU &_msg)
      {
        currentImuMsgDefault_1 = _msg;
      };
  std::function<void(const msgs::IMU &_msg)> defaultCb2 =
      [&](const msgs::IMU &_msg)
      {
        currentImuMsgDefault_2 = _msg;
      };
  std::function<void(const msgs::IMU &_msg)> defaultCb3 =
      [&](const msgs::IMU &_msg)
      {
        currentImuMsgDefault_3 = _msg;
      };

  // subscribe to imu topic
  transport::Node node;
  node.Subscribe("/imu_test_ENU", &imuENUCb);
  node.Subscribe("/imu_test_NED", &imuNEDCb);
  node.Subscribe("/imu_test_DEFAULT_1", defaultCb1);
  node.Subscribe("/imu_test_DEFAULT_2", defaultCb2);
  node.Subscribe("/imu_test_DEFAULT_3", defaultCb3);

  // Run server
  server.Run(true, 50u, false);

  // Store initial orientations reported by the IMUs
  auto initialOrientationDEFAULT_1 = msgs::Convert(
                  currentImuMsgDefault_1.orientation());
  auto initialOrientationDEFAULT_2 = msgs::Convert(
                  currentImuMsgDefault_2.orientation());
  auto initialOrientationDEFAULT_3 = msgs::Convert(
                  currentImuMsgDefault_3.orientation());
  auto initialOrientationENU = msgs::Convert(
                  lastImuMsgENU.orientation());
  auto initialOrientationNED = msgs::Convert(
                  lastImuMsgNED.orientation());

  server.Run(true, 1500u, false);

  // Store final orientations reported by the IMUs
  auto finalOrientationDEFAULT_1 = msgs::Convert(
                  currentImuMsgDefault_1.orientation());
  auto finalOrientationDEFAULT_2 = msgs::Convert(
                  currentImuMsgDefault_2.orientation());
  auto finalOrientationDEFAULT_3 = msgs::Convert(
                  currentImuMsgDefault_3.orientation());
  auto finalOrientationENU = msgs::Convert(
                  lastImuMsgENU.orientation());
  auto finalOrientationNED = msgs::Convert(
                  lastImuMsgNED.orientation());

  auto differenceOrientationDEFAULT_1 = finalOrientationDEFAULT_1 *
          initialOrientationDEFAULT_1.Inverse();
  auto differenceOrientationDEFAULT_2 = finalOrientationDEFAULT_2 *
          initialOrientationDEFAULT_2.Inverse();
  auto differenceOrientationDEFAULT_3 = finalOrientationDEFAULT_3 *
          initialOrientationDEFAULT_3.Inverse();

  auto differenceOrientationENU = finalOrientationENU *
          initialOrientationENU.Inverse();
  auto differenceOrientationNED = finalOrientationNED *
          initialOrientationNED.Inverse();

  // Since the sphere has rotated along the west direction,
  // pitch and yaw change for ENU reporting IMU should be zero,
  // and roll should have some non trivial negative value.
  EXPECT_TRUE((differenceOrientationENU.Roll() < -0.04));
  EXPECT_NEAR(differenceOrientationENU.Pitch(), 0, 1e-2);
  EXPECT_NEAR(differenceOrientationENU.Yaw(), 0, 1e-2);

  // Similarly, roll and yaw for NED reporting IMU should be zero,
  // and pitch should be some non trivial negative value.
  EXPECT_NEAR(differenceOrientationNED.Roll(), 0, 1e-2);
  EXPECT_TRUE((differenceOrientationNED.Pitch() < -0.04));
  EXPECT_NEAR(differenceOrientationNED.Yaw(), 0, 1e-2);

  // In the sdf world, the IMU model & link have a yaw pose of PI/2,
  // which means the initial orientation of DEFAULT IMU is
  // effectively WND (by rotating ENU by PI about N). Therefore,
  // pitch and yaw for DEFAULT case should be zero, and roll
  // should be nontrivial positive value.
  EXPECT_TRUE((differenceOrientationDEFAULT_1.Roll() > 0.04));
  EXPECT_NEAR(differenceOrientationDEFAULT_1.Pitch(), 0, 1e-2);
  EXPECT_NEAR(differenceOrientationDEFAULT_1.Yaw(), 0, 1e-2);

  // For DEFAULT_2, model has a pose PI/2 0 0 & link has a pose of
  // 0 PI/2 0, which makes the frame NUE.
  EXPECT_NEAR(differenceOrientationDEFAULT_2.Roll(), 0, 1e-2);
  EXPECT_NEAR(differenceOrientationDEFAULT_2.Pitch(), 0, 1e-2);
  EXPECT_TRUE((differenceOrientationDEFAULT_2.Yaw() < -0.04));

  // For DEFAULT_3, model has a pose PI/2 0 0 & link has a pose of
  // 0 0 PI/2, which makes the frame UWS.
  EXPECT_NEAR(differenceOrientationDEFAULT_3.Roll(), 0, 1e-2);
  EXPECT_TRUE((differenceOrientationDEFAULT_3.Pitch() > 0.04));
  EXPECT_NEAR(differenceOrientationDEFAULT_3.Yaw(), 0, 1e-2);

  // Those nontrivial values should match for all sensors.
  EXPECT_NEAR(differenceOrientationENU.Roll(),
        differenceOrientationNED.Pitch(), 1e-4);

  EXPECT_NEAR(differenceOrientationENU.Roll(),
        -differenceOrientationDEFAULT_1.Roll(), 1e-4);
  EXPECT_NEAR(differenceOrientationENU.Roll(),
        differenceOrientationDEFAULT_2.Yaw(), 1e-4);
  EXPECT_NEAR(differenceOrientationENU.Roll(),
        -differenceOrientationDEFAULT_3.Pitch(), 1e-4);
}
