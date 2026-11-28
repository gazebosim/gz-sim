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

#include <gz/msgs/wrench.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/WrenchMeasured.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Joint.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "helpers/EnvTestFixture.hh"
#include "helpers/Subscription.hh"

using namespace gz;
using namespace sim;

class ForceTorqueTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ForceTorqueTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(MeasureWeightTopic))
{
  using namespace std::chrono_literals;
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "force_torque.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1us);

  // Having iters exactly in sync with update rate can lead to a race condition
  // in the test between simulation and transport
  const size_t iters = 999u;
  const size_t updates = 100u;

  std::vector<msgs::Wrench> wrenches;
  wrenches.reserve(updates);
  std::mutex wrenchMutex;
  std::condition_variable cv;
  auto wrenchCb = std::function<void(const msgs::Wrench &)>(
      [&wrenchMutex, &wrenches, &cv, updates](const auto &_msg)
      {
        std::lock_guard lock(wrenchMutex);
        wrenches.push_back(_msg);
        if (wrenches.size() >= updates)
        {
          cv.notify_all();
        }
      });

  transport::Node node;
  node.Subscribe("/force_torque1", wrenchCb);

  // Run server
  server.Run(true, iters, false);
  ASSERT_EQ(iters, *server.IterationCount());

  {
    std::unique_lock lock(wrenchMutex);
    cv.wait_for(lock, 30s, [&] { return wrenches.size() >= updates; });
    ASSERT_EQ(updates, wrenches.size());

    const double kSensorMass = 0.2;
    const double kWeightMass = 10;
    const double kGravity = 9.8;
    const auto &wrench = wrenches.back();
    const math::Vector3 expectedForce =
        math::Vector3d{0, 0, kGravity * (kSensorMass + kWeightMass)};
    EXPECT_EQ(expectedForce, msgs::Convert(wrench.force()));
    EXPECT_EQ(math::Vector3d::Zero, msgs::Convert(wrench.torque()));
  }
}

/////////////////////////////////////////////////
TEST_F(ForceTorqueTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(MeasureWeightECM))
{
  using namespace std::chrono_literals;
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "force_torque.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1ns);

  // Get entity for //test1/scale/sensor_joint/force_torque_sensor and add
  // WrenchMeasured component before the first step
  Entity sensorEntity{kNullEntity};
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    auto test1ModelEntity = ecm->EntityByComponents(
        components::Model(), components::Name("test1"));
    Model test1Model(test1ModelEntity);

    Model scaleModel(test1Model.ModelByName(*ecm, "scale"));
    ASSERT_TRUE(scaleModel.Valid(*ecm));

    Joint sensorJoint(scaleModel.JointByName(*ecm, "sensor_joint"));
    ASSERT_TRUE(sensorJoint.Valid(*ecm));

    sensorEntity = sensorJoint.SensorByName(*ecm, "force_torque_sensor");
    ASSERT_NE(kNullEntity, sensorEntity);

    // Expect it doesn't yet have WrenchMeasured
    EXPECT_EQ(nullptr,
              ecm->Component<components::WrenchMeasured>(sensorEntity));

    // Add WrenchMeasured
    ecm->CreateComponent(sensorEntity, components::WrenchMeasured());

    EXPECT_NE(nullptr,
              ecm->Component<components::WrenchMeasured>(sensorEntity));
  }

  const size_t iters = 999u;
  server.Run(true, iters, false);
  ASSERT_EQ(iters, *server.IterationCount());

  const double kSensorMass = 0.2;
  const double kWeightMass = 10;
  const double kGravity = 9.8;
  const math::Vector3 expectedForce =
      math::Vector3d{0, 0, kGravity * (kSensorMass + kWeightMass)};

  // Check the MeasuredWrench for
  // //test1/scale/sensor_joint/force_torque_sensor
  auto ecm = server.EcmScope();
  ASSERT_TRUE(ecm.Valid());
  auto measuredWrench =
      ecm->Component<components::WrenchMeasured>(sensorEntity);
  ASSERT_NE(nullptr, measuredWrench);
  const auto &wrench = measuredWrench->Data();
  EXPECT_EQ(expectedForce, msgs::Convert(wrench.force()));
  EXPECT_EQ(math::Vector3d::Zero, msgs::Convert(wrench.torque()));
}

/////////////////////////////////////////////////
TEST_F(ForceTorqueTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SensorPoseOffset))
{
  using namespace std::chrono_literals;
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "force_torque.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1us);

  // Having iters exactly in sync with update rate can lead to a race condition
  // in the test between simulation and transport
  size_t iters = 999u;
  size_t updates = 100u;

  std::vector<msgs::Wrench> wrenches;
  wrenches.reserve(updates);
  std::mutex wrenchMutex;
  std::condition_variable cv;
  auto wrenchCb = std::function<void(const msgs::Wrench &)>(
      [&wrenchMutex, &wrenches, &cv, updates](const auto &_msg)
      {
        std::lock_guard lock(wrenchMutex);
        wrenches.push_back(_msg);
        if (wrenches.size() >= updates)
        {
          cv.notify_all();
        }
      });

  transport::Node node;
  node.Subscribe("/force_torque2", wrenchCb);

  // Run server
  server.Run(true, iters, false);
  ASSERT_EQ(iters, *server.IterationCount());

  const double kSensorMass = 0.2;
  const double kWeightMass = 10;
  const double kGravity = 9.8;
  {
    std::unique_lock lock(wrenchMutex);
    cv.wait_for(lock, 30s, [&] { return wrenches.size() >= updates; });
    ASSERT_EQ(updates, wrenches.size());

    const double kMomentArm = 0.1;
    const auto &wrench = wrenches.back();
    const math::Vector3 expectedForce =
        math::Vector3d{0, 0, kGravity * (kSensorMass + kWeightMass)};
    EXPECT_EQ(expectedForce, msgs::Convert(wrench.force()));
    EXPECT_NEAR(kMomentArm * expectedForce.Z(), wrench.torque().y(), 1e-3);
    wrenches.clear();
  }

  node.Unsubscribe("/force_torque2");
  node.Subscribe("/force_torque3", wrenchCb);

  server.Run(true, iters, false);
  ASSERT_EQ(2 * iters, *server.IterationCount());
  {
    std::unique_lock lock(wrenchMutex);
    cv.wait_for(lock, 30s, [&] { return wrenches.size() >= updates; });
    ASSERT_EQ(updates, wrenches.size());

    const auto &wrench = wrenches.back();

    const math::Vector3 expectedForce =
        math::Vector3d{0, 0, kGravity * (kSensorMass + kWeightMass)};
    EXPECT_EQ(expectedForce, msgs::Convert(wrench.force()));
    EXPECT_EQ(math::Vector3d::Zero, msgs::Convert(wrench.torque()));
    wrenches.clear();
  }

  node.Unsubscribe("/force_torque3");

  transport::Node rigNode;
  Subscription<msgs::Wrench> originSubscription;
  Subscription<msgs::Wrench> rotatedSubscription;
  originSubscription.Subscribe(rigNode, "/force_torque4_origin", 1u);
  rotatedSubscription.Subscribe(rigNode, "/force_torque4_offset_rotated", 1u);

  // Collect both readings after the load settled during the earlier runs.
  server.Run(true, iters, false);
  ASSERT_EQ(3 * iters, *server.IterationCount());

  ASSERT_TRUE(originSubscription.WaitForMessages(1, 30s));
  ASSERT_TRUE(rotatedSubscription.WaitForMessages(1, 30s));
  const auto originWrench = originSubscription.Last();
  const auto rotatedWrench = rotatedSubscription.Last();

  // The 2 kg load's center of mass is at (0.3, 0.1, 0) in joint axes.
  const math::Vector3d expectedForce{0, 0, 19.6};
  EXPECT_EQ(expectedForce, msgs::Convert(originWrench.force()));
  EXPECT_NEAR(1.96, originWrench.torque().x(), 1e-3);
  EXPECT_NEAR(-5.88, originWrench.torque().y(), 1e-3);
  EXPECT_NEAR(0, originWrench.torque().z(), 1e-3);

  EXPECT_EQ(expectedForce, msgs::Convert(rotatedWrench.force()));
  EXPECT_NEAR(-3.92, rotatedWrench.torque().x(), 1e-3);
  EXPECT_NEAR(-1.96, rotatedWrench.torque().y(), 1e-3);
  EXPECT_NEAR(0, rotatedWrench.torque().z(), 1e-3);
}
