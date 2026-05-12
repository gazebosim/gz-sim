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
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/WrenchMeasured.hh"

#include "gz/sim/Joint.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "helpers/Relay.hh"
#include "helpers/EnvTestFixture.hh"
#include "helpers/Subscription.hh"
#include "helpers/Util.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class ForceTorqueTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
Entity forceTorqueSensorEntity(const EntityComponentManager &_ecm)
{
  const auto test1ModelEntity = _ecm.EntityByComponents(
      components::Model(), components::Name("test1"));
  Model test1Model(test1ModelEntity);
  if (!test1Model.Valid(_ecm))
  {
    return kNullEntity;
  }

  Model scaleModel(test1Model.ModelByName(_ecm, "scale"));
  if (!scaleModel.Valid(_ecm))
  {
    return kNullEntity;
  }

  Joint sensorJoint(scaleModel.JointByName(_ecm, "sensor_joint"));
  if (!sensorJoint.Valid(_ecm))
  {
    return kNullEntity;
  }

  return sensorJoint.SensorByName(_ecm, "force_torque_sensor");
}

/////////////////////////////////////////////////
bool expectedForceTorqueReading(const msgs::Wrench &_wrench)
{
  const double kSensorMass = 0.2;
  const double kWeightMass = 10;
  const double kGravity = 9.8;
  const double kTolerance = 1e-6;
  const auto force = msgs::Convert(_wrench.force());
  const auto torque = msgs::Convert(_wrench.torque());
  const auto expectedForceZ = kGravity * (kSensorMass + kWeightMass);

  return std::abs(force.X()) < kTolerance &&
      std::abs(force.Y()) < kTolerance &&
      std::abs(force.Z() - expectedForceZ) < kTolerance &&
      std::abs(torque.X()) < kTolerance &&
      std::abs(torque.Y()) < kTolerance &&
      std::abs(torque.Z()) < kTolerance;
}

/////////////////////////////////////////////////
void expectSameWrench(const msgs::Wrench &_expected,
    const msgs::Wrench &_actual)
{
  const double kTolerance = 1e-6;
  const auto expectedForce = msgs::Convert(_expected.force());
  const auto actualForce = msgs::Convert(_actual.force());
  const auto expectedTorque = msgs::Convert(_expected.torque());
  const auto actualTorque = msgs::Convert(_actual.torque());

  EXPECT_NEAR(expectedForce.X(), actualForce.X(), kTolerance);
  EXPECT_NEAR(expectedForce.Y(), actualForce.Y(), kTolerance);
  EXPECT_NEAR(expectedForce.Z(), actualForce.Z(), kTolerance);
  EXPECT_NEAR(expectedTorque.X(), actualTorque.X(), kTolerance);
  EXPECT_NEAR(expectedTorque.Y(), actualTorque.Y(), kTolerance);
  EXPECT_NEAR(expectedTorque.Z(), actualTorque.Z(), kTolerance);
}

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
TEST_F(ForceTorqueTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetRestoresEarlyWrench))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "force_torque.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1us);

  msgs::Wrench measuredWrench;
  bool hasMeasuredWrench{false};
  test::Relay testSystem;
  testSystem.OnPreUpdate(
      [](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        const auto sensorEntity = forceTorqueSensorEntity(_ecm);
        if (kNullEntity != sensorEntity &&
            nullptr == _ecm.Component<components::WrenchMeasured>(sensorEntity))
        {
          _ecm.CreateComponent(sensorEntity, components::WrenchMeasured());
        }
      });
  testSystem.OnPostUpdate(
      [&measuredWrench, &hasMeasuredWrench](
          const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        const auto sensorEntity = forceTorqueSensorEntity(_ecm);
        const auto measured =
            _ecm.Component<components::WrenchMeasured>(sensorEntity);
        if (nullptr != measured)
        {
          measuredWrench = measured->Data();
          hasMeasuredWrench = true;
        }
      });
  server.AddSystem(testSystem.systemPtr);

  auto waitForWrench =
      [&server, &measuredWrench, &hasMeasuredWrench]()
      {
        return test::StepUntil(server, 20000, [&]
        {
          return hasMeasuredWrench &&
              expectedForceTorqueReading(measuredWrench);
        });
      };

  ASSERT_TRUE(waitForWrench());
  const auto baseline = measuredWrench;

  server.Run(true, 50000, false);
  server.ResetAll();

  hasMeasuredWrench = false;
  ASSERT_TRUE(waitForWrench());
  const auto postReset = measuredWrench;
  expectSameWrench(baseline, postReset);
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
  // WrenchMeasured component on first PreUpdate
  Entity sensorEntity;
  bool firstRun = true;
  auto addWrenchComponent =
      [&firstRun, &sensorEntity](
          const UpdateInfo &, EntityComponentManager &_ecm)
      {
        if (firstRun)
        {
          firstRun = false;

          sensorEntity = forceTorqueSensorEntity(_ecm);
          ASSERT_NE(kNullEntity, sensorEntity);

          // Expect it doesn't yet have WrenchMeasured
          EXPECT_EQ(nullptr,
                    _ecm.Component<components::WrenchMeasured>(sensorEntity));

          // Add WrenchMeasured
          _ecm.CreateComponent(sensorEntity, components::WrenchMeasured());

          EXPECT_NE(nullptr,
                    _ecm.Component<components::WrenchMeasured>(sensorEntity));
        }
      };

  // Get the MeasuredWrench for //test1/scale/sensor_joint/force_torque_sensor
  msgs::Wrench wrench;
  auto getMeasuredWrench =
    [&wrench, &sensorEntity](const UpdateInfo &,
              const EntityComponentManager &_ecm)
    {
      auto measuredWrench =
          _ecm.Component<components::WrenchMeasured>(sensorEntity);
      ASSERT_NE(nullptr, measuredWrench);
      if (measuredWrench)
      {
        wrench = measuredWrench->Data();
      }
    };

  // Add the system
  test::Relay testSystem;
  testSystem.OnPreUpdate(addWrenchComponent);
  testSystem.OnPostUpdate(getMeasuredWrench);
  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  const size_t iters = 999u;

  // Run server (iters-1) steps, since we already took 1 step above
  server.Run(true, iters - 1, false);
  ASSERT_EQ(iters, *server.IterationCount());

  const double kSensorMass = 0.2;
  const double kWeightMass = 10;
  const double kGravity = 9.8;
  const math::Vector3 expectedForce =
      math::Vector3d{0, 0, kGravity * (kSensorMass + kWeightMass)};
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
}
