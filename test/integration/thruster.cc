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

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>
#include <gz/utilities/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"

#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class ThrusterTest : public InternalFixture<::testing::Test>
{
  /// \brief Test a world file
  /// \param[in] _world Path to world file
  /// \param[in] _namespace Namespace for topic
  /// \param[in] _topic Thrust topic
  /// \param[in] _coefficient Thrust coefficient
  /// \param[in] _density Fluid density
  /// \param[in] _diameter Propeller diameter
  /// \param[in] _baseTol Base tolerance for most quantities
  public: void TestWorld(const std::string &_world,
      const std::string &_namespace, const std::string &_topic,
      double _thrustCoefficient, double _density, double _diameter,
      double _baseTol, double _wakeFraction = 0.2, double _alpha_1 = 1,
      double _alpha_2 = 0, bool _calculateCoefficient = false);
};

//////////////////////////////////////////////////
void ThrusterTest::TestWorld(const std::string &_world,
    const std::string &_namespace, const std::string &_topic,
    double _thrustCoefficient, double _density, double _diameter,
    double _baseTol, double _wakeFraction, double _alpha1, double _alpha2,
    bool _calculateCoefficient)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link propeller;
  std::vector<math::Pose3d> modelPoses;
  std::vector<math::Vector3d> propellerAngVels;
  std::vector<math::Vector3d> propellerLinVels;
  ignition::math::Pose3d jointPose, linkWorldPose;
  ignition::math::Vector3d jointAxis;
  double dt{0.0};
  fixture.
  OnConfigure(
    [&](const ignition::gazebo::Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "sub");
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto jointEntity = model.JointByName(_ecm, "propeller_joint");
      jointAxis =
        _ecm.Component<ignition::gazebo::components::JointAxis>(
        jointEntity)->Data().Xyz();
      jointPose = _ecm.Component<components::Pose>(
          jointEntity)->Data();

      auto propellerEntity = model.LinkByName(_ecm, "propeller");
      EXPECT_NE(propellerEntity, kNullEntity);

      propeller = Link(propellerEntity);
      propeller.EnableVelocityChecks(_ecm);

    }).
  OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                            const gazebo::EntityComponentManager &_ecm)
    {
      dt = std::chrono::duration<double>(_info.dt).count();

      auto modelPose = worldPose(model.Entity(), _ecm);
      modelPoses.push_back(modelPose);

      auto propellerAngVel = propeller.WorldAngularVelocity(_ecm);
      ASSERT_TRUE(propellerAngVel);
      propellerAngVels.push_back(propellerAngVel.value());

      auto proellerLinVel = propeller.WorldLinearVelocity(_ecm);
      ASSERT_TRUE(proellerLinVel);
      propellerLinVels.push_back(proellerLinVel.value());
    }).
  Finalize();

  // Check initial position
  fixture.Server()->Run(true, 100, false);
  EXPECT_EQ(100u, modelPoses.size());
  EXPECT_EQ(100u, propellerAngVels.size());
  EXPECT_EQ(100u, propellerLinVels.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(propeller.Entity(), kNullEntity);

  for (const auto &pose : modelPoses)
  {
    EXPECT_EQ(math::Pose3d(), pose);
  }
  modelPoses.clear();
  for (const auto &vel : propellerAngVels)
  {
    EXPECT_EQ(math::Vector3d::Zero, vel);
  }
  propellerAngVels.clear();
  propellerLinVels.clear();

  // Publish command and check that vehicle moved
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(_topic);

  int sleep{0};
  int maxSleep{30};
  for (; !pub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_TRUE(pub.HasConnections());

  // input force cmd - this should be capped to 0
  double forceCmd{-1000.0};
  msgs::Double msg;
  msg.set_data(forceCmd);
  pub.Publish(msg);

  // Check no movement
  fixture.Server()->Run(true, 100, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(0.0, modelPoses.back().Pos().X());
  EXPECT_EQ(100u, modelPoses.size());
  EXPECT_EQ(100u, propellerAngVels.size());
  EXPECT_EQ(100u, propellerLinVels.size());
  modelPoses.clear();
  propellerAngVels.clear();
  propellerLinVels.clear();

  // input force cmd this should be capped to 300
  forceCmd = 1000.0;
  msg.set_data(forceCmd);
  pub.Publish(msg);

  // Check movement
  if (_namespace != "lowbattery")
  {
    for (sleep = 0; modelPoses.back().Pos().X() < 5.0 && sleep < maxSleep;
        ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      fixture.Server()->Run(true, 100, false);
    }
    EXPECT_LT(sleep, maxSleep);
    EXPECT_LT(5.0, modelPoses.back().Pos().X());

    EXPECT_EQ(100u * sleep, modelPoses.size());
    EXPECT_EQ(100u * sleep, propellerAngVels.size());
    EXPECT_EQ(100u * sleep, propellerLinVels.size());
  }

  // max allowed force
  double force{300.0};

  // F = m * a
  // s = a * t^2 / 2
  // F = m * 2 * s / t^2
  // s = F * t^2 / 2m
  double mass{100.1};
  double xTol{1e-2};
  for (unsigned int i = 0; i < modelPoses.size(); ++i)
  {
    if (_namespace == "lowbattery" && i > 545)
    {
      // Battery discharged should not accelerate
      EXPECT_NEAR(modelPoses[i-1].Pos().X() - modelPoses[i-2].Pos().X(),
        modelPoses[i].Pos().X() - modelPoses[i-1].Pos().X(), 1e-6);
      continue;
    }

    auto pose = modelPoses[i];
    auto time = dt * i;
    EXPECT_NEAR(force * time * time / (2 * mass), pose.Pos().X(), xTol);
    EXPECT_NEAR(0.0, pose.Pos().Y(), _baseTol);
    EXPECT_NEAR(0.0, pose.Pos().Z(), _baseTol);
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), _baseTol);
    EXPECT_NEAR(0.0, pose.Rot().Yaw(), _baseTol);

    // The joint velocity command adds some roll to the body which the PID
    // wrench doesn't
    if (_namespace == "custom" || _namespace == "lowbattery")
      EXPECT_NEAR(0.0, pose.Rot().Roll(), 0.1);
    else
      EXPECT_NEAR(0.0, pose.Rot().Roll(), _baseTol);
  }

  auto jointWorldPose = linkWorldPose * jointPose;
  auto unitVector = jointWorldPose.Rot().RotateVector(jointAxis).Normalize();

  double omegaTol{1e-1};
  for (unsigned int i = 0; i < propellerAngVels.size(); ++i)
  {
    auto angularVelocity = propellerAngVels[i].Dot(unitVector);
    // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246:
    // thrust_coefficient = alpha_1 + alpha_2 * (((1-wake_fraction) *
    //    linear_velocity) / (angular_velocity * propeller_diameter))
    // omega = sqrt(thrust /
    //     (fluid_density * thrust_coefficient * propeller_diameter ^ 4))
    if (_calculateCoefficient && !gz::math::equal(angularVelocity, 0.0))
    {
      _thrustCoefficient = _alpha1 + _alpha2 * (((1 - _wakeFraction) *
          propellerLinVels[i].Length()) / (angularVelocity * _diameter));
    }
    auto omega = sqrt(abs(force / (_density * _thrustCoefficient *
        pow(_diameter, 4))));

    auto angVel = propellerAngVels[i];
    // It takes a few iterations to reach the speed
    if (i > 25)
    {
      if (_namespace == "lowbattery" && i > 545)
      {
        EXPECT_NEAR(0.0, angVel.X(), _baseTol);
      }
      else
      {
        EXPECT_NEAR(omega, angVel.X(), omegaTol) << i;
      }
    }
    EXPECT_NEAR(0.0, angVel.Y(), _baseTol);
    EXPECT_NEAR(0.0, angVel.Z(), _baseTol);
  }
}

/////////////////////////////////////////////////
// See https://github.com/ignitionrobotics/ign-gazebo/issues/1175
TEST_F(ThrusterTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(PIDControl))
{
  const std::string ns{"sub"};
  const std::string topic = "/model/" + ns +
      "/joint/propeller_joint/cmd_thrust";
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thruster_pid.sdf");

  // Tolerance could be lower (1e-6) if the joint pose had a precise 180
  // rotation
  this->TestWorld(world, ns, topic, 0.004, 1000, 0.2, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(ThrusterTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(VelocityControl))
{
  const std::string ns = "custom";
  const std::string topic = "/model/" + ns +
      "/joint/propeller_joint/cmd_thrust";
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thruster_vel_cmd.sdf");

  // Tolerance is high because the joint command disturbs the vehicle body
  this->TestWorld(world, ns, topic, 0.005, 950, 0.25, 1e-2);
}

/////////////////////////////////////////////////
TEST_F(ThrusterTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(BatteryIntegration))
{
  const std::string ns = "lowbattery";
  const std::string topic =  ns + "/thrust";
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thruster_battery.sdf");

  // Tolerance is high because the joint command disturbs the vehicle body
  this->TestWorld(world, ns, topic, 0.005, 950, 0.25, 1e-2);
}

/////////////////////////////////////////////////
TEST_F(ThrusterTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(ThrustCoefficient))
{
  const std::string ns = "custom";
  const std::string topic = "/model/" + ns +
      "/joint/propeller_joint/cmd_thrust";
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thrust_coefficient.sdf");

  // Tolerance is high because the joint command disturbs the vehicle body
  this->TestWorld(world, ns, topic, 1, 950, 0.25, 1e-2, 0.2, 0.9, 0.01, true);
}
