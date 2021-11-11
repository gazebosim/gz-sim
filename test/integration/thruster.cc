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

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class ThrusterTest : public InternalFixture<::testing::Test>
{
  /// \brief Test a world file
  /// \param[in] _world Path to world file
  /// \param[in] _namespace Namespace for topic
  /// \param[in] _coefficient Thrust coefficient
  /// \param[in] _density Fluid density
  /// \param[in] _diameter Propeller diameter
  /// \param[in] _baseTol Base tolerance for most quantities
  public: void TestWorld(const std::string &_world,
      const std::string &_namespace, double _coefficient, double _density,
      double _diameter, double _baseTol);
};

//////////////////////////////////////////////////
void ThrusterTest::TestWorld(const std::string &_world,
    const std::string &_namespace, double _coefficient, double _density,
    double _diameter, double _baseTol)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link propeller;
  std::vector<math::Pose3d> modelPoses;
  std::vector<math::Vector3d> propellerAngVels;
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

      auto propellerEntity = model.LinkByName(_ecm, "propeller");
      EXPECT_NE(propellerEntity, kNullEntity);

      propeller = Link(propellerEntity);
      propeller.EnableVelocityChecks(_ecm);
    }).
  OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                            const gazebo::EntityComponentManager &_ecm)
    {
      dt = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.dt).count()
          * 1e-9;

      auto modelPose = worldPose(model.Entity(), _ecm);
      modelPoses.push_back(modelPose);

      auto propellerAngVel = propeller.WorldAngularVelocity(_ecm);
      ASSERT_TRUE(propellerAngVel);
      propellerAngVels.push_back(propellerAngVel.value());
    }).
  Finalize();

  // Check initial position
  fixture.Server()->Run(true, 100, false);
  EXPECT_EQ(100u, modelPoses.size());
  EXPECT_EQ(100u, propellerAngVels.size());

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

  // Publish command and check that vehicle moved
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/" + _namespace + "/joint/propeller_joint/cmd_thrust");

  int sleep{0};
  int maxSleep{30};
  for (; !pub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_TRUE(pub.HasConnections());

  double force{300.0};
  msgs::Double msg;
  msg.set_data(force);
  pub.Publish(msg);

  // Check movement
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

  // F = m * a
  // s = a * t^2 / 2
  // F = m * 2 * s / t^2
  // s = F * t^2 / 2m
  double mass{100.1};
  double xTol{1e-2};
  for (unsigned int i = 0; i < modelPoses.size(); ++i)
  {
    auto pose = modelPoses[i];
    auto time = dt * i;
    EXPECT_NEAR(force * time * time / (2 * mass), pose.Pos().X(), xTol);
    EXPECT_NEAR(0.0, pose.Pos().Y(), _baseTol);
    EXPECT_NEAR(0.0, pose.Pos().Z(), _baseTol);
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), _baseTol);
    EXPECT_NEAR(0.0, pose.Rot().Yaw(), _baseTol);

    // The joint velocity command adds some roll to the body which the PID
    // wrench doesn't
    if (_namespace == "custom")
      EXPECT_NEAR(0.0, pose.Rot().Roll(), 0.1);
    else
      EXPECT_NEAR(0.0, pose.Rot().Roll(), _baseTol);
  }

  // omega = sqrt(thrust /
  //     (fluid_density * thrust_coefficient * propeller_diameter ^ 4))
  auto omega = sqrt(force / (_density * _coefficient * pow(_diameter, 4)));
  double omegaTol{1e-1};
  for (unsigned int i = 0; i < propellerAngVels.size(); ++i)
  {
    auto angVel = propellerAngVels[i];
    // It takes a few iterations to reach the speed
    if (i > 25)
    {
      EXPECT_NEAR(omega, angVel.X(), omegaTol) << i;
    }
    EXPECT_NEAR(0.0, angVel.Y(), _baseTol);
    EXPECT_NEAR(0.0, angVel.Z(), _baseTol);
  }
}

/////////////////////////////////////////////////
TEST_F(ThrusterTest, PIDControl)
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thruster_pid.sdf");
  this->TestWorld(world, "sub", 0.004, 1000, 0.2, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(ThrusterTest, VelocityControl)
{
  // TODO: check why fail
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "thruster_vel_cmd.sdf");
  this->TestWorld(world, "custom", 0.005, 950, 0.25, 1e-2);
}

