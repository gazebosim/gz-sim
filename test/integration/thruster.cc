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
};

/////////////////////////////////////////////////
TEST_F(ThrusterTest, UniformWorldMovement)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "thruster.sdf");
  serverConfig.SetSdfFile(sdfFile);

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
      "/model/sub/joint/propeller_joint/cmd_thrust");

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
  double tightTol{1e-6};
  double xTol{1e-2};
  for (unsigned int i = 0; i < modelPoses.size(); ++i)
  {
    auto pose = modelPoses[i];
    auto time = dt * i;
    EXPECT_NEAR(force * time * time / (2 * mass), pose.Pos().X(), xTol);
    EXPECT_NEAR(0.0, pose.Pos().Y(), tightTol);
    EXPECT_NEAR(0.0, pose.Pos().Z(), tightTol);
    EXPECT_NEAR(0.0, pose.Rot().Roll(), tightTol);
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), tightTol);
    EXPECT_NEAR(0.0, pose.Rot().Yaw(), tightTol);
  }

  // omega = sqrt(thrust /
  //     (fluid_density * thrust_coefficient * propeller_diameter ^ 4))
  auto omega = sqrt(force / (1000 * 0.004 * pow(0.2, 4)));
  double omegaTol{1e-1};
  for (unsigned int i = 0; i < propellerAngVels.size(); ++i)
  {
    auto angVel = propellerAngVels[i];
    // It takes a few iterations to reach the speed
    if (i > 25)
    {
      EXPECT_NEAR(omega, angVel.X(), omegaTol) << i;
    }
    EXPECT_NEAR(0.0, angVel.Y(), omegaTol);
    EXPECT_NEAR(0.0, angVel.Z(), omegaTol);
  }
}

