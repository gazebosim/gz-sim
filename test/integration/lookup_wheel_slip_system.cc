/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test LookupWheelSlip system
class LookupWheelSlipTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(LookupWheelSlipTest, DriveForwardSlip)
{
  // Verify lookup wheelslip system is able to dynamically update the
  // vehicle's wheel slip compliance parameters as the vehicle traverses
  // over regions of increased slip values.
  // The test sends a command to move the vehicle forward and the vehicle's
  // right wheel should come into contact with the slip region after
  // some distance, at which point it should start slipping which causes
  // the vehicle to rotate and move to the right.

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "examples", "worlds", "lookup_wheel_slip.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the vehicle poses
  test::Relay testSystem;

  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&poses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      auto id = _ecm.EntityByComponents(
        components::Model(),
        components::Name("vehicle_blue"));
      EXPECT_NE(kNullEntity, id);

      auto poseComp = _ecm.Component<components::Pose>(id);
      ASSERT_NE(nullptr, poseComp);

      poses.push_back(poseComp->Data());
    });
  server.AddSystem(testSystem.systemPtr);

  // Run server and check that vehicle didn't move
  server.Run(true, 1000, false);
  EXPECT_EQ(1000u, poses.size());
  for (const auto &pose : poses)
  {
    EXPECT_EQ(poses[0], pose);
  }
  poses.clear();

  // Send a twist message to move the vehicle forward
  const std::string topic = "/model/vehicle_blue/cmd_vel";
  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>(topic);

  msgs::Twist msg;
  double desiredLinVel = 1.0;
  double desiredAngVel = 0.0;
  msgs::Set(msg.mutable_linear(),
            math::Vector3d(desiredLinVel, 0, 0));
  msgs::Set(msg.mutable_angular(),
            math::Vector3d(0.0, 0, desiredAngVel));
  pub.Publish(msg);
  server.Run(true, 3000, false);
  ASSERT_EQ(3000u, poses.size());

  // Look through the vehicle poses to verify that the vehicle did slip
  constexpr double kVehiclePoseInSlipRegion = -4.76;
  math::Pose3d prevPose;
  for (const auto &pose : poses)
  {
    if (prevPose == math::Pose3d::Zero)
    {
      prevPose = pose;
      continue;
    }

    // Verify that the vehicle moved forward in +x direction
    math::Vector3d dir = pose.Pos() - prevPose.Pos();
    prevPose = pose;
    EXPECT_LT(0, dir.X()) << pose;
    EXPECT_NEAR(0, dir.Z(), 1e-4);

    // Vehicle should have no roll and pitch
    EXPECT_NEAR(0.0, pose.Rot().Euler().X(), 1e-4);
    EXPECT_NEAR(0.0, pose.Rot().Euler().Y(), 1e-4);

    if (pose.Pos().X() > kVehiclePoseInSlipRegion)
    {
      // Vehicle should have rotate to the right as its right wheel entered
      // the slip region
      EXPECT_GE(0.0, pose.Rot().Euler().Z()) << pose;
    }
    else
    {
      // Vehicle should move straight before entering the slip region
      EXPECT_NEAR(0.0, pose.Rot().Euler().Z(), 1e-4) << pose;
    }
  }
}
