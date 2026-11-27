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

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Image.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"
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

  // The twist message is delivered asynchronously, so DiffDrive may only
  // receive it a few iterations into the run. Skip the poses from before the
  // vehicle started moving, but make sure it starts within the first 0.5 s.
  constexpr std::size_t kMaxStartIterations = 500u;
  constexpr double kStartTolerance = 1e-6;
  std::size_t firstMovingIndex = 1u;
  while (firstMovingIndex < poses.size() &&
         poses[firstMovingIndex].Pos().X() <
         poses[0].Pos().X() + kStartTolerance)
  {
    ++firstMovingIndex;
  }
  ASSERT_LT(firstMovingIndex, kMaxStartIterations);

  // Look through the vehicle poses to verify that the vehicle did slip
  constexpr double kVehiclePoseInSlipRegion = -4.76;
  for (std::size_t index = firstMovingIndex; index < poses.size(); ++index)
  {
    const auto &pose = poses[index];

    // Verify that the vehicle moved forward in +x direction
    math::Vector3d dir = pose.Pos() - poses[index - 1].Pos();
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

/////////////////////////////////////////////////
TEST_F(LookupWheelSlipTest, NonSquareMap)
{
  common::TempDirectory tempDir("gz-sim-slip-map-", "test", true);
  ASSERT_TRUE(tempDir.Valid());
  const auto mapFile = common::joinPaths(tempDir.Path(), "slip_map.png");

  constexpr unsigned int kWidth = 8u;
  constexpr unsigned int kHeight = 4u;
  constexpr unsigned int kChannels = 3u;
  unsigned char rgb[kWidth * kHeight * kChannels];
  // The green channel stores the row-major pixel index plus 128.
  for (unsigned int pixel = 0; pixel < kWidth * kHeight; ++pixel)
  {
    const auto offset = kChannels * pixel;
    rgb[offset] = 128u;
    rgb[offset + 1u] = static_cast<unsigned char>(128u + pixel);
    rgb[offset + 2u] = 128u;
  }

  common::Image image;
  image.SetFromData(rgb, kWidth, kHeight, common::Image::RGB_INT8);
  image.SavePNG(mapFile);
  ASSERT_TRUE(common::exists(mapFile)) << mapFile;

  const std::string sdf = R"(
<sdf version="1.9">
  <world name="rectangular_slip_map">
    <gravity>0 0 0</gravity>
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <model name="fixed_wheel">
      <pose>1 0 5 0 0 0</pose>
      <link name="base">
        <inertial>
          <mass>5</mass>
        </inertial>
      </link>
      <link name="wheel">
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.01</ixx>
            <iyy>0.01</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="wheel_collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
      </link>
      <joint name="fixed" type="fixed">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint name="wheel_joint" type="revolute">
        <parent>base</parent>
        <child>wheel</child>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>wheel_joint</joint_name>
        <initial_velocity>10</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-wheel-slip-system"
              name="gz::sim::systems::WheelSlip">
        <wheel link_name="wheel">
          <slip_compliance_lateral>0.1</slip_compliance_lateral>
          <slip_compliance_longitudinal>0.2</slip_compliance_longitudinal>
          <wheel_normal_force>10</wheel_normal_force>
          <wheel_radius>0.1</wheel_radius>
        </wheel>
      </plugin>
      <plugin filename="gz-sim-lookup-wheel-slip-system"
              name="gz::sim::systems::LookupWheelSlip">
        <slip_map><![CDATA[)" + mapFile + R"(]]></slip_map>
        <size_x>8</size_x>
        <size_y>4</size_y>
        <wheel_link_name>wheel</wheel_link_name>
        <slip_compliance_longitudinal_delta>
          0.01
        </slip_compliance_longitudinal_delta>
      </plugin>
    </model>
  </world>
</sdf>)";

  constexpr unsigned int iterations = 400u;
  std::vector<double> slipCommand;
  double spinSpeed = 0.0;

  ServerConfig serverConfig;
  serverConfig.SetSdfString(sdf);
  Server server(serverConfig);

  // Read the slip command and the joint speed before physics steps.
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &_info,
    EntityComponentManager &_ecm)
    {
      if (_info.iterations != iterations)
        return;

      const auto collision = _ecm.EntityByComponents(
          components::Collision(), components::Name("wheel_collision"));
      const auto *command =
          _ecm.Component<components::SlipComplianceCmd>(collision);
      if (command)
        slipCommand = command->Data();

      const auto joint = _ecm.EntityByComponents(
          components::Joint(), components::Name("wheel_joint"));
      const auto *velocity = _ecm.Component<components::JointVelocity>(joint);
      if (velocity && !velocity->Data().empty())
        spinSpeed = std::abs(velocity->Data()[0]);
    });
  server.AddSystem(testSystem.systemPtr);
  ASSERT_TRUE(server.Run(true, iterations, false));

  ASSERT_EQ(2u, slipCommand.size()) << "Missing wheel slip command";
  ASSERT_GT(spinSpeed, 0.0);
  constexpr double wheelRadius = 0.1;
  constexpr double wheelNormalForce = 10.0;
  const double compliance =
      slipCommand[1] * wheelNormalForce / (wheelRadius * spinSpeed);

  // The wheel at (1, 0) samples (5, 2), with green offset 2 * 8 + 5.
  EXPECT_NEAR(0.41, compliance, 1e-3);
}
