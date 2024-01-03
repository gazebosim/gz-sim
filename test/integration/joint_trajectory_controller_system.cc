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

#include <gz/msgs/float.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>

#include <array>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for JointTrajectoryController system
class JointTrajectoryControllerTestFixture
  : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Tests that JointTrajectoryController accepts position-controlled joint
// trajectory
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointTrajectoryControllerTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(JointTrajectoryControllerPositionControl))
{
  using namespace std::chrono_literals;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
                       "/test/worlds/joint_trajectory_controller.sdf";

  // Define joints of the model to investigate
  const size_t kNumberOfJoints = 2;
  const std::string jointNames[kNumberOfJoints] = {"RR_position_control_joint1",
                                                  "RR_position_control_joint2"};

  // Define names of Gazebo Transport topics
  const std::string trajectoryTopic =
      "/model/RR_position_control/joint_trajectory";
  const std::string feedbackTopic =
      "/model/RR_position_control/joint_trajectory_feedback";

  // Define initial joint positions
  const std::array<double, kNumberOfJoints> initialPositions = {0.0, 0.0};

  // Define a trajectory to follow
  msgs::Duration timeFromStart;
  std::vector<msgs::Duration> trajectoryTimes;
  std::vector<std::array<double, kNumberOfJoints>> trajectoryPositions;
  // Point1
  timeFromStart.set_sec(0);
  timeFromStart.set_nsec(666000000);
  trajectoryTimes.push_back(timeFromStart);
  trajectoryPositions.push_back({-0.7854, 0.7854});
  // Point2
  timeFromStart.set_sec(1);
  timeFromStart.set_nsec(333000000);
  trajectoryTimes.push_back(timeFromStart);
  trajectoryPositions.push_back({1.5708, -0.7854});
  // Point3
  timeFromStart.set_sec(2);
  timeFromStart.set_nsec(0);
  trajectoryTimes.push_back(timeFromStart);
  trajectoryPositions.push_back({3.1416, -1.5708});

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // Setup test system
  test::Relay testSystem;
  double currentPositions[kNumberOfJoints];
  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        // Create a JointPosition component for each joint if it doesn't exist
        for (const auto &jointName : jointNames)
        {
          const auto joint = _ecm.EntityByComponents(components::Joint(),
                                                     components::Name(
                                                         jointName));
          if (nullptr == _ecm.Component<components::JointPosition>(joint))
          {
            _ecm.CreateComponent(joint, components::JointPosition());
          }
        }
      });
  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        // Get the current position of each joint
        for (std::size_t i = 0; i < kNumberOfJoints; ++i)
        {
          const auto joint = _ecm.EntityByComponents(components::Joint(),
                                                     components::Name(
                                                         jointNames[i]));
          const auto jointPositionComponent =
              _ecm.Component<components::JointPosition>(joint);
          if (nullptr != jointPositionComponent)
          {
            currentPositions[i] = jointPositionComponent->Data()[0];
          }
        }
      });
  server.AddSystem(testSystem.systemPtr);

  // Step few iterations and assert that the initial position is kept
  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  for (size_t i = 0; i < kNumberOfJoints; ++i)
  {
    EXPECT_NEAR(currentPositions[i], initialPositions[i], TOL);
  }

  // Create new JointTrajectory message based on the defined trajectory
  gz::msgs::JointTrajectory msg;
  for (const auto &jointName : jointNames)
  {
    msg.add_joint_names(jointName);
  }
  for (size_t i = 0; i < trajectoryPositions.size(); ++i)
  {
    gz::msgs::JointTrajectoryPoint point;

    // Set the temporal information for the point
    auto time = point.mutable_time_from_start();
    time->set_sec(trajectoryTimes[i].sec());
    time->set_nsec(trajectoryTimes[i].nsec());

    // Add target positions to the point
    for (size_t j = 0; j < kNumberOfJoints; ++j)
    {
      point.add_positions(trajectoryPositions[i][j]);
    }

    // Add point to the trajectory
    msg.add_points();
    msg.mutable_points(i)->CopyFrom(point);
  }

  // Verify that feedback is strictly increasing and reaches value of 1.0
  size_t count = 0;
  std::function<void(const gz::msgs::Float &)> feedbackCallback =
      [&](const gz::msgs::Float &_msg) {
        count++;
        if (trajectoryPositions.size() == count)
        {
          EXPECT_FLOAT_EQ(_msg.data(), 1.0f);
        }
        else
        {
          EXPECT_FLOAT_EQ(_msg.data(), static_cast<float>(count) /
                              static_cast<float>(trajectoryPositions.size()));
        }
      };
  transport::Node node;
  node.Subscribe(feedbackTopic, feedbackCallback);

  // Publish joint trajectory
  auto pub = node.Advertise<msgs::JointTrajectory>(trajectoryTopic);
  pub.Publish(msg);

  // Wait for message to be published
  std::this_thread::sleep_for(100ms);

  // Run the simulation while asserting the target position of all joints at
  // each trajectory point
  auto previousIterFromStart = 0;
  for (size_t i = 0; i < trajectoryPositions.size(); ++i)
  {
    // Number of iters required to reach time_from_start of the current
    // point (1ms step size)
    auto iterFromStart = trajectoryTimes[i].sec() * 1000 +
                         trajectoryTimes[i].nsec() / 1000000;
    auto neededIters = iterFromStart - previousIterFromStart;

    // Run the simulation
    server.Run(true, neededIters, false);

    // Assert that each joint reached its target position
    for (size_t j = 0; j < kNumberOfJoints; ++j)
    {
      EXPECT_NEAR(currentPositions[j], trajectoryPositions[i][j], TOL);
    }

    // Keep track of how many iterations have already passed
    previousIterFromStart = iterFromStart;
  }
}

/////////////////////////////////////////////////
// Tests that JointTrajectoryController accepts velocity-controlled joint
// trajectory
TEST_F(JointTrajectoryControllerTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(JointTrajectoryControllerVelocityControl))
{
  using namespace std::chrono_literals;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
                       "/test/worlds/joint_trajectory_controller.sdf";

  // Define joints of the model to investigate
  const size_t kNumberOfJoints = 2;
  const std::string jointNames[kNumberOfJoints] = {"RR_velocity_control_joint1",
                                                  "RR_velocity_control_joint2"};

  // Define names of Gazebo Transport topics
  const std::string trajectoryTopic = "/test_custom_topic/velocity_control";
  const std::string feedbackTopic =
      "/test_custom_topic/velocity_control_feedback";

  // Define initial joint velocities
  const std::array<double, kNumberOfJoints> initialVelocities = {0.0, 0.0};

  // Define a trajectory to follow
  msgs::Duration timeFromStart;
  std::vector<msgs::Duration> trajectoryTimes;
  std::vector<std::array<double, kNumberOfJoints>> trajectoryVelocities;
  // Point1
  timeFromStart.set_sec(0);
  timeFromStart.set_nsec(500000000);
  trajectoryTimes.push_back(timeFromStart);
  trajectoryVelocities.push_back({0.5, 0.5});
  // Point2
  timeFromStart.set_sec(1);
  timeFromStart.set_nsec(0);
  trajectoryTimes.push_back(timeFromStart);
  trajectoryVelocities.push_back({-1.0, 1.0});

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // Setup test system
  test::Relay testSystem;
  double currentVelocities[kNumberOfJoints];
  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        // Create a JointVelocity component for each joint if it doesn't exist
        for (const auto &jointName : jointNames)
        {
          const auto joint = _ecm.EntityByComponents(components::Joint(),
                                                     components::Name(
                                                         jointName));
          if (nullptr == _ecm.Component<components::JointVelocity>(joint))
          {
            _ecm.CreateComponent(joint, components::JointVelocity());
          }
        }
      });
  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        // Get the current velocity of each joint
        for (std::size_t i = 0; i < kNumberOfJoints; ++i)
        {
          const auto joint = _ecm.EntityByComponents(components::Joint(),
                                                     components::Name(
                                                         jointNames[i]));
          const auto jointVelocityComponent =
              _ecm.Component<components::JointVelocity>(joint);
          if (nullptr != jointVelocityComponent)
          {
            currentVelocities[i] = jointVelocityComponent->Data()[0];
          }
        }
      });
  server.AddSystem(testSystem.systemPtr);

  // Step few iterations and assert that the initial velocity is kept
  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  for (size_t i = 0; i < kNumberOfJoints; ++i)
  {
    EXPECT_NEAR(currentVelocities[i], initialVelocities[i], TOL);
  }

  // Create new JointTrajectory message based on the defined trajectory
  gz::msgs::JointTrajectory msg;
  for (const auto &jointName : jointNames)
  {
    msg.add_joint_names(jointName);
  }
  for (size_t i = 0; i < trajectoryVelocities.size(); ++i)
  {
    gz::msgs::JointTrajectoryPoint point;

    // Set the temporal information for the point
    auto time = point.mutable_time_from_start();
    time->set_sec(trajectoryTimes[i].sec());
    time->set_nsec(trajectoryTimes[i].nsec());

    // Add target velocities to the point
    for (size_t j = 0; j < kNumberOfJoints; ++j)
    {
      point.add_velocities(trajectoryVelocities[i][j]);
    }

    // Add point to the trajectory
    msg.add_points();
    msg.mutable_points(i)->CopyFrom(point);
  }

  // Verify that feedback is strictly increasing and reaches value of 1.0
  size_t count = 0;
  std::function<void(const gz::msgs::Float &)> feedbackCallback =
      [&](const gz::msgs::Float &_msg) {
        count++;
        if (trajectoryVelocities.size() == count)
        {
          EXPECT_FLOAT_EQ(_msg.data(), 1.0f);
        }
        else
        {
          EXPECT_FLOAT_EQ(_msg.data(), static_cast<float>(count) /
                              static_cast<float>(trajectoryVelocities.size()));
        }
      };
  transport::Node node;
  node.Subscribe(feedbackTopic, feedbackCallback);

  // Publish joint trajectory
  auto pub = node.Advertise<msgs::JointTrajectory>(trajectoryTopic);
  pub.Publish(msg);

  // Wait for message to be published
  std::this_thread::sleep_for(100ms);

  // Run the simulation while asserting the target velocity of all joints at
  // each trajectory point
  auto previousIterFromStart = 0;
  for (size_t i = 0; i < trajectoryVelocities.size(); ++i)
  {
    // Number of iters required to reach time_from_start of the current
    // point (1ms step size)
    auto iterFromStart = trajectoryTimes[i].sec() * 1000 +
                         trajectoryTimes[i].nsec() / 1000000;
    auto neededIters = iterFromStart - previousIterFromStart;

    // Run the simulation
    server.Run(true, neededIters, false);

    // Assert that each joint reached its target velocity
    for (size_t j = 0; j < kNumberOfJoints; ++j)
    {
      EXPECT_NEAR(currentVelocities[j], trajectoryVelocities[i][j], TOL);
    }

    // Keep track of how many iterations have already passed
    previousIterFromStart = iterFromStart;
  }
}
