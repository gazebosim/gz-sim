/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "DriveToPoseController.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for DriveToPoseController system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
                    const DriveToPoseController::TopicNames
                      &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  DriveToPoseController plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.cmdPoseTopic, _expectedTopicNames.cmdPoseTopic);
  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.reachedPoseTopic, _expectedTopicNames.reachedPoseTopic);
}

TEST(DriveToPoseControllerTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='drive_to_pose_controller_test_world'>
        <model name='drive_to_pose_controller' namespace='ns'>
          <plugin name='gz::sim::systems::DriveToPoseController'
                  filename='gz-sim-drive-to-pose-controller-system'>
            <odom_topic>/test_pose</odom_topic>
            <cmd_pose_topic>/test_cmd_pose</cmd_pose_topic>
            <cmd_vel_topic>/test_cmd_vel</cmd_vel_topic>
            <reached_pose_topic>/test_reached_pose</reached_pose_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DriveToPoseController::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "/test_pose";
  expectedTopicNames.cmdPoseTopic = "/test_cmd_pose";
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.reachedPoseTopic = "/test_reached_pose";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DriveToPoseControllerTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='drive_to_pose_controller_test_world'>
        <model name='drive_to_pose_controller' namespace='ns'>
          <plugin name='gz::sim::systems::DriveToPoseController'
                  filename='gz-sim-drive-to-pose-controller-system'>
            <odom_topic>test_pose</odom_topic>
            <cmd_pose_topic>test_cmd_pose</cmd_pose_topic>
            <cmd_vel_topic>test_cmd_vel</cmd_vel_topic>
            <reached_pose_topic>test_reached_pose</reached_pose_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DriveToPoseController::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "ns/test_pose";
  expectedTopicNames.cmdPoseTopic = "ns/test_cmd_pose";
  expectedTopicNames.cmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.reachedPoseTopic = "ns/test_reached_pose";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DriveToPoseControllerTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='drive_to_pose_controller_test_world'>
        <model name='drive_to_pose_controller' namespace='ns'>
          <plugin name='gz::sim::systems::DriveToPoseController'
                  filename='gz-sim-drive-to-pose-controller-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DriveToPoseController::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "ns/pose";
  expectedTopicNames.cmdPoseTopic = "ns/cmd_pose";
  expectedTopicNames.cmdVelTopic = "ns/cmd_vel";
  expectedTopicNames.reachedPoseTopic = "ns/reached_pose";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DriveToPoseControllerTest, TopicNamesWithoutNs)
{
  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='drive_to_pose_controller_test_world'>
        <model name='drive_to_pose_controller'>
          <plugin name='gz::sim::systems::DriveToPoseController'
                  filename='gz-sim-drive-to-pose-controller-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DriveToPoseController::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "/model/drive_to_pose_controller/pose";
  expectedTopicNames.cmdPoseTopic = "/model/drive_to_pose_controller/cmd_pose";
  expectedTopicNames.cmdVelTopic = "/model/drive_to_pose_controller/cmd_vel";
  expectedTopicNames.reachedPoseTopic =
    "/model/drive_to_pose_controller/reached_pose";
  TestTopicNames(sdfString, expectedTopicNames);
}
