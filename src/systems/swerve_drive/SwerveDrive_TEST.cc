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
 */

#include "SwerveDrive.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for SwerveDrive.
/// \param[in] _sdfString SDF text describing the model and plugin.
/// \param[in] _expectedTopicNames Expected resolved topic names.
void TestTopicNames(const std::string &_sdfString,
    const SwerveDrive::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  SwerveDrive plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();
  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
}

TEST(SwerveDriveTest, AbsoluteTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='swerve_drive_test_world'>
        <model name='swerve_drive' namespace='alpha'>
          <plugin name='gz::sim::systems::SwerveDrive'
                  filename='gz-sim-swerve-drive-system'>
            <topic>/shared/cmd_vel</topic>
            <odom_topic>/shared/odometry</odom_topic>
            <tf_topic>/shared/tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SwerveDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/shared/cmd_vel";
  expectedTopicNames.odomTopic = "/shared/odometry";
  expectedTopicNames.tfTopic = "/shared/tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SwerveDriveTest, RelativeTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='swerve_drive_test_world'>
        <model name='swerve_drive' namespace='alpha'>
          <plugin name='gz::sim::systems::SwerveDrive'
                  filename='gz-sim-swerve-drive-system'>
            <topic>drive_cmd</topic>
            <odom_topic>state/odometry</odom_topic>
            <tf_topic>state/tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SwerveDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "alpha/drive_cmd";
  expectedTopicNames.odomTopic = "alpha/state/odometry";
  expectedTopicNames.tfTopic = "alpha/state/tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SwerveDriveTest, DefaultTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='swerve_drive_test_world'>
        <model name='swerve_drive' namespace='alpha'>
          <plugin name='gz::sim::systems::SwerveDrive'
                  filename='gz-sim-swerve-drive-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SwerveDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "alpha/cmd_vel";
  expectedTopicNames.odomTopic = "alpha/odometry";
  expectedTopicNames.tfTopic = "alpha/tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SwerveDriveTest, TopicNamesWithoutNamespace)
{
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='swerve_drive_test_world'>
        <model name='swerve_drive'>
          <plugin name='gz::sim::systems::SwerveDrive'
                  filename='gz-sim-swerve-drive-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SwerveDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/model/swerve_drive/cmd_vel";
  expectedTopicNames.odomTopic = "/model/swerve_drive/odometry";
  expectedTopicNames.tfTopic = "/model/swerve_drive/tf";
  TestTopicNames(sdfString, expectedTopicNames);

  sdfString = R"(
    <sdf version='1.10'>
      <world name='swerve_drive_test_world'>
        <model name='swerve_drive'>
          <plugin name='gz::sim::systems::SwerveDrive'
                  filename='gz-sim-swerve-drive-system'>
            <topic>drive_cmd</topic>
            <odom_topic>state/odometry</odom_topic>
            <tf_topic>state/tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdVelTopic = "drive_cmd";
  expectedTopicNames.odomTopic = "state/odometry";
  expectedTopicNames.tfTopic = "state/tf";
  TestTopicNames(sdfString, expectedTopicNames);
}
