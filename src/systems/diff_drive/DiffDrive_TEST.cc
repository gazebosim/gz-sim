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

#include "DiffDrive.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for DiffDrive system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const DiffDrive::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, &pluginSdf);

  auto plugin = new DiffDrive();
  plugin->Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin->ResolvedTopicNames();

  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.enableTopic, _expectedTopicNames.enableTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
}

TEST(DiffDriveTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::DiffDrive'
                  filename='gz-sim-diff-drive-system'>
            <topic>/test_cmd_vel</topic>
            <odom_topic>/test_odom</odom_topic>
            <tf_topic>/test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DiffDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.enableTopic = "ns/enable";
  expectedTopicNames.odomTopic = "/test_odom";
  expectedTopicNames.tfTopic = "/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DiffDriveTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::DiffDrive'
                  filename='gz-sim-diff-drive-system'>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DiffDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.enableTopic = "ns/enable";
  expectedTopicNames.odomTopic = "ns/test_odom";
  expectedTopicNames.tfTopic = "ns/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DiffDriveTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::DiffDrive'
                  filename='gz-sim-diff-drive-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DiffDrive::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/cmd_vel";
  expectedTopicNames.enableTopic = "ns/enable";
  expectedTopicNames.odomTopic = "ns/odometry";
  expectedTopicNames.tfTopic = "ns/tf";
  TestTopicNames(sdfString, expectedTopicNames);
}
