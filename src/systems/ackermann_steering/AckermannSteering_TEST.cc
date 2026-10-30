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

#include "AckermannSteering.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for AckermannSteering system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const AckermannSteering::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  AckermannSteering plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdTopic, _expectedTopicNames.cmdTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
}

TEST(AckermannSteeringTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <topic>/test_cmd_vel</topic>
            <odom_topic>/test_odom</odom_topic>
            <tf_topic>/test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/test_cmd_vel";
  expectedTopicNames.odomTopic = "/test_odom";
  expectedTopicNames.tfTopic = "/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/test_cmd_vel";
  expectedTopicNames.odomTopic = "ns/test_odom";
  expectedTopicNames.tfTopic = "ns/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>false</steering_only>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/cmd_vel";
  expectedTopicNames.odomTopic = "ns/odometry";
  expectedTopicNames.tfTopic = "ns/tf";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "ns/steer_angle";
  expectedTopicNames.odomTopic = "";
  expectedTopicNames.tfTopic = "";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <use_actuator_msg>true</use_actuator_msg>
            <actuator_number>1</actuator_number>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "ns/actuators";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, CustomTopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "test_cmd_vel";
  expectedTopicNames.odomTopic = "test_odom";
  expectedTopicNames.tfTopic = "test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, DefaultTopicNamesWithoutNs)
{
  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>false</steering_only>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/model/diff_drive/cmd_vel";
  expectedTopicNames.odomTopic = "/model/diff_drive/odometry";
  expectedTopicNames.tfTopic = "/model/diff_drive/tf";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "/model/diff_drive/steer_angle";
  expectedTopicNames.odomTopic = "";
  expectedTopicNames.tfTopic = "";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <use_actuator_msg>true</use_actuator_msg>
            <actuator_number>1</actuator_number>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "/actuators";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, SubTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <sub_topic>/sub_topic</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/sub_topic";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='diff_drive_test_world'>
        <model name='diff_drive'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <sub_topic>/sub_topic</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "/model/diff_drive/sub_topic";
  TestTopicNames(sdfString, expectedTopicNames);
}
