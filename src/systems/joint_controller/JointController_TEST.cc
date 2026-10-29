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

#include "JointController.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>


#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for JointController system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const JointController::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  JointController plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdTopic, _expectedTopicNames.cmdTopic);
}

TEST(JointControllerTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller' namespace='ns'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <topic>/test_cmd_vel</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/test_cmd_vel";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointControllerTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller' namespace='ns'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <topic>test_cmd_vel</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/test_cmd_vel";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointControllerTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller' namespace='ns'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/cmd_vel";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller' namespace='ns'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <use_actuator_msg>true</use_actuator_msg>
            <actuator_number>1</actuator_number>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "ns/actuators";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointControllerTest, CustomTopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <topic>test_cmd_vel</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "test_cmd_vel";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointControllerTest, DefaultTopicNamesWithoutNs)
{
  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <use_actuator_msg>false</use_actuator_msg>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/model/joint_controller/joint/test_joint/cmd_vel";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <use_actuator_msg>true</use_actuator_msg>
            <actuator_number>1</actuator_number>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "/actuators";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointControllerTest, SubTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller' namespace='ns'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <sub_topic>/sub_topic</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/sub_topic";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_controller_test_world'>
        <model name='joint_controller'>
          <plugin name='gz::sim::systems::JointController'
                  filename='gz-sim-joint-controller-system'>
            <joint_name>test_joint</joint_name>
            <sub_topic>/sub_topic</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "/model/joint_controller/sub_topic";
  TestTopicNames(sdfString, expectedTopicNames);
}