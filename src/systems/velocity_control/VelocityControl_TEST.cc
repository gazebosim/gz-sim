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

#include "VelocityControl.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for VelocityControl system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const VelocityControl::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  VelocityControl plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.modelCmdVelTopic,
            _expectedTopicNames.modelCmdVelTopic);
  EXPECT_EQ(topics.linkCmdVelTopics,
            _expectedTopicNames.linkCmdVelTopics);
}

TEST(VelocityControlTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='velocity_control_test_world'>
        <model name='velocity_control' namespace='ns'>
          <plugin name='gz::sim::systems::VelocityControl'
                  filename='gz-sim-velocity-control-system'>
            <topic>/test_cmd_vel</topic>
            <link>
              <name>link1</name>
              <topic>/test_link1_cmd_vel</topic>
            </link>
            <link>
              <name>link2</name>
              <topic>/test_link2_cmd_vel</topic>
            </link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  VelocityControl::TopicNames expectedTopicNames;
  expectedTopicNames.modelCmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.linkCmdVelTopics = {
    {"link1", "/test_link1_cmd_vel"},
    {"link2", "/test_link2_cmd_vel"}
  };
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(VelocityControlTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='velocity_control_test_world'>
        <model name='velocity_control' namespace='ns'>
          <plugin name='gz::sim::systems::VelocityControl'
                  filename='gz-sim-velocity-control-system'>
            <topic>test_cmd_vel</topic>
            <link>
              <name>link1</name>
              <topic>test_link1_cmd_vel</topic>
            </link>
            <link>
              <name>link2</name>
              <topic>test_link2_cmd_vel</topic>
            </link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  VelocityControl::TopicNames expectedTopicNames;
  expectedTopicNames.modelCmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.linkCmdVelTopics = {
    {"link1", "ns/test_link1_cmd_vel"},
    {"link2", "ns/test_link2_cmd_vel"}
  };
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(VelocityControlTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='velocity_control_test_world'>
        <model name='velocity_control' namespace='ns'>
          <plugin name='gz::sim::systems::VelocityControl'
                  filename='gz-sim-velocity-control-system'>
            <link_name>link1</link_name>
            <link>
              <name>link2</name>
            </link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  VelocityControl::TopicNames expectedTopicNames;
  expectedTopicNames.modelCmdVelTopic = "ns/cmd_vel";
  expectedTopicNames.linkCmdVelTopics = {
    {"link1", "ns/cmd_vel"},
    {"link2", "ns/cmd_vel"}
  };
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(VelocityControlTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='velocity_control_test_world'>
        <model name='velocity_control'>
          <plugin name='gz::sim::systems::VelocityControl'
                  filename='gz-sim-velocity-control-system'>
            <topic>test_cmd_vel</topic>
            <link>
              <name>link1</name>
              <topic>test_link1_cmd_vel</topic>
            </link>
            <link>
              <name>link2</name>
              <topic>/test_link2_cmd_vel</topic>
            </link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  VelocityControl::TopicNames expectedTopicNames;
  expectedTopicNames.modelCmdVelTopic = "test_cmd_vel";
  expectedTopicNames.linkCmdVelTopics = {
    {"link1", "test_link1_cmd_vel"},
    {"link2", "/test_link2_cmd_vel"}
  };
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='velocity_control_test_world'>
        <model name='velocity_control'>
          <plugin name='gz::sim::systems::VelocityControl'
                  filename='gz-sim-velocity-control-system'>
            <link_name>link1</link_name>
            <link>
              <name>link2</name>
            </link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.modelCmdVelTopic =
      "/model/velocity_control/cmd_vel";
  expectedTopicNames.linkCmdVelTopics = {
    {"link1", "/model/velocity_control/link/link1/cmd_vel"},
    {"link2", "/model/velocity_control/link/link2/cmd_vel"}
  };
  TestTopicNames(sdfString, expectedTopicNames);
}
