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

#include "ApplyJointForce.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for ApplyJointForce system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
    const ApplyJointForce::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  ApplyJointForce plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdTopic, _expectedTopicNames.cmdTopic);
}

TEST(ApplyJointForceTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='apply_joint_force_test_world'>
        <model name='apply_joint_force' namespace='ns'>
          <plugin name='gz::sim::systems::ApplyJointForce'
                  filename='gz-sim-apply-joint-force-system'>
            <joint_name>test_joint</joint_name>
            <topic>/test_cmd_force</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  ApplyJointForce::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/test_cmd_force";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(ApplyJointForceTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='apply_joint_force_test_world'>
        <model name='apply_joint_force' namespace='ns'>
          <plugin name='gz::sim::systems::ApplyJointForce'
                  filename='gz-sim-apply-joint-force-system'>
            <joint_name>test_joint</joint_name>
            <topic>test_cmd_force</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  ApplyJointForce::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/test_cmd_force";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(ApplyJointForceTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='apply_joint_force_test_world'>
        <model name='apply_joint_force' namespace='ns'>
          <plugin name='gz::sim::systems::ApplyJointForce'
                  filename='gz-sim-apply-joint-force-system'>
            <joint_name>test_joint</joint_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  ApplyJointForce::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/cmd_force";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(ApplyJointForceTest, TopicNamesWithoutNs)
{
  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='apply_joint_force_test_world'>
        <model name='apply_joint_force'>
          <plugin name='gz::sim::systems::ApplyJointForce'
                  filename='gz-sim-apply-joint-force-system'>
            <joint_name>test_joint</joint_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  ApplyJointForce::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic =
      "/model/apply_joint_force/joint/test_joint/cmd_force";
  TestTopicNames(sdfString, expectedTopicNames);
}
