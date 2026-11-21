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

#include "JointStatePublisher.hh"

#include <gtest/gtest.h>

#include <string>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Util.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for the JointStatePublisher system.
/// \param[in] _sdfString The SDF string to load.
/// \param[in] _expectedTopicNames The expected resolved topic names.
void TestTopicNames(const std::string &_sdfString,
    const JointStatePublisher::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;
  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  JointStatePublisher plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();
  EXPECT_EQ(topics.jointStateTopic, _expectedTopicNames.jointStateTopic);
}

//////////////////////////////////////////////////
TEST(JointStatePublisherTest, AbsoluteTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_state_test_world'>
        <model name='joint_state_model' namespace='ns'>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
            <topic>/test_joint_state</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointStatePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.jointStateTopic = "/test_joint_state";
  TestTopicNames(sdfString, expectedTopicNames);
}

//////////////////////////////////////////////////
TEST(JointStatePublisherTest, RelativeTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_state_test_world'>
        <model name='joint_state_model' namespace='ns'>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
            <topic>test_joint_state</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointStatePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.jointStateTopic = "ns/test_joint_state";
  TestTopicNames(sdfString, expectedTopicNames);
}

//////////////////////////////////////////////////
TEST(JointStatePublisherTest, DefaultTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_state_test_world'>
        <model name='joint_state_model' namespace='ns'>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointStatePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.jointStateTopic = "ns/joint_state";
  TestTopicNames(sdfString, expectedTopicNames);
}

//////////////////////////////////////////////////
TEST(JointStatePublisherTest, CustomTopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  {
    // Incomplete sdf, only used to test topic resolution behavior
    const std::string sdfString = R"(
      <sdf version='1.10'>
        <world name='joint_state_test_world'>
          <model name='joint_state_model'>
            <plugin name='gz::sim::systems::JointStatePublisher'
                    filename='gz-sim-joint-state-publisher-system'>
              <topic>test_joint_state</topic>
            </plugin>
          </model>
        </world>
      </sdf>)";

    JointStatePublisher::TopicNames expectedTopicNames;
    expectedTopicNames.jointStateTopic = "test_joint_state";
    TestTopicNames(sdfString, expectedTopicNames);
  }

  {
    // Incomplete sdf, only used to test topic resolution behavior
    const std::string sdfString = R"(
      <sdf version='1.10'>
        <world name='joint_state_test_world'>
          <model name='joint_state_model'>
            <plugin name='gz::sim::systems::JointStatePublisher'
                    filename='gz-sim-joint-state-publisher-system'>
              <topic>/test_joint_state</topic>
            </plugin>
          </model>
        </world>
      </sdf>)";

    JointStatePublisher::TopicNames expectedTopicNames;
    expectedTopicNames.jointStateTopic = "/test_joint_state";
    TestTopicNames(sdfString, expectedTopicNames);
  }
}

//////////////////////////////////////////////////
TEST(JointStatePublisherTest, DefaultTopicNameWithoutNs)
{
  // Verify that the original default topic name is preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_state_test_world'>
        <model name='joint_state_model'>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;
  LoadModelContext(sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  JointStatePublisher plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  // The unit test context has no world entity, so compare against the default
  // that existed before namespace support instead of hard-coding the world
  // prefix.
  const std::string expected =
      topicFromScopedName(modelEntity, ecm, false) + "/joint_state";
  EXPECT_EQ(plugin.ResolvedTopicNames().jointStateTopic, expected);
}
