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

#include "DetachableJoint.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for DetachableJoint system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const DetachableJoint::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  DetachableJoint plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.detachTopic, _expectedTopicNames.detachTopic);
  EXPECT_EQ(topics.attachTopic, _expectedTopicNames.attachTopic);
  EXPECT_EQ(topics.outputTopic, _expectedTopicNames.outputTopic);
}

TEST(DetachableJointTest, AbsoluteTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='detachable_joint_test_world'>
        <model name='detachable_joint' namespace='ns'>
          <link name='parent_link'/>
          <model name='child'>
            <link name='child_link'/>
          </model>
          <plugin name='gz::sim::systems::DetachableJoint'
                  filename='gz-sim-detachable-joint-system'>
            <parent_link>parent_link</parent_link>
            <child_model>child</child_model>
            <child_link>child_link</child_link>
            <detach_topic>/test_detach</detach_topic>
            <attach_topic>/test_attach</attach_topic>
            <output_topic>/test_output</output_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DetachableJoint::TopicNames expectedTopicNames;
  expectedTopicNames.detachTopic = "/test_detach";
  expectedTopicNames.attachTopic = "/test_attach";
  expectedTopicNames.outputTopic = "/test_output";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DetachableJointTest, RelativeTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='detachable_joint_test_world'>
        <model name='detachable_joint' namespace='ns'>
          <link name='parent_link'/>
          <model name='child'>
            <link name='child_link'/>
          </model>
          <plugin name='gz::sim::systems::DetachableJoint'
                  filename='gz-sim-detachable-joint-system'>
            <parent_link>parent_link</parent_link>
            <child_model>child</child_model>
            <child_link>child_link</child_link>
            <detach_topic>test_detach</detach_topic>
            <attach_topic>test_attach</attach_topic>
            <output_topic>test_output</output_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DetachableJoint::TopicNames expectedTopicNames;
  expectedTopicNames.detachTopic = "ns/test_detach";
  expectedTopicNames.attachTopic = "ns/test_attach";
  expectedTopicNames.outputTopic = "ns/test_output";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DetachableJointTest, DefaultTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='detachable_joint_test_world'>
        <model name='detachable_joint' namespace='ns'>
          <link name='parent_link'/>
          <model name='child'>
            <link name='child_link'/>
          </model>
          <plugin name='gz::sim::systems::DetachableJoint'
                  filename='gz-sim-detachable-joint-system'>
            <parent_link>parent_link</parent_link>
            <child_model>child</child_model>
            <child_link>child_link</child_link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DetachableJoint::TopicNames expectedTopicNames;
  expectedTopicNames.detachTopic = "ns/detachable_joint/detach";
  expectedTopicNames.attachTopic = "ns/detachable_joint/attach";
  expectedTopicNames.outputTopic = "ns/detachable_joint/state";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(DetachableJointTest, TopicNamesWithoutNs)
{
  // Custom relative topics and the <topic> alias keep their names.
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='detachable_joint_test_world'>
        <model name='detachable_joint'>
          <link name='parent_link'/>
          <model name='child'>
            <link name='child_link'/>
          </model>
          <plugin name='gz::sim::systems::DetachableJoint'
                  filename='gz-sim-detachable-joint-system'>
            <parent_link>parent_link</parent_link>
            <child_model>child</child_model>
            <child_link>child_link</child_link>
            <topic>test_detach</topic>
            <attach_topic>test_attach</attach_topic>
            <output_topic>test_output</output_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  DetachableJoint::TopicNames expectedTopicNames;
  expectedTopicNames.detachTopic = "test_detach";
  expectedTopicNames.attachTopic = "test_attach";
  expectedTopicNames.outputTopic = "test_output";
  TestTopicNames(sdfString, expectedTopicNames);

  // Default topics keep the original names.
  sdfString = R"(
    <sdf version='1.10'>
      <world name='detachable_joint_test_world'>
        <model name='detachable_joint'>
          <link name='parent_link'/>
          <model name='child'>
            <link name='child_link'/>
          </model>
          <plugin name='gz::sim::systems::DetachableJoint'
                  filename='gz-sim-detachable-joint-system'>
            <parent_link>parent_link</parent_link>
            <child_model>child</child_model>
            <child_link>child_link</child_link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.detachTopic =
      "/model/detachable_joint/detachable_joint/detach";
  expectedTopicNames.attachTopic =
      "/model/detachable_joint/detachable_joint/attach";
  expectedTopicNames.outputTopic = "/model/child/detachable_joint/state";
  TestTopicNames(sdfString, expectedTopicNames);
}
