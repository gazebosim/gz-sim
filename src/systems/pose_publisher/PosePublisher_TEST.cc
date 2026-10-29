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

#include "PosePublisher.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for PosePublisher system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const PosePublisher::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  PosePublisher plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.poseTopic, _expectedTopicNames.poseTopic);
  EXPECT_EQ(topics.staticPoseTopic, _expectedTopicNames.staticPoseTopic);
}

TEST(PosePublisherTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='pose_publisher_test_world'>
        <model name='pose_publisher' namespace='ns'>
          <plugin name='gz::sim::systems::PosePublisher'
                  filename='gz-sim-pose-publisher-system'>
            <topic>/test_pose</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  PosePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.poseTopic = "/test_pose";
  expectedTopicNames.staticPoseTopic = "/test_pose_static";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(PosePublisherTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='pose_publisher_test_world'>
        <model name='pose_publisher' namespace='ns'>
          <plugin name='gz::sim::systems::PosePublisher'
                  filename='gz-sim-pose-publisher-system'>
            <topic>test_pose</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  PosePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.poseTopic = "ns/test_pose";
  expectedTopicNames.staticPoseTopic = "ns/test_pose_static";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(PosePublisherTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='pose_publisher_test_world'>
        <model name='pose_publisher' namespace='ns'>
          <plugin name='gz::sim::systems::PosePublisher'
                  filename='gz-sim-pose-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  PosePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.poseTopic = "ns/pose";
  expectedTopicNames.staticPoseTopic = "ns/pose_static";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(PosePublisherTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='pose_publisher_test_world'>
        <model name='pose_publisher'>
          <plugin name='gz::sim::systems::PosePublisher'
                  filename='gz-sim-pose-publisher-system'>
            <topic>test_pose</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  PosePublisher::TopicNames expectedTopicNames;
  expectedTopicNames.poseTopic = "test_pose";
  expectedTopicNames.staticPoseTopic = "test_pose_static";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='pose_publisher_test_world'>
        <model name='pose_publisher'>
          <plugin name='gz::sim::systems::PosePublisher'
                  filename='gz-sim-pose-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.poseTopic = "/pose_publisher/pose";
  expectedTopicNames.staticPoseTopic = "/pose_publisher/pose_static";
  TestTopicNames(sdfString, expectedTopicNames);
}
