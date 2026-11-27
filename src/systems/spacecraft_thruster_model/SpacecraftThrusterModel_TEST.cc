
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

#include "SpacecraftThrusterModel.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for SpacecraftThrusterModel system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
    const SpacecraftThrusterModel::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  SpacecraftThrusterModel plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdTopic, _expectedTopicNames.cmdTopic);
}

TEST(SpacecraftThrusterModelTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster' namespace='ns'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <topic>/test_cmd</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SpacecraftThrusterModel::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "/test_cmd";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SpacecraftThrusterModelTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster' namespace='ns'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <topic>test_cmd</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SpacecraftThrusterModel::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/test_cmd";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SpacecraftThrusterModelTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster' namespace='ns'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SpacecraftThrusterModel::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "ns/spacecraft_thruster";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SpacecraftThrusterModelTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <topic>test_cmd</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SpacecraftThrusterModel::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "test_cmd";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic name is preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "spacecraft_thruster";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(SpacecraftThrusterModelTest, SubTopicNames)
{
  // Verify that the legacy <sub_topic> is appended when no namespace
  // is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <topic>test_cmd</topic>
            <sub_topic>motor</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  SpacecraftThrusterModel::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "test_cmd/motor";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that <sub_topic> is appended to the default topic.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <sub_topic>motor</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "spacecraft_thruster/motor";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that <sub_topic> is appended after namespace resolution.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster' namespace='ns'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <topic>test_cmd</topic>
            <sub_topic>motor</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "ns/test_cmd/motor";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='spacecraft_thruster_test_world'>
        <model name='spacecraft_thruster' namespace='ns'>
          <plugin name='gz::sim::systems::SpacecraftThrusterModel'
                  filename='gz-sim-spacecraft-thruster-model-system'>
            <link_name>test_link</link_name>
            <sub_topic>motor</sub_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdTopic = "ns/spacecraft_thruster/motor";
  TestTopicNames(sdfString, expectedTopicNames);
}
