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

#include "Thruster.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for Thruster system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
    const Thruster::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  Thruster plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdTopic, _expectedTopicNames.cmdTopic);
  EXPECT_EQ(topics.feedbackTopic, _expectedTopicNames.feedbackTopic);
  EXPECT_EQ(topics.deadbandTopic, _expectedTopicNames.deadbandTopic);
}

/// \brief Build an SDF world with a Thruster model plugin
/// \param[in] _modelNs Model namespace. Can be empty.
/// \param[in] _ns Legacy plugin namespace. Can be empty.
/// \param[in] _topic Custom command topic. Can be empty.
/// \param[in] _useAngVelCmd Whether to use angular velocity commands.
/// \return SDF world string
std::string MakeSdf(const std::string &_modelNs, const std::string &_ns,
    const std::string &_topic, bool _useAngVelCmd)
{
  const std::string modelNs = _modelNs.empty() ? "" :
      " namespace='" + _modelNs + "'";
  const std::string ns = _ns.empty() ? "" :
      "<namespace>" + _ns + "</namespace>";
  const std::string topic = _topic.empty() ? "" :
      "<topic>" + _topic + "</topic>";
  const std::string angVelCmd = _useAngVelCmd ? "true" : "false";

  return R"(<sdf version='1.10'>
    <world name='thruster_test_world'>
      <model name='thruster')" + modelNs + R"(>
        <link name='base_link'/>
        <link name='propeller'/>
        <joint name='propeller_joint' type='revolute'>
          <parent>base_link</parent>
          <child>propeller</child>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <plugin name='gz::sim::systems::Thruster'
                filename='gz-sim-thruster-system'>
          <joint_name>propeller_joint</joint_name>
          <use_angvel_cmd>)" + angVelCmd + R"(</use_angvel_cmd>
          )" + ns + topic + R"(
        </plugin>
      </model>
    </world>
  </sdf>)";
}

TEST(ThrusterTest, DefaultTopicNames)
{
  // Test the default topic names when no model namespace, no legacy namespace,
  // and no custom topic is specified.
  Thruster::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic =
      "/model/thruster/joint/propeller_joint/cmd_thrust";
  expectedTopicNames.feedbackTopic =
      "/model/thruster/joint/propeller_joint/ang_vel";
  expectedTopicNames.deadbandTopic =
      "/model/thruster/joint/propeller_joint/enable_deadband";
  TestTopicNames(MakeSdf("", "", "", false), expectedTopicNames);

  expectedTopicNames.cmdTopic =
      "/model/thruster/joint/propeller_joint/cmd_vel";
  expectedTopicNames.feedbackTopic =
      "/model/thruster/joint/propeller_joint/force";
  TestTopicNames(MakeSdf("", "", "", true), expectedTopicNames);

  // Test the default topic names when no model namespace, no custom topic is
  // specified, but a legacy namespace is specified.
  expectedTopicNames.cmdTopic =
      "/model/legacy_ns/joint/propeller_joint/cmd_thrust";
  expectedTopicNames.feedbackTopic =
      "/model/legacy_ns/joint/propeller_joint/ang_vel";
  expectedTopicNames.deadbandTopic =
      "/model/legacy_ns/joint/propeller_joint/enable_deadband";
  TestTopicNames(MakeSdf("", "legacy_ns", "", false), expectedTopicNames);

  expectedTopicNames.cmdTopic =
      "/model/legacy_ns/joint/propeller_joint/cmd_vel";
  expectedTopicNames.feedbackTopic =
      "/model/legacy_ns/joint/propeller_joint/force";
  TestTopicNames(MakeSdf("", "legacy_ns", "", true), expectedTopicNames);

  // Test the default topic names when no leagacy namespace, no custom topic is
  // specified, but a model namespace is specified.
  expectedTopicNames.cmdTopic = "ns/cmd_thrust";
  expectedTopicNames.feedbackTopic = "ns/ang_vel";
  expectedTopicNames.deadbandTopic = "ns/enable_deadband";
  TestTopicNames(MakeSdf("ns", "", "", false), expectedTopicNames);

  expectedTopicNames.cmdTopic = "ns/cmd_vel";
  expectedTopicNames.feedbackTopic = "ns/force";
  TestTopicNames(MakeSdf("ns", "", "", true), expectedTopicNames);

  // Test the default topic names when both a model namespace and a
  // legacy namespaceare specified, but no custom topic is specified.
  expectedTopicNames.cmdTopic = "ns/cmd_thrust";
  expectedTopicNames.feedbackTopic = "ns/ang_vel";
  expectedTopicNames.deadbandTopic = "ns/enable_deadband";
  TestTopicNames(MakeSdf("ns", "legacy_ns", "", false), expectedTopicNames);

  expectedTopicNames.cmdTopic = "ns/cmd_vel";
  expectedTopicNames.feedbackTopic = "ns/force";
  TestTopicNames(MakeSdf("ns", "legacy_ns", "", true), expectedTopicNames);
}

TEST(ThrusterTest, RelativeTopicNames)
{
  // Test the relative topic names when no model namespace and no legacy
  // namespace is specified.
  Thruster::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "thruster/test_cmd";
  expectedTopicNames.feedbackTopic = "thruster/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "thruster/test_cmd/enable_deadband";

  TestTopicNames(MakeSdf("", "", "test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "thruster/test_cmd/force";
  TestTopicNames(MakeSdf("", "", "test_cmd", true), expectedTopicNames);

  // Test the relative topic names when both a model namespace and a legacy
  // namespace are specified.
  expectedTopicNames.cmdTopic = "legacy_ns/test_cmd";
  expectedTopicNames.feedbackTopic = "legacy_ns/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "legacy_ns/test_cmd/enable_deadband";
  TestTopicNames(
      MakeSdf("", "legacy_ns", "test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "legacy_ns/test_cmd/force";
  TestTopicNames(
      MakeSdf("", "legacy_ns", "test_cmd", true), expectedTopicNames);

  // Test the relative topic names when a model namespace is specified,
  // but no legacy namespace is specified.
  expectedTopicNames.cmdTopic = "ns/test_cmd";
  expectedTopicNames.feedbackTopic = "ns/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "ns/test_cmd/enable_deadband";
  TestTopicNames(MakeSdf("ns", "", "test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "ns/test_cmd/force";
  TestTopicNames(MakeSdf("ns", "", "test_cmd", true), expectedTopicNames);

  // Test the relative topic names when both a model namespace and a legacy
  // namespace are specified.
  expectedTopicNames.cmdTopic = "ns/test_cmd";
  expectedTopicNames.feedbackTopic = "ns/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "ns/test_cmd/enable_deadband";
  TestTopicNames(
      MakeSdf("ns", "legacy_ns", "test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "ns/test_cmd/force";
  TestTopicNames(
      MakeSdf("ns", "legacy_ns", "test_cmd", true), expectedTopicNames);
}

TEST(ThrusterTest, AbsoluteTopicNames)
{
  // Test the absolute topic names when no model namespace and no legacy
  // namespace is specified.
  Thruster::TopicNames expectedTopicNames;
  expectedTopicNames.cmdTopic = "thruster/test_cmd";
  expectedTopicNames.feedbackTopic = "thruster/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "thruster/test_cmd/enable_deadband";
  TestTopicNames(MakeSdf("", "", "/test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "thruster/test_cmd/force";
  TestTopicNames(MakeSdf("", "", "/test_cmd", true), expectedTopicNames);

  // Test the absolute topic names when a legacy namespace is specified,
  // but no model namespace is specified.
  expectedTopicNames.cmdTopic = "legacy_ns/test_cmd";
  expectedTopicNames.feedbackTopic = "legacy_ns/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "legacy_ns/test_cmd/enable_deadband";
  TestTopicNames(
      MakeSdf("", "legacy_ns", "/test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "legacy_ns/test_cmd/force";
  TestTopicNames(
      MakeSdf("", "legacy_ns", "/test_cmd", true), expectedTopicNames);

  // Test the absolute topic names when a model namespace is specified,
  // but no legacy namespace is specified.
  expectedTopicNames.cmdTopic = "/test_cmd";
  expectedTopicNames.feedbackTopic = "/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "/test_cmd/enable_deadband";
  TestTopicNames(MakeSdf("ns", "", "/test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "/test_cmd/force";
  TestTopicNames(MakeSdf("ns", "", "/test_cmd", true), expectedTopicNames);

  // Test the absolute topic names when both a model namespace and a legacy
  // namespace are specified.
  expectedTopicNames.cmdTopic = "/test_cmd";
  expectedTopicNames.feedbackTopic = "/test_cmd/ang_vel";
  expectedTopicNames.deadbandTopic = "/test_cmd/enable_deadband";
  TestTopicNames(
      MakeSdf("ns", "legacy_ns", "/test_cmd", false), expectedTopicNames);

  expectedTopicNames.feedbackTopic = "/test_cmd/force";
  TestTopicNames(
      MakeSdf("ns", "legacy_ns", "/test_cmd", true), expectedTopicNames);
}
