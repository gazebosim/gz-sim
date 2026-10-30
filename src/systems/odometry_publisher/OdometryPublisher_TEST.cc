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

#include "OdometryPublisher.hh"

#include <gtest/gtest.h>

#include <string>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for OdometryPublisher system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const OdometryPublisher::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  OdometryPublisher plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.odomCovTopic, _expectedTopicNames.odomCovTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
}

/////////////////////////////////////////////////
TEST(OdometryPublisherTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model' namespace='ns'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
            <odom_topic>/test_odom</odom_topic>
            <odom_covariance_topic>/test_odom_cov</odom_covariance_topic>
            <tf_topic>/test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  OdometryPublisher::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "/test_odom";
  expectedTopicNames.odomCovTopic = "/test_odom_cov";
  expectedTopicNames.tfTopic = "/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

/////////////////////////////////////////////////
TEST(OdometryPublisherTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model' namespace='ns'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
            <odom_topic>test_odom</odom_topic>
            <odom_covariance_topic>test_odom_cov</odom_covariance_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  OdometryPublisher::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "ns/test_odom";
  expectedTopicNames.odomCovTopic = "ns/test_odom_cov";
  expectedTopicNames.tfTopic = "ns/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

/////////////////////////////////////////////////
TEST(OdometryPublisherTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model' namespace='ns'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  OdometryPublisher::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "ns/odometry";
  expectedTopicNames.odomCovTopic = "ns/odometry_with_covariance";
  expectedTopicNames.tfTopic = "ns/pose";
  TestTopicNames(sdfString, expectedTopicNames);
}

/////////////////////////////////////////////////
TEST(OdometryPublisherTest, DefaultTopicNamesWithAbsoluteNs)
{
  // A namespace starting with '/' (common in multi-robot setups) should
  // produce absolute default topics.
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model' namespace='/robot1'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  OdometryPublisher::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "/robot1/odometry";
  expectedTopicNames.odomCovTopic = "/robot1/odometry_with_covariance";
  expectedTopicNames.tfTopic = "/robot1/pose";
  TestTopicNames(sdfString, expectedTopicNames);
}

/////////////////////////////////////////////////
TEST(OdometryPublisherTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
            <odom_topic>test_odom</odom_topic>
            <odom_covariance_topic>test_odom_cov</odom_covariance_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  OdometryPublisher::TopicNames expectedTopicNames;
  expectedTopicNames.odomTopic = "test_odom";
  expectedTopicNames.odomCovTopic = "test_odom_cov";
  expectedTopicNames.tfTopic = "test_tf";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='odometry_publisher_test_world'>
        <model name='odom_model'>
          <plugin name='gz::sim::systems::OdometryPublisher'
                  filename='gz-sim-odometry-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.odomTopic = "/model/odom_model/odometry";
  expectedTopicNames.odomCovTopic =
    "/model/odom_model/odometry_with_covariance";
  expectedTopicNames.tfTopic = "/model/odom_model/pose";
  TestTopicNames(sdfString, expectedTopicNames);
}
