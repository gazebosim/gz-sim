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

#include "TrackController.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for TrackController system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const TrackController::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  TrackController plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.cmdCorTopic, _expectedTopicNames.cmdCorTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
}

TEST(TrackControllerTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller' namespace='ns'>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_track</link>
            <velocity_topic>/test_cmd_vel</velocity_topic>
            <center_of_rotation_topic>/test_cmd_cor</center_of_rotation_topic>
            <odometry_topic>/test_odom</odometry_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.cmdCorTopic = "/test_cmd_cor";
  expectedTopicNames.odomTopic = "/test_odom";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackControllerTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller' namespace='ns'>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_track</link>
            <velocity_topic>test_cmd_vel</velocity_topic>
            <center_of_rotation_topic>test_cmd_cor</center_of_rotation_topic>
            <odometry_topic>test_odom</odometry_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.cmdCorTopic = "ns/test_cmd_cor";
  expectedTopicNames.odomTopic = "ns/test_odom";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackControllerTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller' namespace='ns'>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_track</link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/track_cmd_vel";
  expectedTopicNames.cmdCorTopic = "ns/track_cmd_center_of_rotation";
  expectedTopicNames.odomTopic = "ns/odometry";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackControllerTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller'>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_track</link>
            <velocity_topic>test_cmd_vel</velocity_topic>
            <center_of_rotation_topic>test_cmd_cor</center_of_rotation_topic>
            <odometry_topic>test_odom</odometry_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "test_cmd_vel";
  expectedTopicNames.cmdCorTopic = "test_cmd_cor";
  expectedTopicNames.odomTopic = "test_odom";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller'>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_track</link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdVelTopic =
    "/model/track_controller/link/test_track/track_cmd_vel";
  expectedTopicNames.cmdCorTopic =
    "/model/track_controller/link/test_track/track_cmd_center_of_rotation";
  expectedTopicNames.odomTopic =
    "/model/track_controller/link/test_track/odometry";
  TestTopicNames(sdfString, expectedTopicNames);
}
