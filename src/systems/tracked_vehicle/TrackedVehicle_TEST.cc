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

#include "TrackedVehicle.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for TrackedVehicle system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const TrackedVehicle::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  TrackedVehicle plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
  EXPECT_EQ(topics.seTopic, _expectedTopicNames.seTopic);

  EXPECT_EQ(topics.trackVelTopics.size(),
    _expectedTopicNames.trackVelTopics.size());
  for (const auto &expectedTrackVelTopic : _expectedTopicNames.trackVelTopics)
  {
    EXPECT_EQ(topics.trackVelTopics.at(expectedTrackVelTopic.first),
      expectedTrackVelTopic.second);
  }

  EXPECT_EQ(topics.trackCorTopics.size(),
    _expectedTopicNames.trackCorTopics.size());
  for (const auto &expectedTrackCorTopic : _expectedTopicNames.trackCorTopics)
  {
    EXPECT_EQ(topics.trackCorTopics.at(expectedTrackCorTopic.first),
      expectedTrackCorTopic.second);
  }
}

TEST(TrackedVehicleTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='tracked_vehicle_test_world'>
        <model name='tracked_vehicle' namespace='ns'>
          <plugin name='gz::sim::systems::TrackedVehicle'
                  filename='gz-sim-tracked-vehicle-system'>
            <left_track>
              <link>left_track</link>
              <velocity_topic>/test_left_track_vel</velocity_topic>
              <center_of_rotation_topic>/test_left_track_cor</center_of_rotation_topic>
            </left_track>
            <right_track>
              <link>right_track</link>
              <velocity_topic>/test_right_track_vel</velocity_topic>
              <center_of_rotation_topic>/test_right_track_cor</center_of_rotation_topic>
            </right_track>
            <topic>/test_cmd_vel</topic>
            <odom_topic>/test_odom</odom_topic>
            <tf_topic>/test_tf</tf_topic>
            <steering_efficiency_topic>/test_se</steering_efficiency_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackedVehicle::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.odomTopic = "/test_odom";
  expectedTopicNames.tfTopic = "/test_tf";
  expectedTopicNames.seTopic = "/test_se";
  expectedTopicNames.trackVelTopics["left_track"] = "/test_left_track_vel";
  expectedTopicNames.trackCorTopics["left_track"] = "/test_left_track_cor";
  expectedTopicNames.trackVelTopics["right_track"] = "/test_right_track_vel";
  expectedTopicNames.trackCorTopics["right_track"] = "/test_right_track_cor";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackedVehicleTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='tracked_vehicle_test_world'>
        <model name='tracked_vehicle' namespace='ns'>
          <plugin name='gz::sim::systems::TrackedVehicle'
                  filename='gz-sim-tracked-vehicle-system'>
            <left_track>
              <link>left_track</link>
              <velocity_topic>test_left_track_vel</velocity_topic>
              <center_of_rotation_topic>test_left_track_cor</center_of_rotation_topic>
            </left_track>
            <right_track>
              <link>right_track</link>
              <velocity_topic>test_right_track_vel</velocity_topic>
              <center_of_rotation_topic>test_right_track_cor</center_of_rotation_topic>
            </right_track>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
            <steering_efficiency_topic>test_se</steering_efficiency_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackedVehicle::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.odomTopic = "ns/test_odom";
  expectedTopicNames.tfTopic = "ns/test_tf";
  expectedTopicNames.seTopic = "ns/test_se";
  expectedTopicNames.trackVelTopics["left_track"] = "ns/test_left_track_vel";
  expectedTopicNames.trackCorTopics["left_track"] = "ns/test_left_track_cor";
  expectedTopicNames.trackVelTopics["right_track"] = "ns/test_right_track_vel";
  expectedTopicNames.trackCorTopics["right_track"] = "ns/test_right_track_cor";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackedVehicleTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='tracked_vehicle_test_world'>
        <model name='tracked_vehicle' namespace='ns'>
          <plugin name='gz::sim::systems::TrackedVehicle'
                  filename='gz-sim-tracked-vehicle-system'>
            <left_track>
              <link>left_track</link>
            </left_track>
            <right_track>
              <link>right_track</link>
            </right_track>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackedVehicle::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/cmd_vel";
  expectedTopicNames.odomTopic = "ns/odometry";
  expectedTopicNames.tfTopic = "ns/tf";
  expectedTopicNames.seTopic = "ns/steering_efficiency";
  expectedTopicNames.trackVelTopics["left_track"] = "ns/track_cmd_vel";
  expectedTopicNames.trackCorTopics["left_track"]
    = "ns/track_cmd_center_of_rotation";
  expectedTopicNames.trackVelTopics["right_track"] = "ns/track_cmd_vel";
  expectedTopicNames.trackCorTopics["right_track"]
    = "ns/track_cmd_center_of_rotation";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrackedVehicleTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='tracked_vehicle_test_world'>
        <model name='tracked_vehicle'>
          <plugin name='gz::sim::systems::TrackedVehicle'
                  filename='gz-sim-tracked-vehicle-system'>
            <left_track>
              <link>left_track</link>
              <velocity_topic>test_left_track_vel</velocity_topic>
              <center_of_rotation_topic>test_left_track_cor</center_of_rotation_topic>
            </left_track>
            <right_track>
              <link>right_track</link>
              <velocity_topic>test_right_track_vel</velocity_topic>
              <center_of_rotation_topic>test_right_track_cor</center_of_rotation_topic>
            </right_track>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
            <steering_efficiency_topic>test_se</steering_efficiency_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackedVehicle::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "test_cmd_vel";
  expectedTopicNames.odomTopic = "test_odom";
  expectedTopicNames.tfTopic = "test_tf";
  expectedTopicNames.seTopic = "test_se";
  expectedTopicNames.trackVelTopics["left_track"] = "test_left_track_vel";
  expectedTopicNames.trackCorTopics["left_track"] = "test_left_track_cor";
  expectedTopicNames.trackVelTopics["right_track"] = "test_right_track_vel";
  expectedTopicNames.trackCorTopics["right_track"] = "test_right_track_cor";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='tracked_vehicle_test_world'>
        <model name='tracked_vehicle'>
          <plugin name='gz::sim::systems::TrackedVehicle'
                  filename='gz-sim-tracked-vehicle-system'>
            <left_track>
              <link>left_track</link>
            </left_track>
            <right_track>
              <link>right_track</link>
            </right_track>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.cmdVelTopic = "/model/tracked_vehicle/cmd_vel";
  expectedTopicNames.odomTopic = "/model/tracked_vehicle/odometry";
  expectedTopicNames.tfTopic = "/model/tracked_vehicle/tf";
  expectedTopicNames.seTopic = "/model/tracked_vehicle/steering_efficiency";
  expectedTopicNames.trackVelTopics["left_track"] =
    "/model/tracked_vehicle/link/left_track/track_cmd_vel";
  expectedTopicNames.trackCorTopics["left_track"] =
    "/model/tracked_vehicle/link/left_track/track_cmd_center_of_rotation";
  expectedTopicNames.trackVelTopics["right_track"] =
    "/model/tracked_vehicle/link/right_track/track_cmd_vel";
  expectedTopicNames.trackCorTopics["right_track"] =
    "/model/tracked_vehicle/link/right_track/track_cmd_center_of_rotation";
  TestTopicNames(sdfString, expectedTopicNames);
}
