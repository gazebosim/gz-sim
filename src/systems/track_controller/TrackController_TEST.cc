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

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Test topic name resolution for TrackController system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const TrackController::TopicNames &_expectedTopicNames)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);

  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);

  ASSERT_EQ(1u, worldSdf->ModelCount());
  const sdf::Model *modelSdf = worldSdf->ModelByIndex(0);
  ASSERT_NE(nullptr, modelSdf);

  ASSERT_FALSE(modelSdf->Plugins().empty());
  const sdf::Plugin &pluginSdf = modelSdf->Plugins()[0];
  
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator entityCreator(ecm, eventMgr);

  Entity modelEntity =
    entityCreator.CreateEntitiesWithoutLoadingPlugins(modelSdf);
  ASSERT_NE(kNullEntity, modelEntity);

  auto plugin = new TrackController();
  plugin->Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin->ResolvedTopicNames();

  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  EXPECT_EQ(topics.corTopic, _expectedTopicNames.corTopic);
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
}

TEST(TrackControllerTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='track_controller_test_world'>
        <model name='track_controller' namespace='ns'>
          <link name='test_link'/>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_link</link>
            <velocity_topic>/test_cmd_vel</velocity_topic>
            <center_of_rotation_topic>/test_cor</center_of_rotation_topic>
            <odometry_topic>/test_odom</odometry_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.corTopic = "/test_cor";
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
          <link name='test_link'/>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_link</link>
            <velocity_topic>test_cmd_vel</velocity_topic>
            <center_of_rotation_topic>test_cor</center_of_rotation_topic>
            <odometry_topic>test_odom</odometry_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "ns/test_cmd_vel";
  expectedTopicNames.corTopic = "ns/test_cor";
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
          <link name='test_link'/>
          <plugin name='gz::sim::systems::TrackController'
                  filename='gz-sim-track-controller-system'>
            <link>test_link</link>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrackController::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic =
    "ns/test_link/track_cmd_vel";
  expectedTopicNames.corTopic =
    "ns/test_link/track_cmd_center_of_rotation";
  expectedTopicNames.odomTopic =
    "ns/test_link/odometry";
  TestTopicNames(sdfString, expectedTopicNames);
}
