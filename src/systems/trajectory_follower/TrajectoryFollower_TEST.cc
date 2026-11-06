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

#include "TrajectoryFollower.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Types.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for TrajectoryFollower system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const TrajectoryFollower::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  TrajectoryFollower plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  // Topics are resolved in Load(), which runs on the first unpaused PreUpdate
  UpdateInfo info;
  info.paused = false;
  info.dt = std::chrono::milliseconds(1);
  plugin.PreUpdate(info, ecm);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.pauseTopic, _expectedTopicNames.pauseTopic);
}

TEST(TrajectoryFollowerTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='trajectory_follower_test_world'>
        <model name='trajectory_follower' namespace='ns'>
          <link name='base_link'/>
          <plugin name='gz::sim::systems::TrajectoryFollower'
                  filename='gz-sim-trajectory-follower-system'>
            <link_name>base_link</link_name>
            <topic>/test_pause</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrajectoryFollower::TopicNames expectedTopicNames;
  expectedTopicNames.pauseTopic = "/test_pause";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrajectoryFollowerTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='trajectory_follower_test_world'>
        <model name='trajectory_follower' namespace='ns'>
          <link name='base_link'/>
          <plugin name='gz::sim::systems::TrajectoryFollower'
                  filename='gz-sim-trajectory-follower-system'>
            <link_name>base_link</link_name>
            <topic>test_pause</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrajectoryFollower::TopicNames expectedTopicNames;
  expectedTopicNames.pauseTopic = "ns/test_pause";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrajectoryFollowerTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='trajectory_follower_test_world'>
        <model name='trajectory_follower' namespace='ns'>
          <link name='base_link'/>
          <plugin name='gz::sim::systems::TrajectoryFollower'
                  filename='gz-sim-trajectory-follower-system'>
            <link_name>base_link</link_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrajectoryFollower::TopicNames expectedTopicNames;
  expectedTopicNames.pauseTopic = "ns/trajectory_follower/pause";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(TrajectoryFollowerTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured topic names remain unchanged when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='trajectory_follower_test_world'>
        <model name='trajectory_follower'>
          <link name='base_link'/>
          <plugin name='gz::sim::systems::TrajectoryFollower'
                  filename='gz-sim-trajectory-follower-system'>
            <link_name>base_link</link_name>
            <topic>test_pause</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TrajectoryFollower::TopicNames expectedTopicNames;
  expectedTopicNames.pauseTopic = "test_pause";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the original default topic names are preserved when no
  // namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='trajectory_follower_test_world'>
        <model name='trajectory_follower'>
          <link name='base_link'/>
          <plugin name='gz::sim::systems::TrajectoryFollower'
                  filename='gz-sim-trajectory-follower-system'>
            <link_name>base_link</link_name>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.pauseTopic =
    "/model/trajectory_follower/trajectory_follower/pause";
  TestTopicNames(sdfString, expectedTopicNames);
}