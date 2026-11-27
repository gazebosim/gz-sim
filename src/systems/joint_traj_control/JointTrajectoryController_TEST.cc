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

#include "JointTrajectoryController.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for JointTrajectoryController system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const JointTrajectoryController::TopicNames &_expectedTopicNames)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  JointTrajectoryController plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin.ResolvedTopicNames();

  EXPECT_EQ(topics.trajectoryTopic, _expectedTopicNames.trajectoryTopic);
  EXPECT_EQ(topics.progressTopic, _expectedTopicNames.progressTopic);
}

TEST(JointTrajectoryControllerTest, AbsoluteTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_traj_test_world'>
        <model name='joint_traj_model' namespace='ns'>
          <link name='link1'/>
          <link name='link2'/>
          <joint name='joint1' type='revolute'>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
              <xyz>0 0 1</xyz>
            </axis>
          </joint>
          <plugin name='gz::sim::systems::JointTrajectoryController'
                  filename='gz-sim-joint-trajectory-controller-system'>
            <topic>/test_cmd_traj</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointTrajectoryController::TopicNames expectedTopicNames;
  expectedTopicNames.trajectoryTopic = "/test_cmd_traj";
  expectedTopicNames.progressTopic = "/test_cmd_traj_progress";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointTrajectoryControllerTest, RelativeTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_traj_test_world'>
        <model name='joint_traj_model' namespace='ns'>
          <link name='link1'/>
          <link name='link2'/>
          <joint name='joint1' type='revolute'>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
              <xyz>0 0 1</xyz>
            </axis>
          </joint>
          <plugin name='gz::sim::systems::JointTrajectoryController'
                  filename='gz-sim-joint-trajectory-controller-system'>
            <topic>test_cmd_traj</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointTrajectoryController::TopicNames expectedTopicNames;
  expectedTopicNames.trajectoryTopic = "ns/test_cmd_traj";
  expectedTopicNames.progressTopic = "ns/test_cmd_traj_progress";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointTrajectoryControllerTest, DefaultTopicNames)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_traj_test_world'>
        <model name='joint_traj_model' namespace='ns'>
          <link name='link1'/>
          <link name='link2'/>
          <joint name='joint1' type='revolute'>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
              <xyz>0 0 1</xyz>
            </axis>
          </joint>
          <plugin name='gz::sim::systems::JointTrajectoryController'
                  filename='gz-sim-joint-trajectory-controller-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointTrajectoryController::TopicNames expectedTopicNames;
  expectedTopicNames.trajectoryTopic = "ns/joint_trajectory";
  expectedTopicNames.progressTopic = "ns/joint_trajectory_progress";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(JointTrajectoryControllerTest, TopicNamesWithoutNs)
{
  // Verify that explicitly configured relative topic names remain unchanged
  // when no namespace is specified.
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_traj_test_world'>
        <model name='joint_traj_model'>
          <link name='link1'/>
          <link name='link2'/>
          <joint name='joint1' type='revolute'>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
              <xyz>0 0 1</xyz>
            </axis>
          </joint>
          <plugin name='gz::sim::systems::JointTrajectoryController'
                  filename='gz-sim-joint-trajectory-controller-system'>
            <topic>test_cmd_traj</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  JointTrajectoryController::TopicNames expectedTopicNames;
  expectedTopicNames.trajectoryTopic = "test_cmd_traj";
  expectedTopicNames.progressTopic = "test_cmd_traj_progress";
  TestTopicNames(sdfString, expectedTopicNames);

  // Verify that the default topic names fall back to
  // /model/{model_name}/joint_trajectory when no namespace is specified.
  sdfString = R"(
    <sdf version='1.10'>
      <world name='joint_traj_test_world'>
        <model name='joint_traj_model'>
          <link name='link1'/>
          <link name='link2'/>
          <joint name='joint1' type='revolute'>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
              <xyz>0 0 1</xyz>
            </axis>
          </joint>
          <plugin name='gz::sim::systems::JointTrajectoryController'
                  filename='gz-sim-joint-trajectory-controller-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  expectedTopicNames.trajectoryTopic =
      "/model/joint_traj_model/joint_trajectory";
  expectedTopicNames.progressTopic =
      "/model/joint_traj_model/joint_trajectory_progress";
  TestTopicNames(sdfString, expectedTopicNames);
}
