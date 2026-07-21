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

#include <gtest/gtest.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "AckermannSteering.cc"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Test topic name resolution for AckermannSteering system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
  const AckermannSteering::TopicNames &_expectedTopicNames)
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

  auto plugin = new AckermannSteering();
  plugin->Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  const auto topics = plugin->ResolvedTopicNames();

  EXPECT_EQ(topics.cmdVelTopic, _expectedTopicNames.cmdVelTopic);
  if (pluginSdf.Element()->HasElement("steering_only") &&
      pluginSdf.Element()->Get<bool>("steering_only"))
  {
    return;
  }
  EXPECT_EQ(topics.odomTopic, _expectedTopicNames.odomTopic);
  EXPECT_EQ(topics.tfTopic, _expectedTopicNames.tfTopic);
}

TEST(AckermannSteeringTest, AbsoluteTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <topic>/test_cmd_vel</topic>
            <odom_topic>/test_odom</odom_topic>
            <tf_topic>/test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/test_cmd_vel";
  expectedTopicNames.odomTopic = "/test_odom";
  expectedTopicNames.tfTopic = "/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, RelativeTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <topic>test_cmd_vel</topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/ns/test_cmd_vel";
  expectedTopicNames.odomTopic = "/ns/test_odom";
  expectedTopicNames.tfTopic = "/ns/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, SubTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <sub_topic>test_cmd_vel</sub_topic>
            <odom_topic>test_odom</odom_topic>
            <tf_topic>test_tf</tf_topic>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/ns/test_cmd_vel";
  expectedTopicNames.odomTopic = "/ns/test_odom";
  expectedTopicNames.tfTopic = "/ns/test_tf";
  TestTopicNames(sdfString, expectedTopicNames);
}

TEST(AckermannSteeringTest, DefaultTopicNames)
{
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  AckermannSteering::TopicNames expectedTopicNames;
  expectedTopicNames.cmdVelTopic = "/ns/cmd_vel";
  expectedTopicNames.odomTopic = "/ns/odometry";
  expectedTopicNames.tfTopic = "/ns/tf";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  expectedTopicNames.cmdVelTopic = "/ns/steer_angle";
  TestTopicNames(sdfString, expectedTopicNames);

  // Incomplete sdf, only used to test topic resolution behavior
  sdfString = R"(
    <sdf version='1.10'>
      <world name='ackermann_steering_test_world'>
        <model name='ackermann_steering' namespace='ns'>
          <plugin name='gz::sim::systems::AckermannSteering'
                  filename='gz-sim-ackermann-steering-system'>
            <steering_only>true</steering_only>
            <use_actuator_msg>true</use_actuator_msg>
            <actuator_number>1</actuator_number>
          </plugin>
        </model>
      </world>
    </sdf>)";
  
  expectedTopicNames.cmdVelTopic = "/ns/actuators";
  TestTopicNames(sdfString, expectedTopicNames);
}
