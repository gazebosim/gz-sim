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

#include "ForceTorque.hh"

#include <gtest/gtest.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for ForceTorque system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicName(const std::string &_sdfString,
      const std::string &_expectedTopicName)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity);

  auto plugin = std::make_unique<ForceTorque>();
  UpdateInfo info;
  info.paused = true;
  plugin->PreUpdate(info, ecm);
  plugin->Update(info, ecm);

  const auto topics = plugin->ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, _expectedTopicName);
}

TEST(ForceTorqueTest, AbsoluteTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="force_torque_model" namespace="ns">
        <link name="link">
          <sensor name="force_torque" type="force_torque">
            <topic>/test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/test_sensor_topic");
}

TEST(ForceTorqueTest, RelativeTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="force_torque_model" namespace="ns">
        <link name="link">
          <sensor name="force_torque" type="force_torque">
            <topic>test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/test_sensor_topic");
}

TEST(ImuTest, DefaultTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="force_torque_model" namespace="ns">
        <link name="link">
          <sensor name="force_torque" type="force_torque">
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/forcetorque");
}