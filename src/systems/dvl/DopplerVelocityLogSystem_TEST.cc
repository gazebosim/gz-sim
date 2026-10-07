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

#include "DopplerVelocityLogSystem.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for DopplerVelocityLog system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicName(const std::string &_sdfString,
      const std::string &_expectedTopicName)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;

  LoadModelContext(_sdfString, ecm, eventMgr, modelEntity);

  DopplerVelocityLogSystem plugin;
  sdf::ElementPtr nullSdf;
  plugin.Configure(modelEntity, nullSdf, ecm, eventMgr);

  UpdateInfo info;
  info.paused = true;
  plugin.PreUpdate(info, ecm);
  plugin.PostUpdate(info, ecm);

  const auto topics = plugin.ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, _expectedTopicName);
}

TEST(DopplerVelocityLogSystemTest, AbsoluteTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="dvl_sensor_model" namespace="ns">
        <link name="link">
          <sensor name="dvl_sensor" type="custom" gz:type="dvl">
            <topic>/test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/test_sensor_topic");
}

TEST(DopplerVelocityLogSystemTest, RelativeTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="dvl_sensor_model" namespace="ns">
        <link name="link">
          <sensor name="dvl_sensor" type="custom" gz:type="dvl">
            <topic>test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/test_sensor_topic");
}

TEST(DopplerVelocityLogSystemTest, DefaultTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="dvl_sensor_model" namespace="ns">
        <link name="link">
          <sensor name="dvl_sensor" type="custom" gz:type="dvl">
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/dvl/velocity");
}

TEST(DopplerVelocityLogSystemTest, TopicNamesWithoutNs)
{
  std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="dvl_sensor_model">
        <link name="link">
          <sensor name="dvl_sensor" type="custom" gz:type="dvl">
            <topic>test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "test_sensor_topic");

  sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="dvl_sensor_model">
        <link name="link">
          <sensor name="dvl_sensor" type="custom" gz:type="dvl">
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString,
    "model/dvl_sensor_model/link/link/sensor/dvl_sensor/dvl/velocity");
}
