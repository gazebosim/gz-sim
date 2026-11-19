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

#include "Contact.hh"

#include <gtest/gtest.h>

#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for Contact system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicName The expected resolved topic name
void TestTopicName(const std::string &_sdfString,
      const std::string &_expectedTopicName)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity worldEntity;

  LoadWorldContext(_sdfString, ecm, eventMgr, worldEntity);

  Contact plugin;

  UpdateInfo info;
  info.paused = true;
  plugin.PreUpdate(info, ecm);
  plugin.PostUpdate(info, ecm);

  const auto topics = plugin.ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, _expectedTopicName);
}

TEST(ContactTest, AbsoluteTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model" namespace="ns">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
              <topic>/test_sensor_topic</topic>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/test_sensor_topic");
}

TEST(ContactTest, RelativeTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model" namespace="ns">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
              <topic>test_sensor_topic</topic>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/test_sensor_topic");
}

TEST(ContactTest, DefaultTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model" namespace="ns">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "ns/contact");
}

TEST(ContactTest, TopicNamesWithoutNs)
{
  std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
              <topic>test_sensor_topic</topic>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "test_sensor_topic");

  sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString,
    "world/default/model/contact_model/link/link/sensor/"
    "contact_sensor/contact");
}

TEST(ContactTest, RemoveSensor)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model" namespace="ns">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity worldEntity;

  LoadWorldContext(sdfString, ecm, eventMgr, worldEntity);

  Contact plugin;

  UpdateInfo info;
  info.paused = true;
  plugin.PreUpdate(info, ecm);
  plugin.PostUpdate(info, ecm);

  auto topics = plugin.ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  Entity sensorEntity = topics.begin()->first;
  EXPECT_EQ(topics.begin()->second, "ns/contact");

  ecm.RequestRemoveEntity(sensorEntity);
  plugin.PostUpdate(info, ecm);
  ecm.ProcessRemoveEntityRequests();

  topics = plugin.ResolvedTopicNames();
  EXPECT_TRUE(topics.empty());
}

TEST(ContactTest, Reset)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <plugin
        filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
      </plugin>
      <model name="contact_model" namespace="ns">
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="contact_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity worldEntity;

  LoadWorldContext(sdfString, ecm, eventMgr, worldEntity);

  Contact plugin;

  UpdateInfo info;
  info.paused = true;
  plugin.PreUpdate(info, ecm);
  plugin.PostUpdate(info, ecm);

  auto topics = plugin.ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, "ns/contact");

  plugin.Reset(info, ecm);

  topics = plugin.ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, "ns/contact");
}
