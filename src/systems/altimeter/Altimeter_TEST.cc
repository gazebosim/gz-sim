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

#include "Altimeter.hh"

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

/// \brief Test topic name resolution for Altimeter system
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopicNames The expected resolved topic names
void TestTopicName(const std::string &_sdfString,
      const std::string &_expectedTopicName)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);

  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);

  ASSERT_EQ(1u, worldSdf->ModelCount());
  const sdf::Model *modelSdf = worldSdf->ModelByIndex(0);
  ASSERT_NE(nullptr, modelSdf);

  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator entityCreator(ecm, eventMgr);

  Entity modelEntity =
    entityCreator.CreateEntitiesWithoutLoadingPlugins(modelSdf);
  ASSERT_NE(kNullEntity, modelEntity);

  auto plugin = std::make_unique<Altimeter>();
  UpdateInfo info;
  info.paused = true;
  plugin->PreUpdate(info, ecm);
  plugin->PostUpdate(info, ecm);

  const auto topics = plugin->ResolvedTopicNames();
  ASSERT_EQ(1u, topics.size());
  EXPECT_EQ(topics.begin()->second, _expectedTopicName);
}

TEST(AltimeterTest, AbsoluteTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="altimeter_model" namespace="ns">
        <link name="link">
          <sensor name="altimeter" type="altimeter">
            <topic>/test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/test_sensor_topic");
}

TEST(AltimeterTest, RelativeTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="altimeter_model" namespace="ns">
        <link name="link">
          <sensor name="altimeter" type="altimeter">
            <topic>test_sensor_topic</topic>
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/ns/test_sensor_topic");
}

TEST(AltimeterTest, DefaultTopicName)
{
  const std::string sdfString = R"(
  <sdf version="1.10">
    <world name="default">
      <model name="altimeter_model" namespace="ns">
        <link name="link">
          <sensor name="altimeter" type="altimeter">
          </sensor>
        </link>
      </model>
    </world>
  </sdf>)";

  TestTopicName(sdfString, "/ns/altimeter");
}