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

#include "FreeSpaceExplorer.hh"

#include <string>

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Build an SDF world with a FreeSpaceExplorer model plugin
/// \param[in] _ns Model namespace. Can be empty.
/// \param[in] _lidarTopic LiDAR topic. Can be empty.
/// \param[in] _imageTopic Image topic. Can be empty.
/// \param[in] _startTopic Start topic. Can be empty.
/// \return SDF world string
std::string MakeSdf(const std::string &_ns, const std::string &_lidarTopic,
    const std::string &_imageTopic, const std::string &_startTopic)
{
  const std::string ns = _ns.empty() ? "" : " namespace='" + _ns + "'";
  const std::string lidar = _lidarTopic.empty() ? "" :
      "<lidar_topic>" + _lidarTopic + "</lidar_topic>";
  const std::string image = _imageTopic.empty() ? "" :
      "<image_topic>" + _imageTopic + "</image_topic>";
  const std::string start = _startTopic.empty() ? "" :
      "<start_topic>" + _startTopic + "</start_topic>";

  return R"(<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="free_space_explorer_test_world">
    <model name="test_model")" + ns + R"(>
      <link name="link"/>
      <plugin filename="gz-sim-free-space-explorer-system"
              name="gz::sim::systems::FreeSpaceExplorer">
        )" + lidar + image + start + R"(
      </plugin>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
/// \brief Configure a FreeSpaceExplorer system and check resolved names
/// \param[in] _sdfString SDF world string
/// \param[in] _expected Expected resolved topic names
void TestTopicNames(const std::string &_sdfString,
    const FreeSpaceExplorer::TopicNames &_expected)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity entity = kNullEntity;
  sdf::Plugin pluginSdf;

  test::LoadModelContext(_sdfString, ecm, eventMgr, entity, pluginSdf);

  FreeSpaceExplorer system;
  system.Configure(entity, pluginSdf.Element(), ecm, eventMgr);

  const auto names = system.ResolvedTopicNames();
  EXPECT_EQ(_expected.lidarTopic, names.lidarTopic);
  EXPECT_EQ(_expected.imageTopic, names.imageTopic);
  EXPECT_EQ(_expected.startTopic, names.startTopic);
}

/////////////////////////////////////////////////
TEST(FreeSpaceExplorerTest, AbsoluteTopicsWithNamespace)
{
  FreeSpaceExplorer::TopicNames expected;
  expected.lidarTopic = "/absolute/scan";
  expected.imageTopic = "/absolute/image";
  expected.startTopic = "/absolute/start";

  TestTopicNames(MakeSdf("ns", "/absolute/scan", "/absolute/image",
      "/absolute/start"), expected);
}

/////////////////////////////////////////////////
TEST(FreeSpaceExplorerTest, RelativeTopicsWithNamespace)
{
  FreeSpaceExplorer::TopicNames expected;
  expected.lidarTopic = "ns/test_scan";
  expected.imageTopic = "ns/test_image";
  expected.startTopic = "ns/test_start";

  TestTopicNames(MakeSdf("ns", "test_scan", "test_image", "test_start"),
      expected);
}

/////////////////////////////////////////////////
TEST(FreeSpaceExplorerTest, DefaultTopicsWithNamespace)
{
  FreeSpaceExplorer::TopicNames expected;
  expected.lidarTopic = "ns/scan";
  expected.imageTopic = "ns/scan_image";
  expected.startTopic = "ns/start";

  TestTopicNames(MakeSdf("ns", "", "", ""), expected);
}

/////////////////////////////////////////////////
TEST(FreeSpaceExplorerTest, RelativeTopicsWithoutNamespace)
{
  FreeSpaceExplorer::TopicNames expected;
  expected.lidarTopic = "test_scan";
  expected.imageTopic = "test_image";
  expected.startTopic = "test_start";

  TestTopicNames(MakeSdf("", "test_scan", "test_image", "test_start"),
      expected);
}

/////////////////////////////////////////////////
TEST(FreeSpaceExplorerTest, DefaultTopicsWithoutNamespace)
{
  FreeSpaceExplorer::TopicNames expected;
  expected.lidarTopic = "scan";
  expected.imageTopic = "scan_image";
  expected.startTopic = "start";

  TestTopicNames(MakeSdf("", "", "", ""), expected);
}
