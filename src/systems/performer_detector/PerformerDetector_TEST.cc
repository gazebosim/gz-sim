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

#include "PerformerDetector.hh"

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
/// \brief Build an SDF world with a PerformerDetector model plugin
/// \param[in] _ns Model namespace. Can be empty.
/// \param[in] _topic Detector topic. Can be empty.
/// \return SDF world string
std::string MakeSdf(const std::string &_ns, const std::string &_topic)
{
  const std::string ns = _ns.empty() ? "" : " namespace='" + _ns + "'";
  const std::string topic = _topic.empty() ? "" :
      "<topic>" + _topic + "</topic>";

  return R"(<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="performer_detector_test_world">
    <model name="test_model")" + ns + R"(>
      <link name="link"/>
      <plugin filename="gz-sim-performer-detector-system"
              name="gz::sim::systems::PerformerDetector">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        )" + topic + R"(
      </plugin>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
/// \brief Configure a PerformerDetector system and check resolved names
/// \param[in] _sdfString SDF world string
/// \param[in] _expected Expected resolved status topic
void TestTopicName(const std::string &_sdfString,
    const std::string &_expected)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity entity = kNullEntity;
  sdf::Plugin pluginSdf;

  test::LoadModelContext(_sdfString, ecm, eventMgr, entity, pluginSdf);

  PerformerDetector system;
  system.Configure(entity, pluginSdf.Element(), ecm, eventMgr);

  EXPECT_EQ(_expected, system.ResolvedTopicNames().statusTopic);
}

/////////////////////////////////////////////////
TEST(PerformerDetectorTest, AbsoluteTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", "/absolute/status"), "/absolute/status");
}

/////////////////////////////////////////////////
TEST(PerformerDetectorTest, RelativeTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", "test_status"), "ns/test_status");
}

/////////////////////////////////////////////////
TEST(PerformerDetectorTest, DefaultTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", ""), "ns/performer_detector/status");
}

/////////////////////////////////////////////////
TEST(PerformerDetectorTest, RelativeTopicWithoutNamespace)
{
  TestTopicName(MakeSdf("", "test_status"), "test_status");
}

/////////////////////////////////////////////////
TEST(PerformerDetectorTest, DefaultTopicWithoutNamespace)
{
  TestTopicName(MakeSdf("", ""),
      "/model/test_model/performer_detector/status");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
