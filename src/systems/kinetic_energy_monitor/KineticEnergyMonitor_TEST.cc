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

#include "KineticEnergyMonitor.hh"

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
/// \brief Build an SDF world with a KineticEnergyMonitor model plugin
/// \param[in] _ns Model namespace. Can be empty.
/// \param[in] _topic Kinetic energy topic. Can be empty.
/// \return SDF world string
std::string MakeSdf(const std::string &_ns, const std::string &_topic)
{
  const std::string ns = _ns.empty() ? "" : " namespace='" + _ns + "'";
  const std::string topic = _topic.empty() ? "" :
      "<topic>" + _topic + "</topic>";

  return R"(<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="kinetic_energy_monitor_test_world">
    <model name="test_model")" + ns + R"(>
      <link name="test_link"/>
      <plugin filename="gz-sim-kinetic-energy-monitor-system"
              name="gz::sim::systems::KineticEnergyMonitor">
        <link_name>test_link</link_name>
        <kinetic_energy_threshold>100</kinetic_energy_threshold>
        )" + topic + R"(
      </plugin>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
/// \brief Configure a KineticEnergyMonitor system and check resolved names
/// \param[in] _sdfString SDF world string
/// \param[in] _expected Expected resolved energy topic
void TestTopicName(const std::string &_sdfString,
    const std::string &_expected)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity entity = kNullEntity;
  sdf::Plugin pluginSdf;

  test::LoadModelContext(_sdfString, ecm, eventMgr, entity, pluginSdf);

  KineticEnergyMonitor system;
  system.Configure(entity, pluginSdf.Element(), ecm, eventMgr);

  EXPECT_EQ(_expected, system.ResolvedTopicNames().energyTopic);
}

/////////////////////////////////////////////////
TEST(KineticEnergyMonitorTest, AbsoluteTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", "/absolute/energy"), "/absolute/energy");
}

/////////////////////////////////////////////////
TEST(KineticEnergyMonitorTest, RelativeTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", "test_energy"), "ns/test_energy");
}

/////////////////////////////////////////////////
TEST(KineticEnergyMonitorTest, DefaultTopicWithNamespace)
{
  TestTopicName(MakeSdf("ns", ""), "ns/kinetic_energy");
}

/////////////////////////////////////////////////
TEST(KineticEnergyMonitorTest, RelativeTopicWithoutNamespace)
{
  TestTopicName(MakeSdf("", "test_energy"), "test_energy");
}

/////////////////////////////////////////////////
TEST(KineticEnergyMonitorTest, DefaultTopicWithoutNamespace)
{
  TestTopicName(MakeSdf("", ""), "/model/test_model/kinetic_energy");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
