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

#include "Breadcrumbs.hh"

#include <gtest/gtest.h>

#include <sdf/Plugin.hh>

#include "gz/sim/EventManager.hh"
#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace test;

/// \brief Test topic name resolution for the Breadcrumbs system.
/// \param[in] _namespace Optional model namespace.
/// \param[in] _topic Optional custom deploy topic.
/// \param[in] _expectedTopic Expected resolved deploy topic.
void TestTopicNames(const std::string &_namespace,
    const std::string &_topic, const std::string &_expectedTopic)
{
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='breadcrumbs_test_world'>
        <model name='carrier' namespace=')" + _namespace + R"('>
          <link name='link'/>
          <plugin name='gz::sim::systems::Breadcrumbs'
                  filename='gz-sim-breadcrumbs-system'>
            )" + _topic + R"(
            <breadcrumb>
              <sdf version='1.10'>
                <model name='marker'>
                  <link name='link'/>
                </model>
              </sdf>
            </breadcrumb>
          </plugin>
        </model>
      </world>
    </sdf>)";

  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity modelEntity;
  sdf::Plugin pluginSdf;
  LoadModelContext(sdfString, ecm, eventMgr, modelEntity, pluginSdf);

  Breadcrumbs plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  EXPECT_EQ(_expectedTopic, plugin.topic);
  EXPECT_EQ(_expectedTopic + "/remaining", plugin.remainingTopic);
  EXPECT_TRUE(plugin.remainingPub);
}

TEST(BreadcrumbsTest, DefaultTopicWithNamespace)
{
  TestTopicNames("robot", "",
      "robot/breadcrumbs/marker/deploy");
}

TEST(BreadcrumbsTest, RelativeTopicWithNamespace)
{
  TestTopicNames("robot", "<topic>deploy_marker</topic>",
      "robot/deploy_marker");
}

TEST(BreadcrumbsTest, AbsoluteTopicWithNamespace)
{
  TestTopicNames("robot", "<topic>/deploy_marker</topic>",
      "/deploy_marker");
}

TEST(BreadcrumbsTest, TopicNamesWithoutNamespace)
{
  TestTopicNames("", "",
      "/model/carrier/breadcrumbs/marker/deploy");
  TestTopicNames("", "<topic>deploy_marker</topic>",
      "deploy_marker");
}
