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

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

namespace
{
std::string BaseWorldSdf()
{
  return R"(
<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='default'>
  </world>
</sdf>)";
}

std::string LegacyDefaultPluginsWorldSdf()
{
  return R"(
<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='default'>
    <plugin filename='ignition-gazebo-physics-system'
            name='ignition::gazebo::systems::Physics'>
    </plugin>
    <plugin filename='ignition-gazebo-user-commands-system'
            name='ignition::gazebo::systems::UserCommands'>
    </plugin>
    <plugin filename='ignition-gazebo-scene-broadcaster-system'
            name='ignition::gazebo::systems::SceneBroadcaster'>
    </plugin>
  </world>
</sdf>)";
}
}

class LegacyDefaultPluginsTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(LegacyDefaultPluginsTest, DoesNotDuplicateDefaultSystems)
{
  ServerConfig baselineConfig;
  baselineConfig.SetSdfString(BaseWorldSdf());
  Server baselineServer(baselineConfig);
  EXPECT_TRUE(baselineServer.RunOnce(true));
  auto baselineCount = baselineServer.SystemCount();
  ASSERT_TRUE(baselineCount.has_value());

  ServerConfig legacyConfig;
  legacyConfig.SetSdfString(LegacyDefaultPluginsWorldSdf());
  Server legacyServer(legacyConfig);
  EXPECT_TRUE(legacyServer.RunOnce(true));
  auto legacyCount = legacyServer.SystemCount();
  ASSERT_TRUE(legacyCount.has_value());

  EXPECT_EQ(*baselineCount, *legacyCount);
}
