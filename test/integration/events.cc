/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "plugins/EventTriggerSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class EventTrigger : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(EventTrigger, TriggerPause)
{
  // Create server
  ServerConfig config;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/event_trigger.sdf";
  config.SetSdfFile(sdfFile);

  Server server(config);

  // Check it starts paused
  EXPECT_TRUE(*server.Paused());

  // Run 1 iteration and check that it's now unpaused
  server.Run(true, 1, true);
  EXPECT_FALSE(*server.Paused());
}

