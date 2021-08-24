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
#include <sdf/sdf.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools.hh>
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class SdfInclude : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(SdfInclude, DownloadFromFuel)
{
  std::string path = common::cwd() + "/test_cache";

  // Configure the gazebo server, which will cause a model to be downloaded.
  gazebo::ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include.sdf");
  gazebo::Server server(serverConfig);

  EXPECT_TRUE(common::exists(path +
        "/fuel.ignitionrobotics.org/openrobotics/models/ground plane" +
        "/1/model.sdf"));
}
