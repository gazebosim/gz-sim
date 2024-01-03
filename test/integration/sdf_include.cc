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
#include <gz/common/Filesystem.hh>
#include <gz/fuel_tools.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class SdfInclude : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(SdfInclude, GZ_UTILS_TEST_DISABLED_ON_WIN32(DownloadFromFuel))
{
  std::string path = common::cwd() + "/test_cache";

  // Configure the gazebo server, which will cause a model to be downloaded.
  ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include.sdf");
  Server server(serverConfig);

  EXPECT_TRUE(common::exists(path +
        "/fuel.gazebosim.org/openrobotics/models/ground plane" +
        "/1/model.sdf"));
}
