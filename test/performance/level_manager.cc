/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <array>

#include <ignition/math/Stopwatch.hh>
#include <ignition/common/Console.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;
using namespace gazebo;

TEST(LevelManagerPerfrormance, LevelVsNoLevel)
{
  using namespace std::chrono;

  common::Console::SetVerbosity(4);

  setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
         (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);

  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/level_performance.sdf");
  math::Stopwatch watch;

  const std::size_t iters = 5000;

  // Server with levels
  {
    serverConfig.SetUseLevels(true);
    gazebo::Server server(serverConfig);
    server.SetUpdatePeriod(1ns);

    watch.Start(true);
    server.Run(true, iters, false);
    watch.Stop();
  }
  const auto levelsDuration = watch.ElapsedRunTime();

  // Server without levels
  {
    serverConfig.SetUseLevels(false);
    gazebo::Server serverNoLevels(serverConfig);
    serverNoLevels.SetUpdatePeriod(1ns);

    watch.Start(true);
    serverNoLevels.Run(true, iters, false);
    watch.Stop();
  }
  const auto nolevelsDuration = watch.ElapsedRunTime();

  igndbg << "\nUsing levels = "
         << duration_cast<milliseconds>(levelsDuration).count() << " ms\n"
         << "Without levels = "
         << duration_cast<milliseconds>(nolevelsDuration).count() << " ms\n";

  EXPECT_LE(levelsDuration.count(), nolevelsDuration.count());
}
