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

#include <gz/math/Stopwatch.hh>
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"  // NOLINT(build/include)

using namespace gz;
using namespace sim;

// See https://github.com/gazebosim/gz-sim/issues/1175
TEST(LevelManagerPerfrormance, GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelVsNoLevel))
{
  using namespace std::chrono;

  common::Console::SetVerbosity(4);

  common::setenv("GZ_SIM_SYSTEM_PLUGIN_PATH",
         (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/level_performance.sdf");
  math::Stopwatch watch;

  const std::size_t iters = 5000;


  // Reduce potential startup costs by running the server once before
  // measuring time differences between levels and no levels.
  {
    serverConfig.SetUseLevels(true);
    Server server(serverConfig);
    server.SetUpdatePeriod(1ns);

    server.Run(true, 1, false);
  }

  // Server with levels
  {
    serverConfig.SetUseLevels(true);
    Server server(serverConfig);
    server.SetUpdatePeriod(1ns);

    watch.Start(true);
    server.Run(true, iters, false);
    watch.Stop();
  }
  const auto levelsDuration = watch.ElapsedRunTime();

  // Server without levels
  {
    serverConfig.SetUseLevels(false);
    Server serverNoLevels(serverConfig);
    serverNoLevels.SetUpdatePeriod(1ns);

    watch.Start(true);
    serverNoLevels.Run(true, iters, false);
    watch.Stop();
  }
  const auto nolevelsDuration = watch.ElapsedRunTime();

  gzdbg << "\nUsing levels = "
         << duration_cast<milliseconds>(levelsDuration).count() << " ms\n"
         << "Without levels = "
         << duration_cast<milliseconds>(nolevelsDuration).count() << " ms\n";

  EXPECT_LE(levelsDuration.count(), nolevelsDuration.count());
}
