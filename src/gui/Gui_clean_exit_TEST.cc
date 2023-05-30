/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <csignal>
#include <chrono>
#include <exception>
#include <gz/common/Filesystem.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "helpers/EnvTestFixture.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/gui/Gui.hh"
#include "test_config.hh"  // NOLINT(build/include)

using namespace gz;

/////////////////////////////////////////////////
class GazeboDeathTest
    : public InternalFixture<::testing::TestWithParam<const char *>>
{
};

/////////////////////////////////////////////////
/// \brief Start the server.
/// \param[in] _fileName Full path to the SDFormat file to load.
void startServer(const std::string &_fileName)
{
  sim::ServerConfig config;
  config.SetSdfFile(_fileName);

  sim::Server server(config);
  EXPECT_TRUE(server.Run(true, 1, true));
}

/////////////////////////////////////////////////
/// \brief Start the GUI.
void startGui()
{
  int argc = 1;
  char *argv = const_cast<char *>("gz-sim-gui");
  EXPECT_EQ(0, sim::gui::runGui(argc, &argv, nullptr, nullptr, 0,
      nullptr));
}

/////////////////////////////////////////////////
/// \brief Start both server and GUI.
/// \param[in] _fileName Full path to the SDFormat file to load.
void startBoth(const std::string &_fileName)
{
  std::thread serverThread([&]() { startServer(_fileName); });
  std::thread guiThread(startGui);
  // Sleep long enough for every system to be loaded and initialized. Sending a
  // SIGTERM during initialization doesn't always work properly and that is not
  // what we are testing here.
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(15s);
  std::raise(SIGTERM);
  serverThread.join();
  guiThread.join();
  std::exit(0);
}

/////////////////////////////////////////////////
TEST_P(GazeboDeathTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(CleanExit))
{
  std::string githubAction;
  // This test hangs when there is high CPU usage, so we skip it on Github
  // Actions.
  // Note: The GITHUB_ACTIONS environment variable is automatically set when
  // running on Github Actions. See https://docs.github.com/en/actions/learn-github-actions/environment-variables#default-environment-variables
  if (common::env("GITHUB_ACTIONS", githubAction))
  {
    GTEST_SKIP();
  }

  const auto sdfFile =
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", GetParam());
  ASSERT_TRUE(common::exists(sdfFile))
      << "File [" << sdfFile << "] does not exist";

  EXPECT_EXIT(startBoth(sdfFile), testing::ExitedWithCode(0), ".*");
}

// Test various world files to see if any combination of systems could cause a
// crash. Note, however, that the files must not contain the Sensor system as it
// is currently not possible to run two instances of Ogre2 in one process.
INSTANTIATE_TEST_SUITE_P(WorldFiles, GazeboDeathTest,
                         ::testing::Values("empty.sdf", "shapes.sdf"));

int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::FLAGS_gtest_death_test_style = "fast";
  return RUN_ALL_TESTS();
}
