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
#include <exception>
#include <ignition/common/Filesystem.hh>

#include "helpers/EnvTestFixture.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/gui/Gui.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;

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
  gazebo::ServerConfig config;
  config.SetSdfFile(_fileName);

  gazebo::Server server(config);
  EXPECT_TRUE(server.Run(true, 1, true));
}

/////////////////////////////////////////////////
/// \brief Start the GUI.
void startGui()
{
  int argc = 1;
  char *argv = const_cast<char *>("ign-gazebo-gui");
  EXPECT_EQ(0, gazebo::gui::runGui(argc, &argv, "", ""));
}

/////////////////////////////////////////////////
/// \brief Start both server and GUI.
/// \param[in] _fileName Full path to the SDFormat file to load.
void startBoth(const std::string &_fileName)
{
  std::thread serverThread([&]() { startServer(_fileName); });
  std::thread guiThread(startGui);
  sleep(4);
  std::raise(SIGTERM);
  serverThread.join();
  guiThread.join();
  std::exit(0);
}

/////////////////////////////////////////////////
TEST_P(GazeboDeathTest, CleanExit)
{
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
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}
