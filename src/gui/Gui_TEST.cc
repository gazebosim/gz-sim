/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/gui/Gui.hh"
#include "gz/sim/test_config.hh"

#include "../../test/helpers/EnvTestFixture.hh"

int gg_argc = 1;
char **gg_argv = new char *[gg_argc];

using namespace gz;
using namespace sim;

class GuiTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// https://github.com/gazebosim/gz-sim/issues/8
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(GuiTest, IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PathManager))
{
  common::Console::SetVerbosity(4);
  igndbg << "Start test" << std::endl;

  gz::common::setenv("GZ_SIM_RESOURCE_PATH",
         "/from_env:/tmp/more_env");
  gz::common::setenv("SDF_PATH", "");
  gz::common::setenv("IGN_FILE_PATH", "");
  igndbg << "Environment set" << std::endl;

  transport::Node node;

  // Worlds callback
  bool worldsCalled{false};
  std::function<bool(msgs::StringMsg_V &)> worldsCb =
      [&worldsCalled](msgs::StringMsg_V &_res)
      {
        _res.add_data("world_name");
        worldsCalled = true;
        return true;
      };
  node.Advertise("/gazebo/worlds", worldsCb);
  igndbg << "Worlds advertised" << std::endl;

  // GUI info callback
  bool guiInfoCalled{false};
  std::function<bool(msgs::GUI &)> guiInfoCb =
      [&guiInfoCalled](msgs::GUI &)
      {
        guiInfoCalled = true;
        return true;
      };
  node.Advertise("/world/world_name/gui/info", guiInfoCb);
  igndbg << "GUI info advertised" << std::endl;

  // Resource paths callback
  bool pathsCalled{false};
  std::function<bool(msgs::StringMsg_V &)> pathsCb =
      [&pathsCalled](msgs::StringMsg_V &_res)
      {
        _res.add_data("/from_callback");
        pathsCalled = true;
        return true;
      };
  node.Advertise("/gazebo/resource_paths/get", pathsCb);
  igndbg << "Paths advertised" << std::endl;

  auto app = gz::sim::gui::createGui(
    gg_argc, gg_argv, nullptr, nullptr, false, nullptr);
  EXPECT_NE(nullptr, app);
  igndbg << "GUI created" << std::endl;

  EXPECT_TRUE(worldsCalled);
  EXPECT_TRUE(guiInfoCalled);
  EXPECT_TRUE(pathsCalled);

  // Check paths
  for (auto env : {"GZ_SIM_RESOURCE_PATH", "SDF_PATH", "IGN_FILE_PATH"})
  {
    igndbg << "Checking variable [" << env << "]" << std::endl;
    char *pathCStr = std::getenv(env);

    auto paths = common::Split(pathCStr, ':');
    paths.erase(std::remove_if(paths.begin(), paths.end(),
        [](std::string const &_path)
        {
          return _path.empty();
        }),
        paths.end());

    ASSERT_EQ(3u, paths.size());
    EXPECT_EQ("/from_env", paths[0]);
    EXPECT_EQ("/tmp/more_env", paths[1]);
    EXPECT_EQ("/from_callback", paths[2]);
  }

  // Create a subscriber just so we can check when the message has propagated
  bool topicCalled{false};
  std::function<void(const msgs::StringMsg_V &)> topicCb =
      [&topicCalled](const msgs::StringMsg_V &)
      {
        topicCalled = true;
      };
  node.Subscribe("/gazebo/resource_paths", topicCb);
  igndbg << "Paths subscribed" << std::endl;

  // Notify new path through a topic
  msgs::StringMsg_V msg;
  msg.add_data("/new/path");

  auto pathPub = node.Advertise<msgs::StringMsg_V>("/gazebo/resource_paths");
  pathPub.Publish(msg);

  int sleep{0};
  int maxSleep{30};
  while (!topicCalled && sleep < maxSleep)
  {
    IGN_SLEEP_MS(100);
    sleep++;
  }
  EXPECT_TRUE(topicCalled);

  // Check paths
  for (auto env : {"GZ_SIM_RESOURCE_PATH", "SDF_PATH", "IGN_FILE_PATH"})
  {
    igndbg << "Checking variable [" << env << "]" << std::endl;
    char *pathCStr = std::getenv(env);

    auto paths = common::Split(pathCStr, ':');
    paths.erase(std::remove_if(paths.begin(), paths.end(),
        [](std::string const &_path)
        {
          return _path.empty();
        }),
        paths.end());

    ASSERT_EQ(4u, paths.size());
    EXPECT_EQ("/from_env", paths[0]);
    EXPECT_EQ("/tmp/more_env", paths[1]);
    EXPECT_EQ("/from_callback", paths[2]);
    EXPECT_EQ("/new/path", paths[3]);
  }
}

