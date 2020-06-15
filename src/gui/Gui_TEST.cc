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
#include <ignition/common/Console.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/gui/Gui.hh"
#include "ignition/gazebo/test_config.hh"

int g_argc = 1;
char **g_argv = new char *[g_argc];

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TEST(GuiTest, ResourcePath)
{
  common::Console::SetVerbosity(4);

  setenv("IGN_GAZEBO_RESOURCE_PATH",
         "/from_env:/tmp/more_env", 1);
  setenv("SDF_PATH", "", 1);
  setenv("IGN_FILE_PATH", "", 1);

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

  // GUI info callback
  bool guiInfoCalled{false};
  std::function<bool(msgs::GUI &)> guiInfoCb =
      [&guiInfoCalled](msgs::GUI &)
      {
        guiInfoCalled = true;
        return true;
      };
  node.Advertise("/world/world_name/gui/info", guiInfoCb);

  // Resource paths callback
  bool pathsCalled{false};
  std::function<bool(msgs::StringMsg_V &)> pathsCb =
      [&pathsCalled](msgs::StringMsg_V &_res)
      {
        _res.add_data("/from_callback");
        pathsCalled = true;
        return true;
      };
  node.Advertise("/gazebo/get_resource_paths", pathsCb);

  auto app = ignition::gazebo::gui::createGui(g_argc, g_argv, nullptr);
  EXPECT_NE(nullptr, app);

  EXPECT_TRUE(worldsCalled);
  EXPECT_TRUE(guiInfoCalled);
  EXPECT_TRUE(pathsCalled);

  // Check paths
  for (auto env : {"IGN_GAZEBO_RESOURCE_PATH", "SDF_PATH", "IGN_FILE_PATH"})
  {
    char *pathCStr = getenv(env);

    auto paths = common::Split(pathCStr, ':');
    paths.erase(std::remove_if(paths.begin(), paths.end(),
        [](std::string const &_path)
        {
          return _path.empty();
        }),
        paths.end());

    EXPECT_EQ(3u, paths.size());
    EXPECT_EQ("/from_env", paths[0]);
    EXPECT_EQ("/tmp/more_env", paths[1]);
    EXPECT_EQ("/from_callback", paths[2]);
  }
}

