/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <cstdio>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Util.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <gz/utils/Subprocess.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/TestFixture.hh"
#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

#include "gz/sim/gui/Gui.hh"

int g_argc = 1;
char **g_argv;

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test fixture for MouseDrag plugin
class MouseDragTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(MouseDragTestFixture,
  GZ_UTILS_TEST_DISABLED_ON_WIN32(ApplyLinkWrenchLoaded))
{
  // Run server in headless mode
  const std::string kGzCommand =
    common::joinPaths(getInstallPrefix(), "bin", "gz");
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "shapes.sdf");
  std::vector<std::string> env = {
    "LD_LIBRARY_PATH=" + common::joinPaths(getInstallPrefix(), "lib"),
    "HOME=/home/henrique",
  };
  auto serverProc = utils::Subprocess(
    {kGzCommand, "sim", "-s", "-r", "-v4", sdfFile}, env);
  EXPECT_TRUE(serverProc.Alive());
  gzdbg << "Server alive " << serverProc.Alive() << std::endl;

  // Create app
  auto app = gz::sim::gui::createGui(
    g_argc, g_argv, nullptr, nullptr, false, nullptr);
  gzmsg << "GUI created" << std::endl;
  ASSERT_NE(nullptr, app);

  // Add plugin
  app->AddPluginPath(common::joinPaths(PROJECT_BINARY_PATH, "lib"));
  EXPECT_TRUE(app->LoadPlugin("MouseDrag"));
  gzmsg << "MouseDrag plugin loaded" << std::endl;

  // Setup ECM state callback
  EntityComponentManager ecm;
  std::function<void(const msgs::SerializedStepMap &)> cb =
    [&](const msgs::SerializedStepMap &_msg)
    {
      // gzdbg << "Get state" << std::endl;
      ecm.SetState(_msg.state());
    };
  std::string id = std::to_string(gz::gui::App()->applicationPid());
  transport::Node node;
  std::string reqSrv = transport::TopicUtils::AsValidTopic(
    common::joinPaths(node.Options().NameSpace(), id, "state_async"));
  EXPECT_FALSE(reqSrv.empty());
  EXPECT_TRUE(node.Advertise(reqSrv, cb));
  gzmsg << "Callback on " << reqSrv << std::endl;

  // Wait for state_async service
  std::string stateAsyncTopic = transport::TopicUtils::AsValidTopic(
    "/world/default/state_async");
  EXPECT_FALSE(stateAsyncTopic.empty());
  std::vector<std::string> services;
  node.ServiceList(services);
  while (std::find(services.begin(), services.end(), stateAsyncTopic) ==
        services.end())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    node.ServiceList(services);
  }

  // Request initial ECM state
  msgs::StringMsg req;
  req.set_data(reqSrv);
  unsigned int timeout = 100;
  msgs::Empty res;
  bool result{false};
  gzmsg << "Requesting " << stateAsyncTopic << std::endl;
  node.Request(stateAsyncTopic, req, timeout, res, result);

  // Subscribe to future ECM updates
  std::string stateTopic = transport::TopicUtils::AsValidTopic(
    "/world/default/state");
  EXPECT_FALSE(stateTopic.empty());
  EXPECT_TRUE(node.Subscribe(stateTopic, cb));

  // Get world entity
  Entity worldEntity;
  ecm.Each<components::World, components::Name>(
    [&](const Entity &_entity,
      const components::World */*_world*/,
      const components::Name *_name)->bool
    {
      if (_name->Data() == "default")
      {
        worldEntity = _entity;
        return false;
      }
      return true;
    });
  gzmsg << "worldEntity " << worldEntity << std::endl;

  // Check for ApplyLinkWrench system
  int sleep = 0;
  int maxSleep = 30;
  bool systemLoaded{false};
  while (!systemLoaded && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();

    // Check if ApplyLinkWrench system is loaded
    if (auto msg = ecm.ComponentData<components::SystemPluginInfo>(worldEntity))
    {
      for (const auto &plugin : msg->plugins())
      {
        if (plugin.filename() == "gz-sim-apply-link-wrench-system")
        {
          systemLoaded = true;
          break;
        }
      }
    }

    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
  gzmsg << "System loaded " << systemLoaded << std::endl;

  // Cleanup
  serverProc.Terminate();
  gzdbg << "Server stdout:\n" << serverProc.Stdout() << std::endl;
  gzdbg << "Server stderr:\n" << serverProc.Stderr() << std::endl;
}
