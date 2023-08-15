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
#include <QtTest/QtTest>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Util.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <gz/utils/Subprocess.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/TestFixture.hh"
#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"
#include "TestHelper.hh"

#include "gz/sim/gui/Gui.hh"

int g_argc = 1;
char **g_argv;

double tol = 1e-1;

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test fixture for MouseDrag plugin
class MouseDragTestFixture : public InternalFixture<::testing::Test>
{
  protected: utils::Subprocess StartServer(const std::string &_sdfFile)
  {
    std::vector<std::string> cmd = {
      common::joinPaths(getInstallPrefix(), "bin", "gz"),
      "sim", "-s", "-r", "-v4", /* "--iterations=1000",  */_sdfFile};
    std::vector<std::string> env = {
      "LD_LIBRARY_PATH=" + common::joinPaths(getInstallPrefix(), "lib")};
    auto serverProc = utils::Subprocess(cmd, env);
    EXPECT_TRUE(serverProc.Alive());

    return serverProc;
  }

  protected: std::unique_ptr<gz::gui::Application> StartGUI()
  {
    auto app = gz::sim::gui::createGui(
      g_argc, g_argv, nullptr, nullptr, false, nullptr);
    app->AddPluginPath(common::joinPaths(PROJECT_BINARY_PATH, "lib"));
    gzdbg << "GUI created" << std::endl;
    EXPECT_NE(nullptr, app);

    // Subscribe to future ECM updates
    std::string stateTopic = transport::TopicUtils::AsValidTopic(
      "/world/default/state");
    EXPECT_FALSE(stateTopic.empty());
    EXPECT_TRUE(
      this->node.Subscribe(stateTopic, &MouseDragTestFixture::OnState, this));

    return app;
  }

  protected: void RequestState()
  {
    std::string stateAsyncTopic = transport::TopicUtils::AsValidTopic(
      "/world/default/state_async");
    EXPECT_FALSE(stateAsyncTopic.empty());
    std::string id = std::to_string(gz::gui::App()->applicationPid());
    std::string reqSrv = transport::TopicUtils::AsValidTopic(
      "/" + id + "/state_async");
    EXPECT_FALSE(reqSrv.empty());

    auto advertised = this->node.AdvertisedServices();
    if (std::find(advertised.begin(), advertised.end(), reqSrv) ==
        advertised.end())
    {
      EXPECT_TRUE(
        this->node.Advertise(reqSrv, &MouseDragTestFixture::OnState, this));
    }

    msgs::StringMsg req;
    req.set_data(reqSrv);
    unsigned int timeout = 500;
    msgs::Empty res;
    bool result{false};
    gzdbg << "Requesting " << stateAsyncTopic << std::endl;
    this->stateUpdated = false;
    this->node.Request(stateAsyncTopic, req, timeout, res, result);

    // Wait for initial state
    unsigned int sleep = 0;
    unsigned int maxSleep = 30;
    while (!this->stateUpdated && sleep < maxSleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++sleep;
    }
    EXPECT_LT(sleep, maxSleep);
  }

  protected: void OnState(const msgs::SerializedStepMap &_msg)
  {
    if (!this->stateUpdated)
      gzdbg << "State request answered" << std::endl;
    this->stateUpdated = true;
    this->ecm.SetState(_msg.state());
  }

  protected: transport::Node node;

  protected: EntityComponentManager ecm;

  protected: bool stateUpdated{false};
};

/////////////////////////////////////////////////
TEST_F(MouseDragTestFixture,
  GZ_UTILS_TEST_DISABLED_ON_WIN32(ApplyLinkWrenchLoaded))
{
  // Run server in headless mode
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "mouse_drag.sdf");
  auto serverProc = this->StartServer(sdfFile);

  // Request initial ECM state
  this->RequestState();

  // Get world entity
  Entity worldEntity;
  this->ecm.Each<components::World, components::Name>(
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
  gzdbg << "worldEntity " << worldEntity << std::endl;

  // Setup GUI
  auto app = this->StartGUI();
  EXPECT_TRUE(app->LoadPlugin("MouseDrag"));
  gzdbg << "MouseDrag plugin loaded" << std::endl;

  // Get main window
  auto win = app->findChild<gz::gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Show, but don't exec, so we don't block
  win->QuickWindow()->show();

  // Catch rendering thread
  rendering::ScenePtr scene;
  rendering::CameraPtr camera;
  std::optional<QEvent *> event;
  auto testHelper = std::make_unique<TestHelper>();
  testHelper->forwardEvent = [&](QEvent *_event)
  {
    if (_event->type() == gz::gui::events::Render::kType)
    {
      gzdbg << "Render" << std::endl;
      // Get scene and user camera
      if (!scene)
      {
        scene = rendering::sceneFromFirstRenderEngine();
        EXPECT_NE(nullptr, scene);
        for (unsigned int i = 0; i < scene->NodeCount(); ++i)
        {
          auto cam = std::dynamic_pointer_cast<rendering::Camera>(
            scene->NodeByIndex(i));
          if (cam && cam->HasUserData("user-camera") &&
              std::get<bool>(cam->UserData("user-camera")))
          {
            camera = cam;
            break;
          }
        }
        EXPECT_NE(nullptr, camera);
      }
      // Send events
      if (event)
      {
        app->sendEvent(app->findChild<gz::gui::MainWindow *>(), *event);
        event.reset();
      }
    }
  };

  // Wait for scene
  unsigned int sleep = 0;
  unsigned int maxSleep = 30;
  while (!scene && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    ++sleep;
  }
  EXPECT_LT(sleep, maxSleep);

  // Check for ApplyLinkWrench system
  sleep = 0;
  bool systemLoaded{false};
  while (!systemLoaded && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    gzdbg << "Searching for system" << std::endl;

    // Check if ApplyLinkWrench system is loaded
    if (auto msg =
        this->ecm.ComponentData<components::SystemPluginInfo>(worldEntity))
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
    else
    {
      this->RequestState();
    }
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
  gzdbg << "System loaded " << systemLoaded << std::endl;

  // Get box initial position
  Entity box = ecm.EntityByComponents(components::Model(),
                                      components::Name("box"));
  auto poseInitial = ecm.ComponentData<components::Pose>(box);
  EXPECT_TRUE(poseInitial.has_value());

  // Translation
  gzdbg << "Ctrl-Right Click" << std::endl;
  common::MouseEvent mouse;
  auto pos = camera->Project(poseInitial->Pos());
  mouse.SetPos(pos);
  mouse.SetControl(true);
  mouse.SetType(common::MouseEvent::PRESS);
  mouse.SetButton(common::MouseEvent::RIGHT);
  gz::gui::events::LeftClickOnScene leftClickEvent(mouse);
  event = &leftClickEvent;

  // Wait for event to be sent
  sleep = 0;
  while (event && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    ++sleep;
  }
  EXPECT_LT(sleep, maxSleep);

  gzdbg << "Drag upwards" << std::endl;
  mouse.SetPos(pos.X(), pos.Y()-100);
  mouse.SetType(common::MouseEvent::MOVE);
  gz::gui::events::DragOnScene dragEvent(mouse);
  event = &dragEvent;

  // Wait for event to be sent
  sleep = 0;
  while (event && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    ++sleep;
  }
  EXPECT_LT(sleep, maxSleep);

  sleep = 0;
  auto pose = ecm.ComponentData<components::Pose>(box);
  while (pose->Z() - poseInitial->Z() < tol && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    pose = ecm.ComponentData<components::Pose>(box);
    ++sleep;
  }
  EXPECT_LT(sleep, maxSleep);

  // Cleanup
  serverProc.Terminate();
  sleep = 0;
  while (serverProc.Alive() && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++sleep;
  }
  EXPECT_LT(sleep, maxSleep);
  gzdbg << "Server stdout:\n" << serverProc.Stdout() << std::endl;
  gzdbg << "Server stderr:\n" << serverProc.Stderr() << std::endl;

  win->QuickWindow()->close();
}
