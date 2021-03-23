/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <ignition/msgs/double.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>
#include <ignition/utils/SuppressWarning.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/test_config.hh"

#include "JointPositionController.hh"

int g_argc = 1;
char **g_argv;

using namespace ignition;

/// \brief Tests for the joint position controller GUI plugin
class JointPositionControllerGui : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(JointPositionControllerGui, IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Load))
{
  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle gazebo::gui plugins
  // TODO(anyone) Remove deprecation guard once GuiRunner becomes private
  IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  auto runner = new gazebo::GuiRunner("test");
  IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  runner->connect(app.get(), &gui::Application::PluginAdded,
                  runner, &gazebo::GuiRunner::OnPluginAdded);
  runner->setParent(gui::App());

  // Add plugin
  EXPECT_TRUE(app->LoadPlugin("JointPositionController"));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Get plugin
  auto plugins = win->findChildren<
      gazebo::gui::JointPositionController *>();
  EXPECT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];
  EXPECT_EQ(plugin->Title(), "Joint position controller");
  EXPECT_EQ(plugin->ModelEntity(), gazebo::kNullEntity);
  EXPECT_EQ(plugin->ModelName(), QString("No model selected"))
      << plugin->ModelName().toStdString();
  EXPECT_FALSE(plugin->Locked());

  // Cleanup
  plugins.clear();
}

/////////////////////////////////////////////////
TEST_F(JointPositionControllerGui,
    IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PublishCommand))
{
  // Create a model with a joint
  gazebo::EntityComponentManager ecm;

  auto modelEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelEntity, gazebo::components::Model());
  ecm.CreateComponent(modelEntity, gazebo::components::Name("model_name"));

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gazebo::components::Joint());
  ecm.CreateComponent(jointEntity, gazebo::components::Name("joint_name"));
  ecm.CreateComponent(jointEntity, gazebo::components::ParentEntity(
      modelEntity));
  ecm.CreateComponent(jointEntity, gazebo::components::JointPosition({0.1}));
  ecm.CreateComponent(jointEntity, gazebo::components::JointType(
      sdf::JointType::REVOLUTE));
  sdf::JointAxis jointAxis;
  jointAxis.SetLower(-1.0);
  jointAxis.SetUpper(1.0);
  ecm.CreateComponent(jointEntity, gazebo::components::JointAxis(jointAxis));

  // Populate state message
  msgs::SerializedStepMap stepMsg;
  ecm.State(*stepMsg.mutable_state());

  // Setup state services
  bool stateAsyncCalled{false};
  std::function<void(const msgs::StringMsg &)> stateAsyncCb =
      [&stateAsyncCalled, &stepMsg](const msgs::StringMsg &_req)
      {
        transport::Node node;
        node.Request(_req.data(), stepMsg);

        stateAsyncCalled = true;
      };
  transport::Node node;
  node.Advertise("/world/test/state_async", stateAsyncCb);

  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle gazebo::gui plugins
  // TODO(anyone) Remove deprecation guard once GuiRunner becomes private
  IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  auto runner = new gazebo::GuiRunner("test");
  IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  runner->connect(app.get(), &gui::Application::PluginAdded,
                  runner, &gazebo::GuiRunner::OnPluginAdded);
  runner->setParent(gui::App());

  // Load plugin
  const char *pluginStr =
    "<plugin filename=\"JointPositionController\">"
      "<ignition-gui>"
        "<title>JointPositionController!</title>"
      "</ignition-gui>"
    "</plugin>";

  tinyxml2::XMLDocument pluginDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS, pluginDoc.Parse(pluginStr));
  EXPECT_TRUE(app->LoadPlugin("JointPositionController",
      pluginDoc.FirstChildElement("plugin")));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Show, but don't exec, so we don't block
  win->QuickWindow()->show();

  // Get plugin
  auto plugins = win->findChildren<
      gazebo::gui::JointPositionController *>();
  EXPECT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];
  EXPECT_EQ(plugin->Title(), "JointPositionController!");

  EXPECT_EQ(plugin->ModelEntity(), gazebo::kNullEntity);
  EXPECT_EQ(plugin->ModelName(), QString("No model selected"))
      << plugin->ModelName().toStdString();
  EXPECT_FALSE(plugin->Locked());

  // Get model
  auto models = win->findChildren<gazebo::gui::JointsModel *>();
  EXPECT_EQ(models.size(), 1);

  auto jointsModel = models[0];
  EXPECT_EQ(0, jointsModel->rowCount());

  // Select model
  plugin->SetModelEntity(1);

  // Request state again, do it in separate thread so we can call processEvents
  std::thread waiting([&]()
  {
    runner->RequestState();
  });

  int sleep = 0;
  int maxSleep = 30;
  while (plugin->ModelName() != "model_name" && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    QCoreApplication::processEvents();
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
  waiting.join();

  EXPECT_TRUE(stateAsyncCalled);
  EXPECT_EQ(plugin->ModelEntity(), 1u);
  EXPECT_EQ(plugin->ModelName(), "model_name")
      << plugin->ModelName().toStdString();
  EXPECT_FALSE(plugin->Locked());
  EXPECT_EQ(1, jointsModel->rowCount());

  // Cleanup
  plugins.clear();
}

