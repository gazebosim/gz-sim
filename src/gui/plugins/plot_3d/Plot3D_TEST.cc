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

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/test_config.hh"

#include "../../GuiRunner.hh"
#include "Plot3D.hh"

int g_argc = 1;
char **g_argv;

using namespace ignition;

/// \brief Tests for the joint position controller GUI plugin
class Plot3D : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(Plot3D, IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Load))
{
  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle gazebo::gui plugins
  auto runner = new gazebo::GuiRunner("test");
  runner->connect(app.get(), &gui::Application::PluginAdded,
                  runner, &gazebo::GuiRunner::OnPluginAdded);
  runner->setParent(gui::App());

  // Add plugin
  const char *pluginStr =
    "<plugin filename=\"Plot3D\">"
      "<ignition-gui>"
        "<title>Plot3D!</title>"
      "</ignition-gui>"
      "<entity_name>banana</entity_name>"
      "<maximum_points>123</maximum_points>"
      "<minimum_distance>0.123</minimum_distance>"
      "<offset>1 2 3</offset>"
      "<color>0.1 0.2 0.3</color>"
    "</plugin>";

  tinyxml2::XMLDocument pluginDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS, pluginDoc.Parse(pluginStr));
  EXPECT_TRUE(app->LoadPlugin("Plot3D",
      pluginDoc.FirstChildElement("plugin")));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Get plugin
  auto plugins = win->findChildren<gazebo::gui::Plot3D *>();
  ASSERT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];
  EXPECT_EQ(plugin->Title(), "Plot3D!");
  EXPECT_EQ(plugin->TargetEntity(), gazebo::kNullEntity);
  EXPECT_EQ(plugin->TargetName(), QString("banana"))
      << plugin->TargetName().toStdString();
  EXPECT_EQ(plugin->Color(), QVector3D(0.1, 0.2, 0.3));
  EXPECT_EQ(plugin->Offset(), QVector3D(1, 2, 3));
  EXPECT_TRUE(plugin->Locked());

  // Cleanup
  plugins.clear();
}
