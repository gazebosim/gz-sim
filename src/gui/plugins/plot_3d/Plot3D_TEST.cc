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
#include <gz/msgs/double.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

#include "../../GuiRunner.hh"

#include "Plot3D.hh"

int g_argc = 1;
char* g_argv[] =
{
  reinterpret_cast<char*>(const_cast<char*>("dummy")),
};

using namespace gz;

/// \brief Tests for the joint position controller GUI plugin
class Plot3D : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(Plot3D, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Load))
{
  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle sim::gui plugins
  auto runner = new sim::GuiRunner("test");
  runner->setParent(gui::App());

  // Add plugin
  const char *pluginStr =
    "<plugin filename=\"Plot3D\">"
      "<gz-gui>"
        "<title>Plot3D!</title>"
      "</gz-gui>"
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
  auto plugins = win->findChildren<sim::gui::Plot3D *>();
  ASSERT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];
  EXPECT_EQ("Plot3D!", plugin->Title());
  EXPECT_EQ(sim::kNullEntity, plugin->TargetEntity());
  EXPECT_EQ(QString("banana"), plugin->TargetName())
      << plugin->TargetName().toStdString();
  EXPECT_EQ(QVector3D(0.1, 0.2, 0.3), plugin->Color());
  EXPECT_EQ(QVector3D(1, 2, 3), plugin->Offset());
  EXPECT_TRUE(plugin->Locked());

  // Cleanup
  plugins.clear();
}
