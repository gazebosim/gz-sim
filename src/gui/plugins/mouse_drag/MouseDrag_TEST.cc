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
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <gz/msgs/double.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <string>
#include <vector>
#include <algorithm>

#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

#include "../../GuiRunner.hh"

#include "MouseDrag.hh"

int g_argc = 1;
char **g_argv;

using namespace gz;

/// \brief Tests for the mouse drag GU  I plugin
class MouseDragGui : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(MouseDragGui, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Load))
{
  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Create GUI runner to handle sim::gui plugins
  auto runner = new sim::GuiRunner("default");
  runner->setParent(gui::App());

  // Add plugin
  EXPECT_TRUE(app->LoadPlugin("MouseDrag"));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);

  // Get plugin
  auto plugins = win->findChildren<
      sim::MouseDrag *>();
  EXPECT_EQ(plugins.size(), 1);

  auto plugin = plugins[0];

  EXPECT_EQ(plugin->Title(), "Mouse drag");

  transport::Node node;
  auto topic = transport::TopicUtils::AsValidTopic("/world/default/wrench");
  EXPECT_FALSE(topic.empty());

  std::vector<std::string> topics;
  node.TopicList(topics);
  EXPECT_NE(std::find(topics.begin(), topics.end(), topic), topics.end());

  // Cleanup
  plugins.clear();
}
