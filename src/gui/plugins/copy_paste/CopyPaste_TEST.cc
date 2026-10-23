/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <memory>
#include <string>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"

#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

#include "CopyPaste.hh"

int g_argc = 1;
char* g_argv[] =
{
  reinterpret_cast<char*>(const_cast<char*>("dummy")),
};

using namespace gz;

/// \brief Records the entity names of the SpawnCloneFromName events that
/// CopyPaste::OnPaste sends to the main window.
class SpawnCloneRecorder : public QObject
{
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (_event->type() == gui::events::SpawnCloneFromName::kType)
    {
      this->names.push_back(
          static_cast<gui::events::SpawnCloneFromName *>(_event)->Name());
    }
    return QObject::eventFilter(_obj, _event);
  }

  public: std::vector<std::string> names;
};

/// \brief Tests for the copy/paste GUI plugin
class CopyPasteTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Qt Quick posts private events numbered from QEvent::User + 1 to the
// QuickWindow (FullUpdateRequest is QEvent::User + 1), the same type as
// EntitiesSelected. The plugin filters the QuickWindow for key presses, so it
// must only take the selection from the main window, where it is sent.
TEST_F(CopyPasteTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(SelectionOnlyFromMainWindow))
{
  sim::EntityComponentManager ecm;
  auto box = ecm.CreateEntity();
  ecm.CreateComponent(box, sim::components::Name("box"));
  auto sphere = ecm.CreateEntity();
  ecm.CreateComponent(sphere, sim::components::Name("sphere"));

  // Create app
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  // Add plugin
  EXPECT_TRUE(app->LoadPlugin("CopyPaste"));

  // Get main window
  auto win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);
  ASSERT_NE(nullptr, win->QuickWindow());

  // Get plugin
  auto plugins = win->findChildren<sim::CopyPaste *>();
  ASSERT_EQ(plugins.size(), 1);
  auto plugin = plugins[0];

  SpawnCloneRecorder recorder;
  win->installEventFilter(&recorder);

  sim::gui::events::EntitiesSelected selectBox({box});
  app->sendEvent(win, &selectBox);

  // Stands in for a Qt Quick private event of the same type
  sim::gui::events::EntitiesSelected onQuickWindow({sphere});
  app->sendEvent(win->QuickWindow(), &onQuickWindow);

  plugin->Update(sim::UpdateInfo(), ecm);
  plugin->OnCopy();
  plugin->OnPaste();

  ASSERT_EQ(1u, recorder.names.size());
  EXPECT_EQ("box", recorder.names[0]);

  // Cleanup
  win->removeEventFilter(&recorder);
  plugins.clear();
}
