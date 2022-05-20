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

#include <gz/common/Console.hh>
#include <gz/common/SignalHandler.hh>
#include <gz/common/Filesystem.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>

#include "gz/sim/config.hh"
#include "gz/sim/gui/Gui.hh"

#include "AboutDialogHandler.hh"
#include "GuiFileHandler.hh"
#include "GuiRunner.hh"
#include "PathManager.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_GAZEBO_VERSION_NAMESPACE {
namespace gui
{
//////////////////////////////////////////////////
std::unique_ptr<gz::gui::Application> createGui(
    int &_argc, char **_argv, const char *_guiConfig,
    const char *_defaultGuiConfig, bool _loadPluginsFromSdf,
    const char *_renderEngine)
{
  gz::common::SignalHandler sigHandler;
  bool sigKilled = false;
  sigHandler.AddCallback([&](const int /*_sig*/)
  {
    sigKilled = true;
  });

  ignmsg << "Ignition Gazebo GUI    v" << GZ_GAZEBO_VERSION_FULL
         << std::endl;

  // Set auto scaling factor for HiDPI displays
  if (QString::fromLocal8Bit(qgetenv("QT_AUTO_SCREEN_SCALE_FACTOR")).isEmpty())
  {
    qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
  }

  // Initialize Qt app
  auto app = std::make_unique<gz::gui::Application>(_argc, _argv);
  app->AddPluginPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  auto aboutDialogHandler = new gz::sim::gui::AboutDialogHandler();
  aboutDialogHandler->setParent(app->Engine());

  auto guiFileHandler = new gz::sim::gui::GuiFileHandler();
  guiFileHandler->setParent(app->Engine());

  auto pathManager = new gz::sim::gui::PathManager();
  pathManager->setParent(app->Engine());

  // add import path so we can load custom modules
  app->Engine()->addImportPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);
  std::string defaultGuiConfigName = "gui.config";

  // Set default config file for Gazebo
  std::string defaultConfig;

  // Default config folder.
  std::string defaultConfigFolder;

  if (nullptr == _defaultGuiConfig)
  {
    // The playback flag (and not the gui-config flag) was
    // specified from the command line
    if (nullptr != _guiConfig && std::string(_guiConfig) == "_playback_")
    {
      defaultGuiConfigName = "playback_gui.config";
    }
    gz::common::env(IGN_HOMEDIR, defaultConfig);
    defaultConfigFolder =
      gz::common::joinPaths(defaultConfig, ".ignition",
        "gazebo", IGNITION_GAZEBO_MAJOR_VERSION_STR);
    defaultConfig = gz::common::joinPaths(defaultConfigFolder,
        defaultGuiConfigName);
  }
  else
  {
    defaultConfig = _defaultGuiConfig;
  }

  app->SetDefaultConfigPath(defaultConfig);

  // Customize window
  auto mainWin = app->findChild<gz::gui::MainWindow *>();
  if (_renderEngine != nullptr)
  {
    mainWin->SetRenderEngine(_renderEngine);
  }
  auto win = mainWin->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Let QML files use C++ functions and properties
  auto context = new QQmlContext(app->Engine()->rootContext());
  context->setContextProperty("AboutDialogHandler", aboutDialogHandler);
  context->setContextProperty("GuiFileHandler", guiFileHandler);

  // Instantiate GazeboDrawer.qml file into a component
  QQmlComponent component(app->Engine(), ":/Gazebo/GazeboDrawer.qml");
  auto gzDrawerItem = qobject_cast<QQuickItem *>(component.create(context));
  if (gzDrawerItem)
  {
    // C++ ownership
    QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

    // Add to main window
    auto parentDrawerItem = win->findChild<QQuickItem *>("sideDrawer");
    gzDrawerItem->setParentItem(parentDrawerItem);
    gzDrawerItem->setParent(app->Engine());
  }
  else
  {
    ignerr << "Failed to instantiate custom drawer, drawer will be empty"
           << std::endl;
  }

  // Get list of worlds
  gz::transport::Node node;
  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/gazebo/worlds"};
  gz::msgs::StringMsg_V worldsMsg;

  // This loop is here to allow the server time to download resources.
  // \todo(nkoenig) Async resource download. Search for "Async resource
  // download in `src/Server.cc` for corresponding todo item. This todo is
  // resolved when this while loop can be removed.
  while (!sigKilled && !executed)
  {
    igndbg << "GUI requesting list of world names. The server may be busy "
      << "downloading resources. Please be patient." << std::endl;
    executed = node.Request(service, timeout, worldsMsg, result);
  }

  // Only print error message if a sigkill was not received.
  if (!sigKilled)
  {
    if (!executed)
      ignerr << "Timed out when getting world names." << std::endl;
    else if (!result)
      ignerr << "Failed to get world names." << std::endl;
  }

  if (!executed || !result || worldsMsg.data().empty())
    return nullptr;

  std::size_t runnerCount = 0;

  // Configuration file from command line
  if (_guiConfig != nullptr && std::strlen(_guiConfig) > 0 &&
      std::string(_guiConfig) != "_playback_")
  {
    // Use the first world name with the config file
    // TODO(anyone) Most of ign-gazebo's transport API includes the world name,
    // which makes it complicated to mix configurations across worlds.
    // We could have a way to use world-agnostic topics like Gazebo-classic's ~
    auto runner = new gz::sim::GuiRunner(worldsMsg.data(0));
    ++runnerCount;
    runner->setParent(gz::gui::App());

    // Load plugins after runner is up
    if (!app->LoadConfig(_guiConfig))
    {
      ignwarn << "Failed to load config file[" << _guiConfig << "]."
              << std::endl;
    }
  }
  // GUI configuration from SDF (request to server)
  else
  {
    // TODO(anyone) Parallelize this if multiple worlds becomes an important use
    // case.
    for (int w = 0; w < worldsMsg.data_size(); ++w)
    {
      const auto &worldName = worldsMsg.data(w);

      // Request GUI info for each world
      result = false;
      gz::msgs::GUI res;
      service = transport::TopicUtils::AsValidTopic("/world/" + worldName +
          "/gui/info");
      if (service.empty())
      {
        ignerr << "Failed to generate valid service for world [" << worldName
               << "]" << std::endl;
      }
      else
      {
        igndbg << "Requesting GUI from [" << service << "]..." << std::endl;

        // Request and block
        executed = node.Request(service, timeout, res, result);

        if (!executed)
        {
          ignerr << "Service call timed out for [" << service << "]"
                 << std::endl;
        }
        else if (!result)
        {
          ignerr << "Service call failed for [" << service << "]" << std::endl;
        }
      }

      // GUI runner
      auto runner = new gz::sim::GuiRunner(worldName);
      runner->setParent(gz::gui::App());
      ++runnerCount;

      // Load plugins after creating GuiRunner, so they can access worldName
      if (_loadPluginsFromSdf)
      {
        for (int p = 0; p < res.plugin_size(); ++p)
        {
          const auto &plugin = res.plugin(p);
          const auto &fileName = plugin.filename();
          std::string pluginStr = "<plugin filename='" + fileName + "'>" +
            plugin.innerxml() + "</plugin>";

          tinyxml2::XMLDocument pluginDoc;
          pluginDoc.Parse(pluginStr.c_str());

          app->LoadPlugin(fileName,
              pluginDoc.FirstChildElement("plugin"));
        }
      }
    }
    mainWin->configChanged();
  }

  if (0 == runnerCount)
  {
    ignerr << "Failed to start a GUI runner." << std::endl;
    return nullptr;
  }

  // If no plugins have been added, load default config file
  auto plugins = mainWin->findChildren<gz::gui::Plugin *>();
  if (plugins.empty())
  {
    // Check if there's a default config file under
    // ~/.gz/sim and use that. If there isn't, copy
    // the installed file there first.
    if (!gz::common::exists(defaultConfig))
    {
      if (!gz::common::exists(defaultConfigFolder))
      {
        if (!gz::common::createDirectories(defaultConfigFolder))
        {
          ignerr << "Failed to create the default config folder ["
            << defaultConfigFolder << "]\n";
          return nullptr;
        }
      }

      auto installedConfig = gz::common::joinPaths(
          IGNITION_GAZEBO_GUI_CONFIG_PATH, defaultGuiConfigName);
      if (!gz::common::exists(installedConfig))
      {
        ignerr << "Failed to copy installed config [" << installedConfig
               << "] to default config [" << defaultConfig << "]."
               << "(file " << installedConfig << " doesn't exist)"
               << std::endl;
        return nullptr;
      }

      if (!gz::common::copyFile(installedConfig, defaultConfig))
      {
        ignerr << "Failed to copy installed config [" << installedConfig
               << "] to default config [" << defaultConfig << "]."
               << std::endl;
        return nullptr;
      }
      else
      {
        ignmsg << "Copied installed config [" << installedConfig
               << "] to default config [" << defaultConfig << "]."
               << std::endl;
      }
    }

    // Also set ~/.gz/sim/ver/gui.config as the default path
    if (!app->LoadConfig(defaultConfig))
    {
      ignerr << "Failed to load config file[" << defaultConfig << "]."
             << std::endl;
      return nullptr;
    }
  }

  return app;
}

//////////////////////////////////////////////////
int runGui(int &_argc, char **_argv, const char *_guiConfig,
  const char *_renderEngine)
{
  auto app = sim::gui::createGui(
    _argc, _argv, _guiConfig, nullptr, true, _renderEngine);
  if (nullptr != app)
  {
    // Run main window.
    // This blocks until the window is closed or we receive a SIGINT
    app->exec();
    igndbg << "Shutting down ign-gazebo-gui" << std::endl;
    return 0;
  }

  return -1;
}
}  // namespace gui
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
