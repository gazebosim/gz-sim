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
#include <QScreen>

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/Filesystem.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/gui/Dialog.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/TmpIface.hh"

#include "ignition/gazebo/gui/Gui.hh"
#include "AboutDialogHandler.hh"
#include "QuickStartHandler.hh"
#include "GuiFileHandler.hh"
#include "PathManager.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace gui
{
/// \brief Get the path to the default config file. If the file doesn't exist
/// yet, this function will copy the installed file into its location.
/// \param[in] _isPlayback True if playing back a log file
/// \param[in] _customDefaultConfig A default config passed by the CLI or
/// another caller.
/// \return Path to the default config file.
std::string defaultGuiConfigFile(bool _isPlayback,
    const char *_customDefaultConfig)
{
  std::string defaultConfig;
  std::string defaultGuiConfigName = "gui.config";
  if (nullptr == _customDefaultConfig)
  {
    // The playback flag (and not the gui-config flag) was
    // specified from the command line
    if (_isPlayback)
    {
      defaultGuiConfigName = "playback_gui.config";
    }
    common::env(IGN_HOMEDIR, defaultConfig);
    // TODO(chapulina) Add IGNITION_MAJOR_VERSION_STR when merging forward to
    // Fortress.
    // TODO(chapulina) Update to .gz/sim when merging forward to Garden
    defaultConfig = common::joinPaths(defaultConfig, ".ignition",
        "gazebo", defaultGuiConfigName);
  }
  else
  {
    // Downstream applications can override the default path
    defaultConfig = _customDefaultConfig;
  }

  // Check if the default config file exists. If it doesn't, copy the installed
  // file there first.
  if (!common::exists(defaultConfig))
  {
    common::createDirectories(common::parentPath(defaultConfig));

    auto installedConfig = common::joinPaths(
        IGNITION_GAZEBO_GUI_CONFIG_PATH, defaultGuiConfigName);
    if (!common::copyFile(installedConfig, defaultConfig))
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

  return defaultConfig;
}

//////////////////////////////////////////////////
/// \brief Launch the quick start dialog
/// \param[in] _argc Number of command line arguments.
/// \param[in] _argv Command line arguments.
/// \param[in] _defaultConfig Path to the default configuration file.
/// \param[in] _configInUse The config that the user chose to load. If the user
/// didn't pass one, this will be equal to _defaultConfig
/// \return The path to the starting world or an empty string if none was
/// chosen.
std::string launchQuickStart(int &_argc, char **_argv,
    const std::string &_defaultConfig,
    const std::string &_configInUse)
{
  ignmsg << "Gazebo Sim Quick start dialog" << std::endl;

  // Gui application in dialog mode
  auto app = std::make_unique<gz::gui::Application>(
    _argc, _argv, gz::gui::WindowType::kDialog);
  app->SetDefaultConfigPath(_defaultConfig);

  auto quickStartHandler = new gui::QuickStartHandler();
  quickStartHandler->setParent(app->Engine());

  auto dialog = new gz::gui::Dialog();
  dialog->setObjectName("quick_start");

  igndbg << "Reading Quick start menu config." << std::endl;
  auto showDialog = dialog->ReadConfigAttribute(_configInUse, "show_again");
  if (showDialog == "false")
  {
    ignmsg << "Not showing Quick start menu." << std::endl;
    return "";
  }

  // This is the fixed window size for the quick start dialog
  QSize winSize(960, 540);
  dialog->QuickWindow()->resize(winSize);
  dialog->QuickWindow()->setMaximumSize(dialog->QuickWindow()->size());
  dialog->QuickWindow()->setTitle("Gazebo quick start");

  // Position the quick start in the center of the screen
  QSize screenSize = dialog->QuickWindow()->screen()->size();
  screenSize /= 2.0;
  screenSize -= winSize / 2.0;
  dialog->QuickWindow()->setPosition(screenSize.width(), screenSize.height());

  auto context = new QQmlContext(app->Engine()->rootContext());
  context->setContextProperty("QuickStartHandler", quickStartHandler);

  std::string qmlFile("qrc:/Gazebo/QuickStart.qml");

  QQmlComponent dialogComponent(gz::gui::App()->Engine(),
      QString(QString::fromStdString(qmlFile)));

  auto dialogItem = qobject_cast<QQuickItem *>(dialogComponent.create(context));
  dialogItem->setParentItem(dialog->RootItem());

  // Run qt application and show quick dialog
  if (nullptr != app)
  {
    app->exec();
    igndbg << "Shutting quick setup dialog" << std::endl;
  }

  // Update dialog config
  dialog->UpdateConfigAttribute(_configInUse, "show_again",
    quickStartHandler->ShowAgain());
  return quickStartHandler->StartingWorld();
}

//////////////////////////////////////////////////
std::unique_ptr<gz::gui::Application> createGui(
    int &_argc, char **_argv, const char *_guiConfig,
    const char *_defaultGuiConfig, bool _loadPluginsFromSdf)
{
  return createGui(_argc, _argv, _guiConfig, _defaultGuiConfig,
    _loadPluginsFromSdf, nullptr, 0);
}

//////////////////////////////////////////////////
std::unique_ptr<gz::gui::Application> createGui(
    int &_argc, char **_argv, const char *_guiConfig,
    const char *_defaultGuiConfig, bool _loadPluginsFromSdf,
    const char *_sdfFile, int _waitGui)
{
  gz::common::SignalHandler sigHandler;
  bool sigKilled = false;
  sigHandler.AddCallback([&](const int /*_sig*/)
  {
    sigKilled = true;
  });

  ignmsg << "Ignition Gazebo GUI    v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Set auto scaling factor for HiDPI displays
  if (QString::fromLocal8Bit(qgetenv("QT_AUTO_SCREEN_SCALE_FACTOR")).isEmpty())
  {
    qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
  }

  bool isPlayback = (nullptr != _guiConfig &&
      std::string(_guiConfig) == "_playback_");
  auto defaultConfig = defaultGuiConfigFile(isPlayback, _defaultGuiConfig);

  bool hasSdfFile = (nullptr != _sdfFile && strlen(_sdfFile) != 0);
  bool configFromCli = (nullptr != _guiConfig && std::strlen(_guiConfig) > 0 &&
      std::string(_guiConfig) != "_playback_");

  transport::Node node;

  // Quick start dialog if no specific SDF file was passed and it's not playback
  std::string startingWorld;
  if (!hasSdfFile && _waitGui && !isPlayback)
  {
    std::string configInUse = configFromCli ? _guiConfig : defaultConfig;
    startingWorld = launchQuickStart(_argc, _argv, defaultConfig, configInUse);
  }
  else if (hasSdfFile)
  {
    startingWorld = _sdfFile;
  }

  if (sigKilled)
  {
    igndbg << "Received kill signal. Not starting main window." << std::endl;
    return nullptr;
  }

  // Publish starting world even if it's empty. The server is blocking waiting
  // for it.
  if (_waitGui)
  {
    std::string topic{"/gazebo/starting_world"};
    auto startingWorldPub = node.Advertise<msgs::StringMsg>(topic);
    msgs::StringMsg msg;
    msg.set_data(startingWorld);

    // Wait for the server to be listening, so we're sure it receives the
    // message.
    igndbg << "Waiting for subscribers to [" << topic << "]..." << std::endl;
    for (int sleep = 0; sleep < 100 && !startingWorldPub.HasConnections();
        ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!startingWorldPub.HasConnections())
    {
      ignwarn << "Waited for 10s for a subscriber to [" << topic
              << "] and got none." << std::endl;
    }
    startingWorldPub.Publish(msg);
  }

  // Launch main window
  auto app = std::make_unique<gz::gui::Application>(
    _argc, _argv, gz::gui::WindowType::kMainWindow);

  app->AddPluginPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  // Temporary transport interface
  auto tmp = new gz::sim::TmpIface();
  tmp->setParent(app->Engine());

  auto aboutDialogHandler = new gz::sim::gui::AboutDialogHandler();
  aboutDialogHandler->setParent(app->Engine());

  auto guiFileHandler = new gz::sim::gui::GuiFileHandler();
  guiFileHandler->setParent(app->Engine());

  auto pathManager = new gz::sim::gui::PathManager();
  pathManager->setParent(app->Engine());

  // add import path so we can load custom modules
  app->Engine()->addImportPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  app->SetDefaultConfigPath(defaultConfig);

  // Customize window
  auto mainWin = app->findChild<gz::gui::MainWindow *>();
  auto win = mainWin->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Let QML files use TmpIface' functions and properties
  auto context = new QQmlContext(app->Engine()->rootContext());
  context->setContextProperty("TmpIface", tmp);
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
  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/gazebo/worlds"};
  msgs::StringMsg_V worldsMsg;

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
  if (configFromCli)
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
      service = std::string("/world/" + worldName + "/gui/info");

      igndbg << "Requesting GUI from [" << service << "]..." << std::endl;

      // Request and block
      gz::msgs::GUI res;
      executed = node.Request(service, timeout, res, result);

      if (!executed)
        ignerr << "Service call timed out for [" << service << "]" << std::endl;
      else if (!result)
        ignerr << "Service call failed for [" << service << "]" << std::endl;

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
    // Also set ~/.ignition/gazebo/gui.config as the default path
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
int runGui(int &_argc, char **_argv, const char *_guiConfig)
{
  return runGui(_argc, _argv, _guiConfig, nullptr, 0);
}

//////////////////////////////////////////////////
int runGui(int &_argc, char **_argv,
  const char *_guiConfig, const char *_sdfFile, int _waitGui)
{
  auto app = sim::gui::createGui(_argc, _argv, _guiConfig, nullptr, true,
      _sdfFile, _waitGui);
  if (nullptr != app)
  {
    // Run main window.
    // This blocks until the window is closed or we receive a SIGINT
    app->exec();
    igndbg << "Shutting down ign-gazebo-gui" << std::endl;
    return 0;
  }
  else
    return -1;
}
}  // namespace gui
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
