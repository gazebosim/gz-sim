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

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/Filesystem.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/gui/Dialog.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/TmpIface.hh"

#include "ignition/gazebo/gui/Gui.hh"
#include "AboutDialogHandler.hh"
#include "QuickSetup.hh"
#include "GuiFileHandler.hh"
#include "PathManager.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace gui
{

//////////////////////////////////////////////////
std::unique_ptr<ignition::gui::Application> createGui(
    int &_argc, char **_argv, const char *_guiConfig,
    const char *_defaultGuiConfig, bool _loadPluginsFromSdf)
{
  ignition::common::SignalHandler sigHandler;
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

  // Initialize Qt app
  auto app = std::make_unique<ignition::gui::Application>(_argc, _argv, ignition::gui::WindowType::kDialog);
  app->AddPluginPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  // Temporary transport interface
  auto tmp = new ignition::gazebo::TmpIface();
  tmp->setParent(app->Engine());

  auto aboutDialogHandler = new ignition::gazebo::gui::AboutDialogHandler();
  aboutDialogHandler->setParent(app->Engine());

  auto quickSetupHandler = new ignition::gazebo::gui::QuickSetupHandler();
  quickSetupHandler->setParent(app->Engine());

  auto guiFileHandler = new ignition::gazebo::gui::GuiFileHandler();
  guiFileHandler->setParent(app->Engine());

  auto pathManager = new ignition::gazebo::gui::PathManager();
  pathManager->setParent(app->Engine());

  // add import path so we can load custom modules
  app->Engine()->addImportPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);
  std::string defaultGuiConfigName = "gui.config";

  // Set default config file for Gazebo
  std::string defaultConfig;
  if (nullptr == _defaultGuiConfig)
  {
    // The playback flag (and not the gui-config flag) was
    // specified from the command line
    if (nullptr != _guiConfig && std::string(_guiConfig) == "_playback_")
    {
      defaultGuiConfigName = "playback_gui.config";
    }
    ignition::common::env(IGN_HOMEDIR, defaultConfig);
    defaultConfig = ignition::common::joinPaths(defaultConfig, ".ignition",
        "gazebo", defaultGuiConfigName);
  }
  else
  {
    defaultConfig = _defaultGuiConfig;
  }

  app->SetDefaultConfigPath(defaultConfig);
  auto dialog = new ignition::gui::Dialog();
  dialog->QuickWindow();

  std::string qmlFile("qrc:/Gazebo/start_dialog.qml");
  if (!QFile(QString::fromStdString(qmlFile)).exists())
  {
    ignerr << "Can't find [" << qmlFile
           << "]. Are you sure it was added to the .qrc file?" << std::endl;
  }
  QQmlComponent dialogComponent(ignition::gui::App()->Engine(),
      QString(QString::fromStdString(qmlFile)));
  if (dialogComponent.isError())
  {
    std::stringstream errors;
    errors << "Failed to instantiate QML file [" << qmlFile << "]."
           << std::endl;
    for (auto error : dialogComponent.errors())
    {
      errors << "* " << error.toString().toStdString() << std::endl;
    }
    ignerr << errors.str();
  }

  auto dialogItem = qobject_cast<QQuickItem *>(dialogComponent.create());
  if (!dialogItem)
  {
    ignerr << "Failed to instantiate QML file [" << qmlFile << "]." << std::endl
           << "Are you sure the file is valid QML? "
           << "You can check with the `qmlscene` tool" << std::endl;
  }

  dialogItem->setParentItem(dialog->RootItem());
  app->exec();

  // After dialog is shut, display the main window
  igndbg << "Dialog closed, open main window" << std::endl;


  if (!app->InitializeMainWindow())
      ignerr << "Failed to initialize main window." << std::endl;

  auto mainWin = app->findChild<ignition::gui::MainWindow *>();
  auto win = mainWin->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Let QML files use TmpIface' functions and properties
  auto context = new QQmlContext(app->Engine()->rootContext());
  context->setContextProperty("TmpIface", tmp);
  context->setContextProperty("AboutDialogHandler", aboutDialogHandler);
  context->setContextProperty("QuickSetupHandler", quickSetupHandler);
  context->setContextProperty("GuiFileHandler", guiFileHandler);

  // Instantiate GazeboDrawer.qml file into a component
  QQmlComponent component(app->Engine(), "qrc:/Gazebo/GazeboDrawer.qml");
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

  for (auto it = win->children().begin(); it != win->children().end(); ++it) {
    // if the current index is needed:
    // auto i = std::distance(v.begin(), it); 
    ignwarn << "*it.objectName()" << std::endl;
    // access element as *it
  }

  // Get list of worlds
  ignition::transport::Node node;
  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/gazebo/worlds"};
  ignition::msgs::StringMsg_V worldsMsg;

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
    auto runner = new ignition::gazebo::GuiRunner(worldsMsg.data(0));
    ++runnerCount;
    runner->setParent(ignition::gui::App());

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
      ignition::msgs::GUI res;
      executed = node.Request(service, timeout, res, result);

      if (!executed)
        ignerr << "Service call timed out for [" << service << "]" << std::endl;
      else if (!result)
        ignerr << "Service call failed for [" << service << "]" << std::endl;

      // GUI runner
      auto runner = new ignition::gazebo::GuiRunner(worldName);
      runner->setParent(ignition::gui::App());
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
  auto plugins = mainWin->findChildren<ignition::gui::Plugin *>();
  if (plugins.empty())
  {
    // Check if there's a default config file under
    // ~/.ignition/gazebo and use that. If there isn't, copy
    // the installed file there first.
    if (!ignition::common::exists(defaultConfig))
    {
      auto installedConfig = ignition::common::joinPaths(
          IGNITION_GAZEBO_GUI_CONFIG_PATH, defaultGuiConfigName);
      if (!ignition::common::copyFile(installedConfig, defaultConfig))
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

    // Also set ~/.ignition/gazebo/gui.config as the default path
    if (!app->LoadConfig(defaultConfig))
    {
      ignerr << "Failed to load config file[" << defaultConfig << "]."
             << std::endl;
      return nullptr;
    }
  }


  // Create a subscriber just so we can check when the message has propagated
  bool topicCalled{false};

  // node.Advertise("/gazebo/starting_world", 1000, 5);
  // ignerr << "Worlds advertised" << std::endl;

  std::function<void(const ignition::msgs::StringMsg_V &)> topicCb =
      [&topicCalled](const auto &_msg)
      {
        igndbg << " Recived FROM GUI " << std::endl; 
        topicCalled = true;
      };
  node.Subscribe("/gazebo/starting_world", topicCb);

  ignition::transport::Node::Publisher startingWorldPub;
  startingWorldPub = node.Advertise<msgs::StringMsg_V>("/gazebo/starting_world");
  msgs::StringMsg_V msg;
  msg.add_data("empty");
  
  if(startingWorldPub.ThrottledUpdateReady())
  {
    for (auto i = 0; i < 10; ++i)
    {
      startingWorldPub.Publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // False afterwards, because targeting 1 msg/second.
      // EXPECT_FALSE(pub.ThrottledUpdateReady());

      // Rate: 10 msgs/sec.
    }
  }
  startingWorldPub.Publish(msg);

  return app;
}

//////////////////////////////////////////////////
int runGui(int &_argc, char **_argv, const char *_guiConfig)
{
  auto app = gazebo::gui::createGui(_argc, _argv, _guiConfig);
  if (nullptr != app)
  {
    // Run main window.
    // This blocks until the window is closed or we receive a SIGINT
    // app->exec();
    igndbg << "Shutting down ign-gazebo-gui" << std::endl;
    return 0;
  }
  else
    return -1;
}
}  // namespace gui
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
