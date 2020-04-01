/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <tinyxml2.h>
#include <cstring>
#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/Filesystem.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/Plugin.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/TmpIface.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ign.hh"

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE char *ignitionGazeboVersion()
{
  return strdup(IGNITION_GAZEBO_VERSION_FULL);
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE char *gazeboVersionHeader()
{
  return strdup(IGNITION_GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdVerbosity(
    const char *_verbosity)
{
  ignition::common::Console::SetVerbosity(std::atoi(_verbosity));
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE const char *worldInstallDir()
{
  return IGN_GAZEBO_WORLD_INSTALL_DIR;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE int runServer(const char *_sdfString,
    int _iterations, int _run, float _hz, int _levels, const char *_networkRole,
    int _networkSecondaries, int _record, const char *_recordPath,
    int _logOverwrite, int _logCompress,
    const char *_playback, const char *_file)
{
  ignition::gazebo::ServerConfig serverConfig;

  // Path for logs
  std::string recordPathMod = serverConfig.LogRecordPath();

  // Path for compressed log, used to check for duplicates
  std::string cmpPath = std::string(recordPathMod);
  if (!std::string(1, cmpPath.back()).compare(ignition::common::separator("")))
  {
    // Remove the separator at end of path
    cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
  }
  cmpPath += ".zip";

  // Initialize console log
  if ((_recordPath != nullptr && std::strlen(_recordPath) > 0) || _record > 0)
  {
    if (_playback != nullptr && std::strlen(_playback) > 0)
    {
      ignerr << "Both record and playback are specified. Only specify one.\n";
      return -1;
    }

    serverConfig.SetUseLogRecord(true);

    // If a record path is specified
    if (_recordPath != nullptr && std::strlen(_recordPath) > 0)
    {
      recordPathMod = std::string(_recordPath);

      // Update compressed file path to name of recording directory path
      cmpPath = std::string(recordPathMod);
      if (!std::string(1, cmpPath.back()).compare(ignition::common::separator(
        "")))
      {
        // Remove the separator at end of path
        cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
      }
      cmpPath += ".zip";

      // Check if path or compressed file with same prefix exists
      if (ignition::common::exists(recordPathMod) ||
        ignition::common::exists(cmpPath))
      {
        // Overwrite if flag specified
        if (_logOverwrite > 0)
        {
          bool recordMsg = false;
          bool cmpMsg = false;
          // Remove files before initializing console log files on top of them
          if (ignition::common::exists(recordPathMod))
          {
            recordMsg = true;
            ignition::common::removeAll(recordPathMod);
          }
          if (ignition::common::exists(cmpPath))
          {
            cmpMsg = true;
            ignition::common::removeFile(cmpPath);
          }

          // Create log file before printing any messages so they can be logged
          ignLogInit(recordPathMod, "server_console.log");

          if (recordMsg)
          {
            ignmsg << "Log path already exists on disk! Existing files will "
              << "be overwritten." << std::endl;
            ignmsg << "Removing existing path [" << recordPathMod << "]\n";
          }
          if (cmpMsg)
          {
            if (_logCompress > 0)
            {
              ignwarn << "Compressed log path already exists on disk! Existing "
                << "files will be overwritten." << std::endl;
            }
            ignmsg << "Removing existing compressed file [" << cmpPath << "]\n";
          }
        }
        // Otherwise rename to unique path
        else
        {
          // Remove the separator at end of path
          if (!std::string(1, recordPathMod.back()).compare(
            ignition::common::separator("")))
          {
            recordPathMod = recordPathMod.substr(0, recordPathMod.length()
              - 1);
          }

          std::string recordOrigPrefix = std::string(recordPathMod);
          int count = 1;

          // Keep renaming until path does not exist for both directory and
          // compressed file
          while (ignition::common::exists(recordPathMod) ||
            ignition::common::exists(cmpPath))
          {
            recordPathMod = recordOrigPrefix +  "(" + std::to_string(count++) +
              ")";

            cmpPath = std::string(recordPathMod);
            // Remove the separator at end of path
            if (!std::string(1, cmpPath.back()).compare(
              ignition::common::separator("")))
            {
              cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
            }
            cmpPath += ".zip";
          }

          ignLogInit(recordPathMod, "server_console.log");
          ignwarn << "Log path already exists on disk! "
            << "Recording instead to [" << recordPathMod << "]" << std::endl;
          if (_logCompress > 0)
          {
            ignwarn << "Compressed log path already exists on disk! "
              << "Recording instead to [" << cmpPath << "]" << std::endl;
          }
        }
      }
      else
      {
        ignLogInit(recordPathMod, "server_console.log");
      }
      // TODO(anyone) In Ignition-D, to be moved to outside and after this
      //   if-else statement, after all ignLogInit() calls have been finalized,
      //   so that <path> in SDF will always be ignored in favor of logging both
      //   console logs and LogRecord recordings to common::ignLogDirectory().
      //   In Blueprint and Citadel, LogRecord will record to <path> if no
      //   --record-path is specified on command line.
      serverConfig.SetLogRecordPath(recordPathMod);
      serverConfig.SetLogIgnoreSdfPath(true);
    }
    // Empty record path specified. Use default.
    else
    {
      // Create log file before printing any messages so they can be logged
      ignLogInit(recordPathMod, "server_console.log");
      ignmsg << "Recording states to default path [" << recordPathMod << "]"
             << std::endl;

      serverConfig.SetLogRecordPath(recordPathMod);
    }
  }
  else
  {
    ignLogInit(serverConfig.LogRecordPath(), "server_console.log");
  }

  if (_logCompress > 0)
  {
    serverConfig.SetLogRecordCompressPath(cmpPath);
  }

  ignmsg << "Ignition Gazebo Server v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Set the SDF string to user
  if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
  {
    if (!serverConfig.SetSdfString(_sdfString))
    {
      ignerr << "Failed to set SDF string [" << _sdfString << "]" << std::endl;
      return -1;
    }
  }
  serverConfig.SetSdfFile(_file);

  // Set the update rate.
  if (_hz > 0.0)
    serverConfig.SetUpdateRate(_hz);

  // Set whether levels should be used.
  if (_levels > 0)
  {
    ignmsg << "Using the level system\n";
    serverConfig.SetUseLevels(true);
  }

  if (_networkRole && std::strlen(_networkRole) > 0)
  {
    ignmsg << "Using the distributed simulation and levels systems\n";
    serverConfig.SetNetworkRole(_networkRole);
    serverConfig.SetNetworkSecondaries(_networkSecondaries);
    serverConfig.SetUseLevels(true);
  }

  if (_playback != nullptr && std::strlen(_playback) > 0)
  {
    if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
    {
      ignerr << "Both an SDF file and playback flag are specified. "
        << "Only specify one.\n";
      return -1;
    }
    else
    {
      ignmsg << "Playing back states" << _playback << std::endl;
      serverConfig.SetLogPlaybackPath(ignition::common::absPath(
        std::string(_playback)));
    }
  }

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  // Run the server
  server.Run(true, _iterations, _run == 0);

  igndbg << "Shutting down server" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE int runGui(const char *_guiConfig)
{
  ignition::common::SignalHandler sigHandler;
  bool sigKilled = false;
  sigHandler.AddCallback([&](const int /*_sig*/)
  {
    sigKilled = true;
  });

  ignmsg << "Ignition Gazebo GUI    v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Temporary transport interface
  auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

  int argc = 0;
  char **argv = nullptr;

  // Initialize Qt app
  ignition::gui::Application app(argc, argv);
  app.AddPluginPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  // add import path so we can load custom modules
  app.Engine()->addImportPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

  // Set default config file for Gazebo
  std::string defaultConfig;
  ignition::common::env(IGN_HOMEDIR, defaultConfig);
  defaultConfig = ignition::common::joinPaths(defaultConfig, ".ignition",
      "gazebo", "gui.config");
  app.SetDefaultConfigPath(defaultConfig);

  // Customize window
  auto mainWin = app.findChild<ignition::gui::MainWindow *>();
  auto win = mainWin->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Let QML files use TmpIface' functions and properties
  auto context = new QQmlContext(app.Engine()->rootContext());
  context->setContextProperty("TmpIface", tmp.get());

  // Instantiate GazeboDrawer.qml file into a component
  QQmlComponent component(app.Engine(), ":/Gazebo/GazeboDrawer.qml");
  auto gzDrawerItem = qobject_cast<QQuickItem *>(component.create(context));
  if (gzDrawerItem)
  {
    // C++ ownership
    QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

    // Add to main window
    auto parentDrawerItem = win->findChild<QQuickItem *>("sideDrawer");
    gzDrawerItem->setParentItem(parentDrawerItem);
    gzDrawerItem->setParent(app.Engine());
  }
  else
  {
    ignerr << "Failed to instantiate custom drawer, drawer will be empty"
           << std::endl;
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
    return false;

  std::vector<ignition::gazebo::GuiRunner *> runners;

  // Configuration file from command line
  if (_guiConfig != nullptr && std::strlen(_guiConfig) > 0)
  {
    if (!app.LoadConfig(_guiConfig))
    {
      ignwarn << "Failed to load config file[" << _guiConfig << "]."
              << std::endl;
    }

    // Use the first world name with the config file
    // TODO(anyone) Most of ign-gazebo's transport API includes the world name,
    // which makes it complicated to mix configurations across worlds.
    // We could have a way to use world-agnostic topics like Gazebo-classic's ~
    auto runner = new ignition::gazebo::GuiRunner(worldsMsg.data(0));
    runner->connect(&app, &ignition::gui::Application::PluginAdded, runner,
        &ignition::gazebo::GuiRunner::OnPluginAdded);
    runners.push_back(runner);
    runner->setParent(ignition::gui::App());
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

      for (int p = 0; p < res.plugin_size(); ++p)
      {
        const auto &plugin = res.plugin(p);
        const auto &fileName = plugin.filename();
        std::string pluginStr = "<plugin filename='" + fileName + "'>" +
            plugin.innerxml() + "</plugin>";

        tinyxml2::XMLDocument pluginDoc;
        pluginDoc.Parse(pluginStr.c_str());

        app.LoadPlugin(fileName,
            pluginDoc.FirstChildElement("plugin"));
      }

      // GUI runner
      auto runner = new ignition::gazebo::GuiRunner(worldName);
      runner->connect(&app, &ignition::gui::Application::PluginAdded, runner,
          &ignition::gazebo::GuiRunner::OnPluginAdded);
      runner->setParent(ignition::gui::App());
      runners.push_back(runner);
    }
    mainWin->configChanged();
  }

  if (runners.empty())
  {
    ignerr << "Failed to start a GUI runner." << std::endl;
    return -1;
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
          IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");
      if (!ignition::common::copyFile(installedConfig, defaultConfig))
      {
        ignerr << "Failed to copy installed config [" << installedConfig
               << "] to default config [" << defaultConfig << "]."
               << std::endl;
        return -1;
      }
      else
      {
        ignmsg << "Copied installed config [" << installedConfig
               << "] to default config [" << defaultConfig << "]."
               << std::endl;
      }
    }

    // Also set ~/.ignition/gazebo/gui.config as the default path
    if (!app.LoadConfig(defaultConfig))
    {
      ignerr << "Failed to load config file[" << _guiConfig << "]."
             << std::endl;
      return -1;
    }
  }

  // Run main window.
  // This blocks until the window is closed or we receive a SIGINT
  app.exec();

  for (auto runner : runners)
    delete runner;
  runners.clear();

  igndbg << "Shutting down GUI" << std::endl;
  return 0;
}
