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

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>


#include "ignition/gazebo/config.hh"
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
    const char *_playback)
{
  ignmsg << "Ignition Gazebo Server v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  ignition::gazebo::ServerConfig serverConfig;

  // Set the SDF string to user
  if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
  {
    if (!serverConfig.SetSdfString(_sdfString))
    {
      ignerr << "Failed to set SDF string [" << _sdfString << "]" << std::endl;
      return -1;
    }
  }

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

  if ((_recordPath != nullptr && std::strlen(_recordPath) > 0) || _record > 0)
  {
    if (_playback != nullptr && std::strlen(_playback) > 0)
    {
      ignerr << "Both record and playback are specified. Only specify one.\n";
      return -1;
    }

    serverConfig.SetUseLogRecord(true);

    if (_recordPath != nullptr && std::strlen(_recordPath) > 0)
    {
      serverConfig.SetLogRecordPath(_recordPath);
    }
    else
    {
      ignmsg << "Recording states to default path\n";
    }
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
      serverConfig.SetLogPlaybackPath(_playback);
    }
  }

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  // Run the server
  server.Run(true, _iterations, _run == 0);

  igndbg << "Shutting down ign-gazebo-server" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE int runGui()
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

  // Load configuration file
  auto configPath = ignition::common::joinPaths(
      IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");

  if (!app.LoadConfig(configPath))
  {
    return -1;
  }

  // Customize window
  auto win = app.findChild<ignition::gui::MainWindow *>()->QuickWindow();
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
    igndbg << "Requesting list of world names. The server may be busy "
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

  if (!executed || !result)
    return -1;

  // TODO(anyone) Parallelize this if multiple worlds becomes an important use
  // case.
  for (int w = 0; w < worldsMsg.data_size(); ++w)
  {
    // Request GUI info for each world
    result = false;
    service = std::string("/world/" + worldsMsg.data(w) + "/gui/info");

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

      ignition::gui::App()->LoadPlugin(fileName,
          pluginDoc.FirstChildElement("plugin"));
    }
  }

  // Run main window.
  // This blocks until the window is closed or we receive a SIGINT
  app.exec();

  igndbg << "Shutting down ign-gazebo-gui" << std::endl;
  return 0;
}
