/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gflags/gflags.h>
#include <tinyxml2.h>

#include <csignal>
#include <iostream>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/TmpIface.hh"

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");
DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");

//////////////////////////////////////////////////
void help()
{
  std::cout
  << "DEPRECATED: Use the 'ign gazebo' command line tool."
  << std::endl
  << std::endl
  << "ign-gazebo-gui -- Run the Gazebo GUI." << std::endl
  << std::endl
  << "`ign-gazebo-gui` [options]" << std::endl
  << std::endl
  << std::endl
  << "Options:" << std::endl
  << "  -h [ --help ]          Print help message."
  << std::endl
  << "  --version              Print version information."
  << std::endl
  << "  -v [--verbose] arg     Adjust the level of console output (0~4)."
  << " The default verbosity is 1"
  << std::endl
  << std::endl;
}

//////////////////////////////////////////////////
void version()
{
  std::cout << IGNITION_GAZEBO_VERSION_HEADER << std::endl;
}

//////////////////////////////////////////////////
static bool VerbosityValidator(const char */*_flagname*/, int _value)
{
  return _value >= 0 && _value <= 4;
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Register validators
  gflags::RegisterFlagValidator(&FLAGS_verbose, &VerbosityValidator);
  gflags::RegisterFlagValidator(&FLAGS_v, &VerbosityValidator);

  // Parse command line
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Hold info as we parse it
  gflags::CommandLineFlagInfo info;

  // Help
  // Parse out the help flag in such a way that the full help text
  // is suppressed: if --help* or -h is specified, override the default
  // help behavior and turn on --helpmatch, to only shows help for the
  // current executable (instead of showing a huge list of gflags built-ins).
  gflags::GetCommandLineFlagInfo("help", &info);
  bool showHelp = FLAGS_h || (info.current_value == "true");

  // Version
  gflags::GetCommandLineFlagInfo("version", &info);
  bool showVersion = (info.current_value == "true");

  // Verbosity
  gflags::GetCommandLineFlagInfo("verbose", &info);
  if (info.is_default)
  {
    gflags::GetCommandLineFlagInfo("v", &info);
    if (!info.is_default)
      FLAGS_verbose = FLAGS_v;
    else
      FLAGS_verbose = 1;
  }

  // If help message is requested, substitute in the override help function.
  if (showHelp)
  {
    gflags::SetCommandLineOption("help", "false");
    gflags::SetCommandLineOption("helpshort", "false");
    gflags::SetCommandLineOption("helpfull", "false");
    gflags::SetCommandLineOption("helpmatch", "");
    help();
    return 0;
  }

  // If version is requested, override with custom version print function.
  if (showVersion)
  {
    gflags::SetCommandLineOption("version", "false");
    version();
    return 0;
  }

  ignition::common::Console::SetVerbosity(FLAGS_verbose);
  ignerr << "The ign-gazebo-gui tool is deprecated, and is replaced by "
    << "`ign gazebo`\n";
  ignmsg << "Ignition Gazebo GUI    v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Temporary transport interface
  auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

  // Initialize Qt app
  ignition::gui::Application app(_argc, _argv);

  // add import path so we can load custom modules
  app.Engine()->addImportPath(IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR);

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
  while (!executed)
  {
    igndbg << "Requesting list of world names. The server may be busy "
      << "downloading resources. Please be patient." << std::endl;
    executed = node.Request(service, timeout, worldsMsg, result);
  }

  if (!executed)
    ignerr << "Timed out when getting world names." << std::endl;
  else if (!result)
    ignerr << "Failed to get world names." << std::endl;

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
