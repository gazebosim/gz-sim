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
#include <signal.h>
#include <gflags/gflags.h>

#include <ignition/common/Console.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <iostream>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/TmpIface.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");
DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");
DEFINE_double(z, -1, "Update rate in Hertz.");
DEFINE_uint64(iterations, 0, "Number of iterations to execute.");
DEFINE_bool(s, false, "Run only the server (headless mode).");
DEFINE_bool(g, false, "Run only the GUI.");
DEFINE_string(f, "", "Load an SDF file on start.");
DEFINE_bool(r, false, "Run simulation on start. "
    "The default is false, which starts simulation paused.");
DEFINE_bool(levels, false, "Use levels");

//////////////////////////////////////////////////
void Help()
{
  std::cout
  << "ign-gazebo -- Run the Gazebo server and GUI." << std::endl
  << std::endl
  << "`ign-gazebo` [options] <world_file>" << std::endl
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
  << "  --iterations arg       Number of iterations to execute."
  << std::endl
  << "  -s                     Run only the server (headless mode). This will "
  << " override -g, if it is also present."
  << std::endl
  << "  -g                     Run only the GUI."
  << std::endl
  << "  -f                     Load an SDF file on start. "
  << std::endl
  << "  -z arg                 Update rate in Hertz."
  << std::endl
  << "  -r                     Run simulation on start."
  << " The default is false, which starts simulation paused."
  << std::endl
  << "  --levels               Use the level system."
  << " The default is false, which loads all models."
  << std::endl
  << std::endl;
}

//////////////////////////////////////////////////
void Version()
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
    Help();
  }
  // If version is requested, override with custom version print function.
  else if (showVersion)
  {
    gflags::SetCommandLineOption("version", "false");
    Version();
  }
  // Run Gazebo
  else
  {
    // Set verbosity
    ignition::common::Console::SetVerbosity(FLAGS_verbose);
    ignmsg << "Ignition Gazebo v" << IGNITION_GAZEBO_VERSION_FULL << std::endl;

    ignition::gazebo::ServerConfig serverConfig;
    if (!serverConfig.SetSdfFile(FLAGS_f))
    {
      ignerr << "Failed to set SDF file[" << FLAGS_f << "]" << std::endl;
      return -1;
    }

    // Set the update rate.
    if (FLAGS_z > 0.0)
      serverConfig.SetUpdateRate(FLAGS_z);

    if (FLAGS_levels)
    {
      igndbg << "Using the level system\n";
      serverConfig.SetUseLevels(true);
    }

    // Run only the server (headless)
    if (FLAGS_s)
    {
      // Create the Gazebo server
      ignition::gazebo::Server server(serverConfig);

      // Run the server, and block.
      server.Run(true, FLAGS_iterations, !FLAGS_r);
    }
    // Run the GUI, or GUI+server
    else
    {
      // Temporary transport interface
      auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

      // Initialize Qt app
      ignition::gui::Application app(_argc, _argv);

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

      // Run the server along with the GUI if FLAGS_g is not set.
      std::unique_ptr<ignition::gazebo::Server> server;
      if (!FLAGS_g)
      {
        // Create the server
        server.reset(new ignition::gazebo::Server(serverConfig));

        // Run the server, and don't block.
        server->Run(false, FLAGS_iterations, !FLAGS_r);
      }


      // Run main window.
      // This blocks until the window is closed or we receive a SIGINT
      app.exec();
    }
  }

  igndbg << "Shutting down" << std::endl;
  return 0;
}
