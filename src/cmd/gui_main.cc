/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>
#include <gz/utils/Environment.hh>

#include "gz/sim/config.hh"
#include "gz.hh"

using namespace gz;

//////////////////////////////////////////////////
/// \brief Structure to hold all available GUI launch options
struct GuiOptions
{
  /// \brief Name of the SDFormat file
  std::string file{""};

  /// \brief Path to GUI configuration file
  std::string guiConfig{""};

  /// \brief Render engine GUI plugin
  std::string renderEngineGui{""};

  /// \brief Render engine GUI API Backend
  std::string renderEngineGuiApiBackend{""};

  /// \brief Show the world loading menu
  int waitGui{1};
};

//////////////////////////////////////////////////
void addGuiFlags(CLI::App &_app)
{
  auto opt = std::make_shared<GuiOptions>();

  _app.add_option_function<std::string>("file",
    [opt](const std::string &_file){
      opt->file = _file;
    },
    "Name of the SDFormat file.");

  _app.add_option("--render-engine-gui", opt->renderEngineGui,
                  "Gazebo Rendering engine plugin to load for the GUI.\n"
                  "Gazebo will use OGRE2 by default. Make sure custom\n"
                  "plugins are in GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-gui-api-backend",
                  opt->renderEngineGuiApiBackend,
                  "Same as --render-engine-api-backend but only\n"
                  "for the GUI.")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_option("--render-engine", opt->renderEngineGui,
                  "Gazebo Rendering engine plugin to load for both the Server\n"
                  "and the GUI. Gazebo will use OGRE2 by default.\n"
                  "Make sure custom plugins are inside\n"
                  "GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-api-backend",
                  opt->renderEngineGuiApiBackend,
                  "API to use for both Server and GUI.\n"
                  "Possible values for ogre2:\n"
                  "   - opengl (default)\n"
                  "   - vulkan (beta)\n"
                  "   - metal (Apple only. Default for Apple)\n"
                  "Note: If Vulkan is being in the GUI and gz-gui was\n"
                  "built against Qt < 5.15.2, it may be very slow")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_option("--gui-config", opt->guiConfig,
                  "Gazebo GUI configuration file to load.\n"
                  "If no file is provided then the configuration in\n"
                  "SDF file is used. If that is also missing then\n"
                  "the default installed configuration is used.");

  _app.callback([opt](){

    // Get verbosity level from environment
    std::string verbosity;
    if(utils::env("GZ_SIM_VERBOSITY", verbosity))
    {
      cmdVerbosity(std::stoi(verbosity));
    }

    // Check SDF file and parse into string
    if(checkFile(opt->file) < 0)
      return;

    // Get flag to display Quickstart menu
    std::string waitGui;
    if(utils::env("GZ_SIM_WAIT_GUI", waitGui))
    {
      opt->waitGui = std::stoi(waitGui);
    }

    runGui(opt->guiConfig.c_str(), opt->file.c_str(), opt->waitGui,
           opt->renderEngineGui.c_str(),
           opt->renderEngineGuiApiBackend.c_str());
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run and manage Gazebo GUI."};

  addGuiFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
