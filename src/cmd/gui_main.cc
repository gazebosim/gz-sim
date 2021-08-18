/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <ignition/utils/cli/CLI.hpp>

#include "ign.hh"

#include "ignition/gazebo/config.hh"

//////////////////////////////////////////////////
/// \brief Structure to hold available GUI options
struct GuiOptions
{
  /// GUI config file
  public: std::string guiConfigFile;

  /// Verbosity level
  public: int verboseLevel = 1;
};

//////////////////////////////////////////////////
void runGuiCommand(const GuiOptions &_opt)
{
  cmdVerbosity(_opt.verboseLevel);

  runGui(_opt.guiConfigFile.c_str());
}

//////////////////////////////////////////////////
void addGuiFlags(CLI::App &_app)
{
  auto opt = std::make_shared<GuiOptions>();

  _app.add_option("-v,--verbose",
                  opt->verboseLevel,
                  "Adjust the level of console output (0~4).\n"
                  "The default verbosity is 1, use -v without \n"
                  "arguments for level 3.");

  _app.add_option("--gui-config",
                  opt->guiConfigFile,
                  "Ignition GUI configuration file to load\n"
                  "If no config is given, the configuration in\n"
                  "the SDF file is used. And if that's not"
                  "provided, the default installed config is"
                  "used.");

  _app.callback([opt](){
      runGuiCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Start Ignition Gazebo GUI"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
    std::cout << IGNITION_GAZEBO_VERSION_FULL << std::endl;
    throw CLI::Success();
  });

  addGuiFlags(app);
  CLI11_PARSE(app, argc, argv);
}
