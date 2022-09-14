/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "gz.hh"

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

#include <gz/sim/gui/Gui.hh>

//////////////////////////////////////////////////
/// \brief Structure to hold all available topic options
struct GuiOptions 
{
  /// \brief Gui configuration
  std::string guiConfig;

  /// \brief Render engine to use
  std::string renderEngine {""};

  std::string sdfFile;

  std::vector<std::string> args;
};

//////////////////////////////////////////////////
void runGui(GuiOptions _options)
{
  std::vector<char *> argv;
  argv.reserve(_options.args.size());
  for (const auto &arg: _options.args)
  {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  int argc = argv.size();

  auto app = gz::sim::gui::createGui(
      argc,
      &argv[0],
      _options.guiConfig.c_str(), 
      nullptr, true,
      _options.sdfFile.c_str(), false, "");

  app->exec();
}

//////////////////////////////////////////////////
void addGuiFlags(CLI::App &_app)
{
  auto opt = std::make_shared<GuiOptions>();

  _app.add_option("file", opt->sdfFile);
  opt->args.push_back(_app.get_name());
  _app.callback([opt](){runGui(*opt); });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run Gazebo Server"};
  addGuiFlags(app);
  CLI11_PARSE(app, argc, argv);
}

