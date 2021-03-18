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
/// \brief Structure to hold available server options
struct ServerOptions
{
};

//////////////////////////////////////////////////
void runServer(ServerOptions *_opts)
{

}

//////////////////////////////////////////////////
void addServerFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ServerOptions>();

  _app.callback([opt](){ 
      runServerCommand(*opt); 
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Start Ignition Gazebo Server"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("-v,--version", [](){
      std::cout << IGNITION_GAZEBO_VERSION_FULL << std::endl;
      throw CLI::Success();
  });

  addServerFlags(app);
  CLI11_PARSE(app, argc, argv);
}

