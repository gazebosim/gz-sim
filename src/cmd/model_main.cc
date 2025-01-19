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

#include <string>

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>

#include "gz/sim/config.hh"
#include "gz.hh"

enum class ModelCommand
{
  kNone,
  kModelList,
  kModelInfo
};

struct ModelOptions
{
  ModelCommand command{ModelCommand::kNone};

  std::string modelName{""};

  int pose{0};

  std::string linkName{""};

  std::string jointName{""};

  std::string sensorName{""};
};

void runModelCommand(const ModelOptions &_opt)
{
  switch(_opt.command)
  {
    case ModelCommand::kModelList:
      cmdModelList();
      break;
    case ModelCommand::kModelInfo:
      cmdModelInfo(_opt.modelName.c_str(), _opt.pose, _opt.linkName.c_str(),
                   _opt.jointName.c_str(), _opt.sensorName.c_str());
      break;
    case ModelCommand::kNone:
    default:
      throw CLI::CallForHelp();
  }
}

void addModelFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ModelOptions>();

  _app.add_flag("-p,--pose", opt->pose,
                "Print the pose of the model.");
  _app.add_option("-l,--link", opt->linkName,
                  "Select a link to show its properties.\n"
                  "If no arg is passed all links are printed.");
  _app.add_option("-s,--sensor", opt->sensorName,
                  "Select a sensor to show its properties.\n"
                  "If no arg is passed all sensors are printed.");
  _app.add_option("-j,--joint", opt->jointName,
                  "Select a joint to show its properties.\n"
                  "If no arg is passed all joints are printed.");

  auto command = _app.add_option_group("command", "Command to be executed.");

  command->add_flag_callback("--list",
    [opt](){
      opt->command = ModelCommand::kModelList;
    },
    "Get a list of available models.");

  command->add_option_function<std::string>("-m,--model",
    [opt](const std::string &_modelName){
      opt->command = ModelCommand::kModelInfo;
      opt->modelName = _modelName;
    },
    "Select the model to be shown.");

  _app.callback([opt](){ runModelCommand(*opt); });
}

int main(int argc, char** argv)
{
  CLI::App app{"Utilities for information about models."};

  app.add_flag_callback("-v,--version", [](){
      std::cout << GZ_SIM_VERSION_FULL << std::endl;
      throw CLI::Success();
    });

  addModelFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
