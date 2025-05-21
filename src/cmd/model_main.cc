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

#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>

#include "gz/sim/config.hh"
#include "ModelCommandAPI.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available model commands
enum class ModelCommand
{
  kNone,
  kModelList,
  kModelInfo
};

//////////////////////////////////////////////////
/// \brief Structure to hold all available model options
struct ModelOptions
{
  /// \brief Command to execute
  ModelCommand command{ModelCommand::kNone};

  /// \brief Name of the model to show
  std::string modelName{""};

  /// \brief Flag to print pose of the model
  int pose{0};

  /// \brief Name of the link to show when set using argument
  /// Prints all links if set to empty string
  std::optional<std::string> linkName;

  /// \brief Name of the joint to show when set using argument
  /// Prints all joints if set to empty string
  std::optional<std::string> jointName;

  /// \brief Name of the sensor to show when set using argument
  /// Prints all sensors if set to empty string
  std::optional<std::string> sensorName;
};

//////////////////////////////////////////////////
/// \brief Callback fired when options are successfully parsed
void runModelCommand(const ModelOptions &_opt)
{
  switch(_opt.command)
  {
    case ModelCommand::kModelList:
      cmdModelList();
      break;
    case ModelCommand::kModelInfo:
      {
        const char *linkName{nullptr};
        if(_opt.linkName.has_value())
          linkName = _opt.linkName.value().c_str();

        const char *jointName{nullptr};
        if(_opt.jointName.has_value())
          jointName = _opt.jointName.value().c_str();

        const char *sensorName{nullptr};
        if(_opt.sensorName.has_value())
          sensorName = _opt.sensorName.value().c_str();

        cmdModelInfo(_opt.modelName.c_str(), _opt.pose, linkName,
                     jointName, sensorName);
      }
      break;
    case ModelCommand::kNone:
    default:
      throw CLI::CallForHelp();
  }
}

//////////////////////////////////////////////////
void addModelFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ModelOptions>();

  auto command = _app.add_option_group("command", "Command to be executed.");

  command->add_flag_callback("--list",
    [opt](){
      opt->command = ModelCommand::kModelList;
    },
    "Get a list of available models.");

  auto modelCmd = command->add_option_function<std::string>("-m,--model",
    [opt](const std::string &_modelName){
      opt->command = ModelCommand::kModelInfo;
      opt->modelName = _modelName;
    },
    "Select the model to be shown.");

  auto linkCmd = command->add_option_function<std::string>("-l,--link",
    [opt](const std::string &_linkName){
      opt->linkName = _linkName;
    },
    "Select a link to show its properties.\n"
    "If no arg is passed all links are printed.")
    ->needs(modelCmd)
    ->expected(0, 1)
    ->default_val("");

  command->add_option_function<std::string>("-s,--sensor",
    [opt](const std::string &_sensorName){
      opt->sensorName = _sensorName;
    },
    "Select a sensor to show its properties.\n"
    "If no arg is passed all sensors are printed.")
    ->needs(modelCmd)
    ->needs(linkCmd)
    ->expected(0, 1)
    ->default_val("");

  command->add_option_function<std::string>("-j,--joint",
    [opt](const std::string &_jointName){
      opt->jointName = _jointName;
    },
    "Select a joint to show its properties.\n"
    "If no arg is passed all joints are printed.")
    ->needs(modelCmd)
    ->expected(0, 1)
    ->default_val("");

  command->add_flag("-p,--pose", opt->pose,
                "Print the pose of the model.")
                ->needs(modelCmd);

  _app.callback([opt](){ runModelCommand(*opt); });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Utilities for information about models."};

  app.add_flag_callback("-v,--version", [](){
      std::cout << GZ_SIM_VERSION_FULL << std::endl;
      throw CLI::Success();
    },
    "Print the current library version");

  // Dummy flags handled by gz-tools
  app.add_flag("--force-version", "Use a specific library version.");
  app.add_flag("--versions", "Show the available versions.");

  addModelFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
