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
#include "ModelCommandAPI.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available commands
enum class ModelCommand
{
  kNone,
  kList,
};

//////////////////////////////////////////////////
/// \brief Structure to hold available server options
struct ModelOptions
{
  /// \brief Command to execute
  ModelCommand command{ModelCommand::kNone};

  std::string modelName;

  bool pose = false;

  std::string link;

  std::string joint;

  bool allJoints = false;
  bool allLinks = false;

  /// Verbosity level
  int verboseLevel = 1;
};

//////////////////////////////////////////////////
void runModelCommand(const ModelOptions &_opt)
{
  cmdVerbosity(_opt.verboseLevel);

  switch(_opt.command)
  {
    case ModelCommand::kList:
      cmdModelList();
      return;
    default:
      break;
  }

  if (!_opt.modelName.empty())
  {
    const char * link_c = nullptr;
    const char * joint_c = nullptr;
    if (!_opt.link.empty() || _opt.allLinks)
    {
      link_c = static_cast<const char *>(_opt.link.c_str());
    }

    if (!_opt.joint.empty() || _opt.allJoints)
    {
      joint_c = static_cast<const char *>(_opt.joint.c_str());
    }

    cmdModelInfo(
      _opt.modelName.c_str(), _opt.pose,
      link_c, joint_c);
  }
}

//////////////////////////////////////////////////
void addServerFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ModelOptions>();

  _app.add_option("-v,--verbose",
                  opt->verboseLevel,
                  "Adjust the level of console output (0~4).\n"
                  "The default verbosity is 1, use -v without \n"
                  "arguments for level 3.");


  _app.add_flag_callback("--list",
    [opt](){
      opt->command = ModelCommand::kList;
    }, "Get a list of the available models");

  auto allJointsOpt = _app.add_flag_callback("--all-joints",
    [opt](){
      opt->allJoints = true;
    }, "All joints are printed. Requires the -m option");
  allJointsOpt->needs(allJointsOpt);

  auto allLinksOpt =_app.add_flag_callback("--all-links",
    [opt](){
      opt->allLinks = true;
    }, "All links are printed. Requires the -m option");
  allLinksOpt->needs(allLinksOpt);

  auto modelOpt = _app.add_option("-m,--model",
    opt->modelName,
    "Select the model to be shown");

  _app.add_flag_callback("-p,--pose",
    [opt](){
      opt->pose = true;
    },
    "Print the pose of the model");

  auto linkOpt = _app.add_option("-l,--link",
    opt->link,
    "Select a link to show its properties.\n"
    "Requires the -m option\n\n"
    "E.g. to get information about the\n"
    "caster link in the diff_drive world, run:\n"
    "ign model -m vehicle_blue -l caster\n");
  linkOpt->needs(modelOpt);

  auto jointOpt = _app.add_option("-j,--joint",
    opt->joint,
    "Select a joint to show its properties.\n"
    "Requires the -m option\n\n"
    "E.g. to get information about the\n"
    "caster_wheel joint in the diff_drive\n"
    "world, run:\n"
    "ign model -m vehicle_blue -j caster_wheel\n");
  jointOpt->needs(modelOpt);
  _app.callback([opt](){
    runModelCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Model Ignition Gazebo Command"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
   std::cout << ignitionGazeboVersion() << std::endl;
   throw CLI::Success();
  });

  addServerFlags(app);
  CLI11_PARSE(app, argc, argv);
}
