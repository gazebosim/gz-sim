/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <sdf/Root.hh>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "InsertModel.hh"

namespace filesys = std::filesystem;

namespace ignition::gazebo
{
  class InsertModelPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
InsertModel::InsertModel()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<InsertModelPrivate>())
{
}

/////////////////////////////////////////////////
InsertModel::~InsertModel() = default;

/////////////////////////////////////////////////
void InsertModel::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "InsertModel";

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);

  ignwarn << "Starting up\n";
  // TODO recursively search paths for model.config
  //std::experimental::filesystem::recursive_directory_iterator
  //  iter();

  //std::string path = filesys::path("/home/john/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models", root-directory="");
  std::string path = filesys::path("/home/john/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models");
  //std::string path = filesys::path("test");

 
  ignwarn << "Current path is " << filesys::current_path() << std::endl;
  //std::cout << "Absolute path is " << filesys::absolute(path) << "\n";

  std::vector<std::string> listOfFiles;
  try
  {
    if (filesys::exists(path) && filesys::is_directory(path))
    {
      ignwarn << "path exists and is valid\n";
      filesys::recursive_directory_iterator iter(path);
      ignwarn << "1\n";
      filesys::recursive_directory_iterator end;
      /*
      ignwarn << "2\n";
      while (iter != end)
      {
        ignwarn << "3\n";
        if (filesys::is_directory(iter->path()))
        {
          iter.disable_recursion_pending();
        ignwarn << "3.2\n";
        }
        else
        {
          listOfFiles.push_back(iter->path().string());
        ignwarn << "3.4\n";
        }
        std::error_code ec;
        ignwarn << "3.6\n";
        iter.increment(ec);
        ignwarn << "3.8\n";
        if (ec)
        {
          ignerr << "Error while accessing : " << iter->path().string() << " :: " << ec.message() << "\n";
        }
      }
      */
    }
  }
  catch (std::system_error &e)
  {
    ignerr << "Exception :: " << e.what();
  }
  ignwarn << "Found files:\n";
  for (auto &i : listOfFiles)
  {
    ignwarn << i << "\n";
  }
}

/////////////////////////////////////////////////
void InsertModel::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  

  if (modelSdfString == "box")
  {
    // TODO load sdf string from path here
    std::ifstream nameFileout;
    nameFileout.open("/home/john/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/Vent/1/model.sdf");
    std::string line;
    modelSdfString = "";
    while (std::getline(nameFileout, line))
    {
      modelSdfString += line + "\n";
    }
    std::cout << modelSdfString;
  }
  else if (modelSdfString == "sphere")
  {
  }
  else if (modelSdfString == "cylinder")
  {
  }
  else
  {
    ignwarn << "Invalid model string " << modelSdfString << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - box\n";
    ignwarn << " - sphere\n";
    ignwarn << " - cylinder\n";
    return;
  }

  auto event = new gui::events::SpawnPreviewModel(modelSdfString);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::InsertModel,
                    ignition::gui::Plugin)
