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

#include <sdf/Root.hh>
#include <sdf/parser.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "InsertModel.hh"

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

    public: GridModel gridModel;
  };
}

using namespace ignition;
using namespace gazebo;

GridModel::GridModel() : QStandardItemModel()
{
}

void GridModel::AddLocalModel(LocalModel &_model)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("GridModel::AddLocalModel");
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto localModel = new QStandardItem(QString::fromStdString(_model.name));
  localModel->setData(QString::fromStdString(_model.thumbnailPath),
                      this->roleNames().key("thumbnail"));
  localModel->setData(QString::fromStdString(_model.name),
                      this->roleNames().key("name"));
  localModel->setData(QString::fromStdString(_model.sdfPath),
                      this->roleNames().key("sdf"));

  parentItem->appendRow(localModel);
}

QHash<int, QByteArray> GridModel::roleNames() const
{
  return {
    std::pair(100, "thumbnail"),
    std::pair(101, "name"),
    std::pair(102, "sdf"),
  };
}

/////////////////////////////////////////////////
InsertModel::InsertModel()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<InsertModelPrivate>())
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "LocalModelList", &this->dataPtr->gridModel);
}

/////////////////////////////////////////////////
InsertModel::~InsertModel() = default;

void InsertModel::FindLocalModels(const std::string &_path)
{
  std::string path = _path;
  // Recurse if directory
  if (common::isDirectory(path))
  {
    for (common::DirIter file(path); file != common::DirIter(); ++file)
    {
      std::string current(*file);
      this->FindLocalModels(current);
    }
  }
  else if (common::isFile(path))
  {
    common::changeToUnixPath(path);
    std::string::size_type index = path.rfind("/");
    std::string fileName = path.substr(index+1);
    // If we have found model.config, extract thumbnail and sdf
    if (fileName == "model.config")
    {
      LocalModel model;
      model.configPath = path;
      std::string modelPath = path.substr(0, index);
      std::string thumbnailPath = modelPath + "/thumbnails";

      tinyxml2::XMLDocument doc;
      doc.LoadFile((modelPath + "/model.config").c_str());
      auto modelXml = doc.FirstChildElement("model");
      if (modelXml)
      {
        auto modelName = modelXml->FirstChildElement("name");
        if (modelName)
          model.name = modelName->GetText();
      }

      std::string sdfPath = sdf::getModelFilePath(modelPath);
      model.sdfPath = sdfPath;
      // Get first thumbnail image found
      if (common::exists(thumbnailPath))
      {
        for (common::DirIter file(thumbnailPath);
             file != common::DirIter(); ++file)
        {
          std::string current(*file);
          if (common::isFile(current))
          {
            std::string::size_type thumbnailIndex = current.rfind("/");
            std::string thumbnailFileName = current.substr(thumbnailIndex + 1);
            std::string::size_type thumbnailExtensionIndex =
              thumbnailFileName.rfind(".");
            std::string thumbnailFileExtension =
              thumbnailFileName.substr(thumbnailExtensionIndex + 1);
            if (thumbnailFileExtension == "png" ||
                thumbnailFileExtension == "jpg" ||
                thumbnailFileExtension == "jpeg")
            {
              model.thumbnailPath = current;
              break;
            }
          }
        }
      }
      this->dataPtr->gridModel.AddLocalModel(model);
    }
  }
}

void InsertModel::FindLocalModels(const std::vector<std::string> &_paths)
{
  for (const auto &path : _paths)
  {
    this->FindLocalModels(path);
  }
}

/////////////////////////////////////////////////
void InsertModel::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "InsertModel";

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);

  // TODO(john): create vector of paths from IGN_GAZEBO_RESOURCE_PATH here
  std::string path =
    "/home/john/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models";
  std::vector<std::string> paths;
  paths.push_back(path);

  this->FindLocalModels(paths);
}

/////////////////////////////////////////////////
void InsertModel::OnMode(const QString &_sdfPath)
{
  std::string modelSdfPath = _sdfPath.toStdString();

  // Parse the sdf from the path
  std::ifstream nameFileout;
  nameFileout.open(modelSdfPath);
  std::string line;
  std::string modelSdfString = "";
  while (std::getline(nameFileout, line))
    modelSdfString += line + "\n";

  auto event = new gui::events::SpawnPreviewModel(modelSdfString);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::InsertModel,
                    ignition::gui::Plugin)
