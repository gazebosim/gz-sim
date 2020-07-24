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
#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/ClientConfig.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ResourceSpawner.hh"
#include <set>

namespace ignition::gazebo
{
  class ResourceSpawnerPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief The grid model that the qml gridview reflects
    public: GridModel localGridModel;

    /// \brief The path list model that the qml treeview reflects
    public: PathModel pathModel;

    /// \brief The grid model that the qml gridview reflects
    public: GridModel fuelGridModel;

    /// \brief The path list model that the qml treeview reflects
    public: PathModel ownerModel;

    public: std::unique_ptr<ignition::fuel_tools::FuelClient> fuelClient = nullptr;

    public: std::vector<ignition::fuel_tools::ModelIdentifier> models;

    public: std::unordered_map<std::string, std::vector<ignition::fuel_tools::ModelIdentifier>> fuelDetails;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
PathModel::PathModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void PathModel::AddPath(const std::string &_path)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("PathModel::AddPath");
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto localModel = new QStandardItem(QString::fromStdString(_path));
  localModel->setData(QString::fromStdString(_path),
                      this->roleNames().key("path"));

  parentItem->appendRow(localModel);
}

/////////////////////////////////////////////////
QHash<int, QByteArray> PathModel::roleNames() const
{
  return
  {
    std::pair(100, "path"),
  };
}

/////////////////////////////////////////////////
GridModel::GridModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void GridModel::Clear()
{
  QStandardItem *parentItem{nullptr};
  parentItem = this->invisibleRootItem();

  while (parentItem->rowCount() > 0)
  {
    parentItem->removeRow(0);
  }
}

/////////////////////////////////////////////////
void GridModel::AddLocalModel(LocalModel &_model)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("GridModel::AddLocalModel");
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto localModel = new QStandardItem(QString::fromStdString(_model.name));
  localModel->setData(_model.isFuel,
                      this->roleNames().key("isFuel"));
  localModel->setData(_model.isDownloaded,
                      this->roleNames().key("isDownloaded"));
  localModel->setData(QString::fromStdString(_model.thumbnailPath),
                      this->roleNames().key("thumbnail"));
  localModel->setData(QString::fromStdString(_model.name),
                      this->roleNames().key("name"));
  localModel->setData(QString::fromStdString(_model.sdfPath),
                      this->roleNames().key("sdf"));

  parentItem->appendRow(localModel);
}

/////////////////////////////////////////////////
QHash<int, QByteArray> GridModel::roleNames() const
{
  return
  {
    std::pair(100, "thumbnail"),
    std::pair(101, "name"),
    std::pair(102, "sdf"),
    std::pair(103, "isDownloaded"),
    std::pair(104, "isFuel"),
  };
}

/////////////////////////////////////////////////
ResourceSpawner::ResourceSpawner()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<ResourceSpawnerPrivate>())
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "LocalModelList", &this->dataPtr->localGridModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "PathList", &this->dataPtr->pathModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "FuelModelList", &this->dataPtr->fuelGridModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "OwnerList", &this->dataPtr->ownerModel);
  this->dataPtr->fuelClient = std::make_unique<ignition::fuel_tools::FuelClient>();
}

/////////////////////////////////////////////////
ResourceSpawner::~ResourceSpawner() = default;

/////////////////////////////////////////////////
void ResourceSpawner::LoadLocalModel(const std::string &_path)
{
  std::string fileName = common::basename(_path);
  if (!common::isFile(_path) || fileName != "model.config")
    return;

  // If we have found model.config, extract thumbnail and sdf
  LocalModel model;
  std::string modelPath = common::parentPath(_path);
  std::string thumbnailPath = common::joinPaths(modelPath, "thumbnails");
  std::string configFileName = common::joinPaths(modelPath, "model.config");
  tinyxml2::XMLDocument doc;
  doc.LoadFile(configFileName.c_str());
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
        std::string thumbnailFileName = common::basename(current);
        std::string::size_type thumbnailExtensionIndex =
          thumbnailFileName.rfind(".");
        std::string thumbnailFileExtension =
          thumbnailFileName.substr(thumbnailExtensionIndex + 1);
        // The standard image types QML supports
        if (thumbnailFileExtension == "png"  ||
            thumbnailFileExtension == "jpg"  ||
            thumbnailFileExtension == "jpeg" ||
            thumbnailFileExtension == "svg")
        {
          model.thumbnailPath = current;
          break;
        }
      }
    }
  }
  this->dataPtr->localGridModel.AddLocalModel(model);
}

/////////////////////////////////////////////////
void ResourceSpawner::FindLocalModels(const std::string &_path)
{
  std::string path = _path;
  if (common::isDirectory(path))
  {
    for (common::DirIter file(path); file != common::DirIter(); ++file)
    {
      std::string currentPath(*file);
      if (common::isDirectory(currentPath))
      {
        std::string modelConfigPath =
          common::joinPaths(currentPath, "model.config");
          this->LoadLocalModel(modelConfigPath);
      }
      else
      {
        this->LoadLocalModel(currentPath);
      }
    }
  }
  else
  {
    this->LoadLocalModel(path);
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::FindFuelModels(const std::string &_owner)
{
  auto servers = this->dataPtr->fuelClient->Config().Servers();
  for (auto const &server : servers)
  {
    std::string serverUrl = server.Url().Str();
    for (auto id : this->dataPtr->fuelDetails[serverUrl])
    {
      if (_owner == id.Owner())
      {
        LocalModel model;
        model.name = id.Name();
        model.isFuel = true;
        model.isDownloaded = false;
        model.sdfPath = id.UniqueName();
        // TODO add resource to this->dataPtr->fuelGridModel by
        // 1 - Set the name
        // 2 - Set model sdf path
        // 3 - Set model thumbnail - only if downloaded?
        // 4
        //
        std::string path;
        if (this->dataPtr->fuelClient->CachedModel(ignition::common::URI(id.UniqueName()), path))
        {
          ignwarn << "locally cached: " << id.Name() << " and is at " << path << std::endl;
          model.isDownloaded = true;
          model.thumbnailPath = path;
        }
        
        this->dataPtr->fuelGridModel.AddLocalModel(model);
      }
    }
  }
  /*
  std::string path = _path;
  if (common::isDirectory(path))
  {
    for (common::DirIter file(path); file != common::DirIter(); ++file)
    {
      std::string currentPath(*file);
      if (common::isDirectory(currentPath))
      {
        std::string modelConfigPath =
          common::joinPaths(currentPath, "model.config");
          this->LoadLocalModel(modelConfigPath);
      }
      else
      {
        this->LoadLocalModel(currentPath);
      }
    }
  }
  else
  {
    this->LoadLocalModel(path);
  }
  */

}

/////////////////////////////////////////////////
void ResourceSpawner::AddPath(const std::string &_path)
{
  this->dataPtr->pathModel.AddPath(_path);
}

/////////////////////////////////////////////////
void ResourceSpawner::OnPathClicked(const QString &_path)
{
  this->dataPtr->localGridModel.Clear();
  this->FindLocalModels(_path.toStdString());
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFuelResource(const QString &_path)
{
  ignwarn << "Downloading " << _path.toStdString() << std::endl;

}

/////////////////////////////////////////////////
void ResourceSpawner::OnOwnerClicked(const QString &_owner)
{
  ignwarn << "Clicked " << _owner.toStdString() << std::endl;
  this->dataPtr->fuelGridModel.Clear();
  this->FindFuelModels(_owner.toStdString());
}

/////////////////////////////////////////////////
void ResourceSpawner::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Resource Spawner";

  // For resource spawn requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);

  msgs::StringMsg_V res;
  bool result;
  bool executed = this->dataPtr->node.Request(
      "/gazebo/resource_paths/get", 5000, res, result);
  if (!executed || !result || res.data_size() < 1)
  {
    ignwarn << "No paths found in IGN_GAZEBO_RESOURCE_PATH.\n";
  }

  for (int i = 0; i < res.data_size(); i++)
  {
    const std::string path = res.data(i);
    this->AddPath(path);
  }

  /*
  // TODO fuel models here
  std::vector<ignition::fuel_tools::ModelIdentifier> models;
  std::string serverName = "fuel.ignitionrobotics.org";

  auto servers = common::FuelModelDatabase::Instance()->Servers();
  for (auto const &server : servers)
  {
    std::function <void(
        const std::vector<ignition::fuel_tools::ModelIdentifier> &) f =
        [server, this](
            const std::vector<ignition::fuel_tools::ModelIdentifier> &_models)
        {
          for (auto iter = this->dataPtr->fuelClient->Models(server); iter; ++iter)
          {
            this->dataPtr->models.push_back(iter->Identification());
          }
        };

  }

  */

  auto servers = this->dataPtr->fuelClient->Config().Servers();
  
  std::thread t([this, servers]
  {
    std::set<std::string> ownerSet;
    for (auto const &server : servers)
    {
      std::vector<ignition::fuel_tools::ModelIdentifier> models;
      for (auto iter = this->dataPtr->fuelClient->Models(server); iter; ++iter)
      {
        models.push_back(iter->Identification());
      }
      std::string serverUrl = server.Url().Str();
      this->dataPtr->fuelDetails[serverUrl] = models;
      for (auto id : this->dataPtr->fuelDetails[serverUrl])
      {
        auto ownerName = id.Owner();
        ownerSet.insert(ownerName);
        auto url = id.Server().Url();
        std::string path;
      }
    }

    // Add all unique owners to the owner model
    for (const auto &owner : ownerSet)
    {
      this->dataPtr->ownerModel.AddPath(owner);
    } 
  });
  t.detach();
  ignwarn << "after server url " << std::endl;

}

/////////////////////////////////////////////////
void ResourceSpawner::OnFuelResourceSpawn(const QString &_sdfPath)
{
  std::string modelSdfPath = _sdfPath.toStdString();
  ignwarn << "Path is " << modelSdfPath << std::endl;
  /*
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
  */
}

/////////////////////////////////////////////////
void ResourceSpawner::OnResourceSpawn(const QString &_sdfPath)
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
IGNITION_ADD_PLUGIN(ignition::gazebo::ResourceSpawner,
                    ignition::gui::Plugin)
