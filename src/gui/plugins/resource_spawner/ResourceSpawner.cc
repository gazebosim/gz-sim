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

#include <set>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ResourceSpawner.hh"

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

    /// \brief The path list model that the qml treeview reflects
    public: PathModel ownerModel;

    /// \brief Client used to download resources from Ignition Fuel.
    public: std::unique_ptr<ignition::fuel_tools::FuelClient>
            fuelClient = nullptr;

    /// \brief Stores details about all Fuel servers providing assets.
    /// The key is the server URI and the value is the list of models
    /// corresponding to that URI.
    public: std::unordered_map<std::string,
            std::vector<ignition::fuel_tools::ModelIdentifier>> fuelDetails;
  };
}

static int gridIndex = 0;
static int localGridIndex = 0;

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
  if (_model.isFuel)
  {
    localModel->setData(gridIndex,
                        this->roleNames().key("index"));
    gridIndex++;
  }
  else
  {
    localModel->setData(localGridIndex,
                        this->roleNames().key("index"));
    localGridIndex++;
  }

  parentItem->appendRow(localModel);
}

void GridModel::UpdateGridModel(int index, LocalModel &_model)
{
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto grid = parentItem->child(index);

  grid->setData(_model.isFuel,
                      this->roleNames().key("isFuel"));
  grid->setData(_model.isDownloaded,
                      this->roleNames().key("isDownloaded"));
  grid->setData(QString::fromStdString(_model.thumbnailPath),
                      this->roleNames().key("thumbnail"));
  grid->setData(QString::fromStdString(_model.sdfPath),
                      this->roleNames().key("sdf"));
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
    std::pair(105, "index"),
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
      "OwnerList", &this->dataPtr->ownerModel);
  this->dataPtr->fuelClient =
    std::make_unique<ignition::fuel_tools::FuelClient>();
}

/////////////////////////////////////////////////
ResourceSpawner::~ResourceSpawner() = default;

void ResourceSpawner::SetThumbnail(const std::string &_thumbnailPath,
    LocalModel &_model)
{
  // Get first thumbnail image found
  if (common::exists(_thumbnailPath))
  {
    for (common::DirIter file(_thumbnailPath);
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
        // The standard image types QML supports, search for any file
        // with this extension and use the first found
        if (thumbnailFileExtension == "png"  ||
            thumbnailFileExtension == "jpg"  ||
            thumbnailFileExtension == "jpeg" ||
            thumbnailFileExtension == "svg")
        {
          _model.thumbnailPath = current;
          break;
        }
      }
    }
  }
}

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

  // Get the name of the model
  if (modelXml)
  {
    auto modelName = modelXml->FirstChildElement("name");
    if (modelName)
      model.name = modelName->GetText();
  }
  std::string sdfPath = sdf::getModelFilePath(modelPath);
  model.sdfPath = sdfPath;

  // Get first thumbnail image found
  this->SetThumbnail(thumbnailPath, model);
  this->dataPtr->localGridModel.AddLocalModel(model);
}

/////////////////////////////////////////////////
void ResourceSpawner::FindLocalModels(const std::string &_path)
{
  // Only searches one directory deep for potential files named `model.config`
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

  // Iterate through the loaded servers and search for any models belonging
  // to the owner
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
        std::string path;

        // If the resource is cached, we can go ahead and populate the
        // respective information
        if (this->dataPtr->fuelClient->CachedModel(
              ignition::common::URI(id.UniqueName()), path))
        {
          model.isDownloaded = true;
          model.sdfPath = ignition::common::joinPaths(path, "model.sdf");
          std::string thumbnailPath = common::joinPaths(path, "thumbnails");
          this->SetThumbnail(thumbnailPath, model);
        }

        this->dataPtr->localGridModel.AddLocalModel(model);
      }
    }
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::AddPath(const std::string &_path)
{
  this->dataPtr->pathModel.AddPath(_path);
}

/////////////////////////////////////////////////
void ResourceSpawner::OnPathClicked(const QString &_path)
{
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  this->dataPtr->localGridModel.Clear();
  this->FindLocalModels(_path.toStdString());
  QGuiApplication::restoreOverrideCursor();
  localGridIndex = 0;
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFuelResource(const QString &_path, int index)
{
  LocalModel model;
  std::string localPath;

  // Set the waiting cursor while the resource downloads
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  if (this->dataPtr->fuelClient->DownloadModel(
        ignition::common::URI(_path.toStdString()), localPath))
  {
    // Successful download, set thumbnail
    std::string thumbnailPath = common::joinPaths(localPath, "thumbnails");
    this->SetThumbnail(thumbnailPath, model);
    model.isDownloaded = true;
    model.sdfPath = common::joinPaths(localPath, "model.sdf");
    model.isFuel = true;
    this->dataPtr->localGridModel.UpdateGridModel(index, model);
  }
  else
  {
    ignwarn << "Download failed.  Try again." << std::endl;
  }
  QGuiApplication::restoreOverrideCursor();
}

/////////////////////////////////////////////////
void ResourceSpawner::OnOwnerClicked(const QString &_owner)
{
  // This may take a few seconds, set waiting cursor
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  this->dataPtr->localGridModel.Clear();
  this->FindFuelModels(_owner.toStdString());
  QGuiApplication::restoreOverrideCursor();
  gridIndex = 0;
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

  auto servers = this->dataPtr->fuelClient->Config().Servers();
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  ignmsg << "Please wait... Loading fuel models from online.\n";

  // Add notice for the user that fuel models are being loaded
  this->dataPtr->ownerModel.AddPath("Please wait, loading Fuel models...");

  // Pull in fuel models asynchronously
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
      }
    }

    // Clear the loading message
    this->dataPtr->ownerModel.clear();

    // Add all unique owners to the owner model
    for (const auto &owner : ownerSet)
    {
      this->dataPtr->ownerModel.AddPath(owner);
    }
    QGuiApplication::restoreOverrideCursor();
    ignmsg << "Fuel models loaded.\n";
  });
  t.detach();
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
