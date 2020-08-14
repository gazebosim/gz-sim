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
    public: ResourceModel resourceModel;

    /// \brief The path list model that the qml treeview reflects for local
    /// resources
    public: PathModel pathModel;

    /// \brief The owner list model that the qml treeview reflects for fuel
    /// resources
    public: PathModel ownerModel;

    /// \brief Client used to download resources from Ignition Fuel.
    public: std::unique_ptr<ignition::fuel_tools::FuelClient>
            fuelClient = nullptr;

    /// \brief Stores details about all Fuel servers providing assets.
    /// The key is the server URI and the value is the list of models
    /// corresponding to that URI.
    public: std::unordered_map<std::string,
            std::vector<ignition::fuel_tools::ModelIdentifier>> fuelDetails;

    /// \brief The map to cache resources after a search is made on an owner,
    /// reduces redundant searches
    public: std::unordered_map<std::string,
            std::vector<Resource>> ownerModelMap;
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
ResourceModel::ResourceModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void ResourceModel::Clear()
{
  QStandardItem *parentItem{nullptr};
  parentItem = this->invisibleRootItem();

  while (parentItem->rowCount() > 0)
  {
    parentItem->removeRow(0);
  }
  this->fuelGridIndex = 0;
  this->localGridIndex = 0;
}

/////////////////////////////////////////////////
void ResourceModel::AddResource(Resource &_resource)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("GridModel::AddResource");
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto resource = new QStandardItem(QString::fromStdString(_resource.name));
  resource->setData(_resource.isFuel,
                      this->roleNames().key("isFuel"));
  resource->setData(_resource.isDownloaded,
                      this->roleNames().key("isDownloaded"));
  resource->setData(QString::fromStdString(_resource.thumbnailPath),
                      this->roleNames().key("thumbnail"));
  resource->setData(QString::fromStdString(_resource.name),
                      this->roleNames().key("name"));
  resource->setData(QString::fromStdString(_resource.sdfPath),
                      this->roleNames().key("sdf"));
  if (_resource.isFuel)
  {
    resource->setData(this->fuelGridIndex,
                        this->roleNames().key("index"));
    this->fuelGridIndex++;
  }
  else
  {
    resource->setData(this->localGridIndex,
                        this->roleNames().key("index"));
    this->localGridIndex++;
  }

  parentItem->appendRow(resource);
}

/////////////////////////////////////////////////
void ResourceModel::UpdateResourceModel(int index, Resource &_resource)
{
  QStandardItem *parentItem{nullptr};

  parentItem = this->invisibleRootItem();

  auto resource = parentItem->child(index);

  resource->setData(_resource.isFuel,
                      this->roleNames().key("isFuel"));
  resource->setData(_resource.isDownloaded,
                      this->roleNames().key("isDownloaded"));
  resource->setData(QString::fromStdString(_resource.thumbnailPath),
                      this->roleNames().key("thumbnail"));
  resource->setData(QString::fromStdString(_resource.sdfPath),
                      this->roleNames().key("sdf"));
}

/////////////////////////////////////////////////
QHash<int, QByteArray> ResourceModel::roleNames() const
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
      "ResourceList", &this->dataPtr->resourceModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "PathList", &this->dataPtr->pathModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "OwnerList", &this->dataPtr->ownerModel);
  this->dataPtr->fuelClient =
    std::make_unique<ignition::fuel_tools::FuelClient>();
}

/////////////////////////////////////////////////
ResourceSpawner::~ResourceSpawner() = default;

/////////////////////////////////////////////////
void ResourceSpawner::SetThumbnail(const std::string &_thumbnailPath,
    Resource &_resource)
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
          _resource.thumbnailPath = current;
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::LoadLocalResource(const std::string &_path)
{
  std::string fileName = common::basename(_path);
  if (!common::isFile(_path) || fileName != "model.config")
    return;

  // If we have found model.config, extract thumbnail and sdf
  Resource resource;
  std::string resourcePath = common::parentPath(_path);
  std::string thumbnailPath = common::joinPaths(resourcePath, "thumbnails");
  std::string configFileName = common::joinPaths(resourcePath, "model.config");
  tinyxml2::XMLDocument doc;
  doc.LoadFile(configFileName.c_str());
  auto modelXml = doc.FirstChildElement("model");

  // Get the name of the model
  if (modelXml)
  {
    auto modelName = modelXml->FirstChildElement("name");
    if (modelName)
      resource.name = modelName->GetText();
  }
  std::string sdfPath = sdf::getModelFilePath(resourcePath);
  resource.sdfPath = sdfPath;

  // Get first thumbnail image found
  this->SetThumbnail(thumbnailPath, resource);
  this->dataPtr->resourceModel.AddResource(resource);
}

/////////////////////////////////////////////////
void ResourceSpawner::FindLocalResources(const std::string &_path)
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
          this->LoadLocalResource(modelConfigPath);
      }
      else
      {
        this->LoadLocalResource(currentPath);
      }
    }
  }
  else
  {
    this->LoadLocalResource(path);
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::FindFuelResources(const std::string &_owner)
{
  // If we have already made this search, load the stored results
  if (this->dataPtr->ownerModelMap.find(_owner) !=
      this->dataPtr->ownerModelMap.end())
  {
    for (Resource resource : this->dataPtr->ownerModelMap[_owner])
    {
      this->dataPtr->resourceModel.AddResource(resource);
    }
    return;
  }

  auto servers = this->dataPtr->fuelClient->Config().Servers();

  // Iterate through the loaded servers and search for any models belonging
  // to the owner
  std::vector<Resource> ownerResources;
  for (auto const &server : servers)
  {
    std::string serverUrl = server.Url().Str();
    for (auto id : this->dataPtr->fuelDetails[serverUrl])
    {
      if (_owner == id.Owner())
      {
        Resource resource;
        resource.name = id.Name();
        resource.isFuel = true;
        resource.isDownloaded = false;
        resource.sdfPath = id.UniqueName();
        std::string path;

        // If the resource is cached, we can go ahead and populate the
        // respective information
        if (this->dataPtr->fuelClient->CachedModel(
              ignition::common::URI(id.UniqueName()), path))
        {
          resource.isDownloaded = true;
          resource.sdfPath = ignition::common::joinPaths(path, "model.sdf");
          std::string thumbnailPath = common::joinPaths(path, "thumbnails");
          this->SetThumbnail(thumbnailPath, resource);
        }

        ownerResources.push_back(resource);
        this->dataPtr->resourceModel.AddResource(resource);
      }
    }
  }
  this->dataPtr->ownerModelMap[_owner] = ownerResources;
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
  this->dataPtr->resourceModel.Clear();
  this->FindLocalResources(_path.toStdString());
  QGuiApplication::restoreOverrideCursor();
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFuelResource(const QString &_path, int index)
{
  Resource resource;
  std::string localPath;

  // Set the waiting cursor while the resource downloads
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  if (this->dataPtr->fuelClient->DownloadModel(
        ignition::common::URI(_path.toStdString()), localPath))
  {
    // Successful download, set thumbnail
    std::string thumbnailPath = common::joinPaths(localPath, "thumbnails");
    this->SetThumbnail(thumbnailPath, resource);
    resource.isDownloaded = true;
    resource.sdfPath = common::joinPaths(localPath, "model.sdf");
    resource.isFuel = true;
    this->dataPtr->resourceModel.UpdateResourceModel(index, resource);
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
  this->dataPtr->resourceModel.Clear();
  this->FindFuelResources(_owner.toStdString());
  QGuiApplication::restoreOverrideCursor();
}

/////////////////////////////////////////////////
void ResourceSpawner::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Resource Spawner";

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
  ignmsg << "Please wait... Loading models from Fuel.\n";

  // Add notice for the user that fuel resources are being loaded
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
    ignmsg << "Fuel resources loaded.\n";
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
