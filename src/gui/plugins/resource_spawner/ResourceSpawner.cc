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

#include "ResourceSpawner.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <algorithm>
#include <set>
#include <thread>
#include <unordered_map>

#include <sdf/Root.hh>
#include <sdf/parser.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/fuel_tools/ClientConfig.hh>
#include <gz/fuel_tools/FuelClient.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/EntityComponentManager.hh"


Q_DECLARE_METATYPE(gz::sim::Resource)

namespace gz::sim
{
  class ResourceSpawnerPrivate
  {
    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief The grid model that the qml gridview reflects
    public: ResourceModel resourceModel;

    /// \brief The path list model that the qml treeview reflects for local
    /// resources
    public: PathModel pathModel;

    /// \brief The owner list model that the qml treeview reflects for fuel
    /// resources
    public: PathModel ownerModel;

    /// \brief Client used to download resources from Gazebo Fuel.
    public: std::unique_ptr<gz::fuel_tools::FuelClient>
            fuelClient = nullptr;

    /// \brief The map to cache resources after a search is made on an owner,
    /// reduces redundant searches
    public: std::unordered_map<std::string,
            std::vector<Resource>> ownerModelMap;

    /// \brief Holds all of the relevant data used by `DisplayData()` in order
    /// to filter and sort the displayed resources as desired by the user.
    public: Display displayData;

    /// \brief The list of Fuel servers to download from.
    public: std::vector<fuel_tools::ServerConfig> servers;

    /// \brief Data structure to hold relevant bits for a worker thread that
    /// fetches the list of recources available for an owner on Fuel.
    struct FetchResourceListWorker
    {
      /// \brief Thread that runs the worker
      std::thread thread;
      /// \brief Flag to notify the worker that it needs to stop. This could be
      /// when an owner is removed or when the program exits.
      std::atomic<bool> stopDownloading{false};
      /// \brief The workers own Fuel client to avoid synchronization.
      fuel_tools::FuelClient fuelClient;
    };

    /// \brief Holds a map from owner to the associated resource list worker.
    public: std::unordered_map<std::string,
            FetchResourceListWorker> fetchResourceListWorkers;
  };
}

namespace {

// Default owner to be fetched from Fuel. This owner cannot be removed.
constexpr const char *kDefaultOwner = "openrobotics";
}
using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
PathModel::PathModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void PathModel::AddPath(const std::string &_path)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("PathModel::AddPath");
  auto localModel = new QStandardItem(QString::fromStdString(_path));
  localModel->setData(QString::fromStdString(_path),
      this->roleNames().key("path"));

  this->appendRow(localModel);
}

/////////////////////////////////////////////////
void PathModel::RemovePath(const std::string &_path)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("PathModel::RemovePath");
  QString qPath = QString::fromStdString(_path);
  for (int i = 0; i < this->rowCount(); ++i)
  {
    if (this->data(this->index(i, 0)) == qPath)
    {
      this->removeRow(i);
      break;
    }
  }
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
  this->clear();
  this->gridIndex = 0;
  emit sizeChanged();
}

/////////////////////////////////////////////////
void ResourceModel::AddResources(std::vector<Resource> &_resources)
{
  for (auto &resource : _resources)
    this->AddResource(resource);
}

/////////////////////////////////////////////////
void ResourceModel::AddResource(const Resource &_resource)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("GridModel::AddResource");

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
  resource->setData(QString::fromStdString(_resource.owner),
      this->roleNames().key("owner"));

  if (_resource.isFuel)
  {
    resource->setData(this->gridIndex,
        this->roleNames().key("index"));
    this->gridIndex++;
  }
  else
  {
    resource->setData(this->gridIndex,
        this->roleNames().key("index"));
    this->gridIndex++;
  }
  emit sizeChanged();

  this->appendRow(resource);
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
    std::pair(106, "owner"),
  };
}

/////////////////////////////////////////////////
ResourceSpawner::ResourceSpawner()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<ResourceSpawnerPrivate>())
{
  qRegisterMetaType<gz::sim::Resource>();
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
      "ResourceList", &this->dataPtr->resourceModel);
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
      "PathList", &this->dataPtr->pathModel);
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
      "OwnerList", &this->dataPtr->ownerModel);
  this->dataPtr->fuelClient =
    std::make_unique<fuel_tools::FuelClient>();

  auto servers = this->dataPtr->fuelClient->Config().Servers();
  // Since the ign->gz rename, `servers` here returns two items for the
  // canonical Fuel server: fuel.ignitionrobotics.org and fuel.gazebosim.org.
  // For the purposes of the ResourceSpawner, these will be treated as the same
  // and we will remove the ignitionrobotics server here.
  auto urlIs = [](const std::string &_url)
  {
    return [_url](const fuel_tools::ServerConfig &_server)
    { return _server.Url().Str() == _url; };
  };

  auto ignIt = std::find_if(servers.begin(), servers.end(),
                            urlIs("https://fuel.ignitionrobotics.org"));
  if (ignIt != servers.end())
  {
    auto gzsimIt = std::find_if(servers.begin(), servers.end(),
                                urlIs("https://fuel.gazebosim.org"));
    if (gzsimIt != servers.end())
    {
      servers.erase(ignIt);
    }
  }

  this->dataPtr->servers = servers;
}

/////////////////////////////////////////////////
ResourceSpawner::~ResourceSpawner()
{
  for (auto &workers : this->dataPtr->fetchResourceListWorkers)
  {
    workers.second.stopDownloading = true;
    if (workers.second.thread.joinable())
    {
      workers.second.thread.join();
    }
  }
}

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
Resource ResourceSpawner::LocalResource(const std::string &_path)
{
  std::string fileName = common::basename(_path);
  Resource resource;

  if (!common::isFile(_path) || fileName != "model.config")
    return resource;

  // If we have found model.config, extract thumbnail and sdf
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
  return resource;
}

/////////////////////////////////////////////////
std::vector<Resource> ResourceSpawner::LocalResources(const std::string &_path)
{
  // Only searches one directory deep for potential files named `model.config`
  std::string path = _path;
  std::vector<Resource> localResources;
  if (common::isDirectory(path))
  {
    for (common::DirIter file(path); file != common::DirIter(); ++file)
    {
      std::string currentPath(*file);
      Resource resource;
      if (common::isDirectory(currentPath))
      {
        std::string modelConfigPath =
          common::joinPaths(currentPath, "model.config");
        resource = this->LocalResource(modelConfigPath);
      }
      else
      {
        resource = this->LocalResource(currentPath);
      }
      if (resource.sdfPath != "")
        localResources.push_back(resource);
    }
  }
  else
  {
    Resource resource = this->LocalResource(path);
    if (resource.sdfPath != "")
      localResources.push_back(resource);
  }
  return localResources;
}

/////////////////////////////////////////////////
std::vector<Resource> ResourceSpawner::FuelResources(const std::string &_owner)
{
  std::vector<Resource> fuelResources;

  if (this->dataPtr->ownerModelMap.find(_owner) !=
      this->dataPtr->ownerModelMap.end())
  {
    for (const Resource &resource : this->dataPtr->ownerModelMap[_owner])
    {
      fuelResources.push_back(resource);
    }
  }
  return fuelResources;
}

/////////////////////////////////////////////////
bool compareByAlphabet(const Resource &a, const Resource &b)
{
  std::string aName = a.name;
  std::string bName = b.name;
  std::for_each(aName.begin(), aName.end(), [](char & c){
      c = ::tolower(c);
      });

  std::for_each(bName.begin(), bName.end(), [](char & c){
      c = ::tolower(c);
      });

  return (aName.compare(bName) < 0);
}

/////////////////////////////////////////////////
bool compareByDownloaded(const Resource &a, const Resource &b)
{
  // Only return true when a is download and b is not
  // Return false in all other cases and when they are equal
  if (a.isDownloaded && !b.isDownloaded)
  {
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void ResourceSpawner::FilterResources(std::vector<Resource> &_resources)
{
  if (this->dataPtr->displayData.searchKeyword == "")
    return;

  std::string searchKeyword = this->dataPtr->displayData.searchKeyword;
  std::for_each(searchKeyword.begin(), searchKeyword.end(), [](char & c){
      c = ::tolower(c);
      });
  auto it = _resources.begin();

  // Remove any resources from the vector that don't contain the entered
  // keyword in their name or owner name
  while (it != _resources.end())
  {
    // Check for matches in the owner and name of the resource
    std::string name = it->name;
    std::string owner = it->owner;
    std::for_each(name.begin(), name.end(), [](char & c){
        c = ::tolower(c);
        });
    std::for_each(owner.begin(), owner.end(), [](char & c){
        c = ::tolower(c);
        });
    if (name.find(searchKeyword) == std::string::npos &&
        owner.find(searchKeyword) == std::string::npos)
    {
      it = _resources.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::SortResources(std::vector<Resource> &_resources)
{
  // Sort the results by the desired sort, don't do anything
  // in the fourth case (Most Recent) as that is the order
  // by default
  if (this->dataPtr->displayData.sortMethod == "A - Z")
  {
    // Sort std::vector<Resource> resource from a to z
    std::sort(_resources.begin(), _resources.end(), compareByAlphabet);
  }
  else if (this->dataPtr->displayData.sortMethod == "Z - A")
  {
    // Sort std::vector<Resource> resource from z to a
    std::sort(_resources.begin(), _resources.end(), compareByAlphabet);
    std::reverse(_resources.begin(), _resources.end());
  }
  else if (this->dataPtr->displayData.sortMethod == "Downloaded")
  {
    // Sort std::vector<Resource> resource from downloaded to not downloaded
    std::sort(_resources.begin(), _resources.end(), compareByDownloaded);
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::Resources(std::vector<Resource> &_resources)
{
  if (this->dataPtr->displayData.isFuel)
  {
    _resources = this->FuelResources(this->dataPtr->displayData.ownerPath);
  }
  else
  {
    _resources = this->LocalResources(this->dataPtr->displayData.ownerPath);
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::DisplayResources()
{
  // Get the resources for an owner or path
  std::vector<Resource> resources;
  this->Resources(resources);

  // Filter the resource vector with the entered search keyword
  this->FilterResources(resources);

  // Sort the resources by the provided search method
  this->SortResources(resources);

  // Clear the qml grid and add the resource results
  this->dataPtr->resourceModel.Clear();
  this->dataPtr->resourceModel.AddResources(resources);
}

/////////////////////////////////////////////////
void ResourceSpawner::AddPath(const std::string &_path)
{
  this->dataPtr->pathModel.AddPath(_path);
}

/////////////////////////////////////////////////
void ResourceSpawner::OnPathClicked(const QString &_path)
{
  this->dataPtr->displayData.ownerPath = _path.toStdString();
  this->dataPtr->displayData.isFuel = false;
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFuelResource(const QString &_path,
    const QString &_name, const QString &_owner, int index)
{
  Resource modelResource;
  std::string localPath;

  // Set the waiting cursor while the resource downloads
  QGuiApplication::setOverrideCursor(Qt::WaitCursor);
  if (this->dataPtr->fuelClient->DownloadModel(
        common::URI(_path.toStdString(), true), localPath))
  {
    // Successful download, set thumbnail
    std::string thumbnailPath = common::joinPaths(localPath, "thumbnails");
    this->SetThumbnail(thumbnailPath, modelResource);
    modelResource.isDownloaded = true;
    modelResource.sdfPath = common::joinPaths(localPath, "model.sdf");
    modelResource.isFuel = true;
    // Update the current grid of resources
    this->dataPtr->resourceModel.UpdateResourceModel(index, modelResource);

    // Update the ground truth ownerModelMap
    if (this->dataPtr->ownerModelMap.find(_owner.toStdString()) !=
        this->dataPtr->ownerModelMap.end())
    {
      std::vector<Resource> fuelResources =
        this->dataPtr->ownerModelMap[_owner.toStdString()];
      for (auto &resource : fuelResources)
      {
        if (resource.name == _name.toStdString())
        {
          resource.isDownloaded = modelResource.isDownloaded;
          resource.isFuel = modelResource.isFuel;
          resource.sdfPath = modelResource.sdfPath;
          this->SetThumbnail(thumbnailPath, resource);
          this->dataPtr->ownerModelMap[_owner.toStdString()] = fuelResources;
          break;
        }
      }
    }
  }
  else
  {
    gzwarn << "Download failed.  Try again." << std::endl;
  }
  QGuiApplication::restoreOverrideCursor();
}

/////////////////////////////////////////////////
void ResourceSpawner::OnOwnerClicked(const QString &_owner)
{
  // This may take a few seconds, set waiting cursor
  this->dataPtr->displayData.ownerPath = _owner.toStdString();
  this->dataPtr->displayData.isFuel = true;
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
    gzwarn << "No paths found in GZ_SIM_RESOURCE_PATH.\n";
  }

  // Add all local paths found in `GZ_SIM_RESOURCE_PATH` to the qml list
  for (int i = 0; i < res.data_size(); i++)
  {
    const std::string path = res.data(i);
    this->AddPath(path);
  }

  gzmsg << "Please wait... Loading models from Fuel.\n";
  this->dataPtr->ownerModel.AddPath(kDefaultOwner);
  RunFetchResourceListThread(kDefaultOwner);
}

/////////////////////////////////////////////////
void ResourceSpawner::OnSearchEntered(const QString &_searchKeyword)
{
  this->dataPtr->displayData.searchKeyword =
    _searchKeyword.toStdString();
}

/////////////////////////////////////////////////
void ResourceSpawner::OnSortChosen(const QString &_sortType)
{
  this->dataPtr->displayData.sortMethod =
    _sortType.toStdString();
}

/////////////////////////////////////////////////
void ResourceSpawner::OnResourceSpawn(const QString &_sdfPath)
{
  gz::gui::events::SpawnFromPath event(_sdfPath.toStdString());
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);
}

/////////////////////////////////////////////////
void ResourceSpawner::UpdateOwnerListModel(Resource _resource)
{
  // If the resource is cached, we can go ahead and populate the
  // respective information
  std::string path;
  if (this->dataPtr->fuelClient->CachedModel(
        common::URI(_resource.sdfPath), path))
  {
    _resource.isDownloaded = true;
    _resource.sdfPath = common::joinPaths(path, "model.sdf");
    std::string thumbnailPath = common::joinPaths(path, "thumbnails");
    this->SetThumbnail(thumbnailPath, _resource);
  }

  this->dataPtr->ownerModelMap[_resource.owner].push_back(_resource);
  if (this->dataPtr->displayData.ownerPath == _resource.owner)
  {
    this->dataPtr->resourceModel.AddResource(_resource);
  }
}

/////////////////////////////////////////////////
bool ResourceSpawner::AddOwner(const QString &_owner)
{
  const std::string ownerString = _owner.toStdString();
  if (this->dataPtr->ownerModelMap.find(ownerString) !=
      this->dataPtr->ownerModelMap.end())
  {
    QString errorMsg = QString("Owner %1 already added").arg(_owner);
    emit resourceSpawnerError(errorMsg);
    return false;
  }
  this->dataPtr->ownerModel.AddPath(ownerString);
  RunFetchResourceListThread(ownerString);
  return true;
}

/////////////////////////////////////////////////
void ResourceSpawner::RemoveOwner(const QString &_owner)
{
  const std::string ownerString = _owner.toStdString();
  this->dataPtr->ownerModelMap.erase(ownerString);
  this->dataPtr->ownerModel.RemovePath(ownerString);
  this->dataPtr->fetchResourceListWorkers[ownerString].stopDownloading = true;
}

/////////////////////////////////////////////////
bool ResourceSpawner::IsDefaultOwner(const QString &_owner) const
{
  return _owner.toStdString() == kDefaultOwner;
}

/////////////////////////////////////////////////
void ResourceSpawner::RunFetchResourceListThread(const std::string &_owner)
{
  auto &worker = this->dataPtr->fetchResourceListWorkers[_owner];
  // If the owner had been deleted, we need to clean the previous thread and
  // restart.
  if (worker.thread.joinable())
  {
    worker.stopDownloading = true;
    worker.thread.join();
  }

  worker.stopDownloading = false;

  // Pull in fuel models asynchronously
  this->dataPtr->fetchResourceListWorkers[_owner].thread = std::thread(
      [this, owner = _owner, &worker]
      {
        int counter = 0;
        for (auto const &server : this->dataPtr->servers)
        {
          fuel_tools::ModelIdentifier modelId;
          modelId.SetServer(server);
          modelId.SetOwner(owner);
          for (auto iter = worker.fuelClient.Models(modelId, false);
               iter; ++iter, ++counter)
          {
            if (worker.stopDownloading)
            {
              return;
            }
            auto id = iter->Identification();
            Resource resource;
            resource.name = id.Name();
            resource.isFuel = true;
            resource.isDownloaded = false;
            resource.owner = id.Owner();
            resource.sdfPath = id.Url().Str();

            QMetaObject::invokeMethod(
                this, "UpdateOwnerListModel", Qt::QueuedConnection,
                Q_ARG(gz::sim::Resource, resource));
          }
        }
        if (counter == 0)
        {
          QString errorMsg = QString("No resources found for %1")
                                 .arg(QString::fromStdString(owner));
          emit resourceSpawnerError(errorMsg);
        }
      });
}

// Register this plugin
GZ_ADD_PLUGIN(ResourceSpawner,
              gz::gui::Plugin)
