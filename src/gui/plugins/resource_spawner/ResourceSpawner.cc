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
#include <atomic>
#include <cctype>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <QDir>
#include <QNetworkAccessManager>
#include <QNetworkDiskCache>
#include <QSettings>
#include <QStandardPaths>
#include <QUrl>

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
    /// fetches the list of resources available for an owner on Fuel.
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

    /// \brief Model holding the most recently spawned resources.
    public: ResourceModel recentModel;

    /// \brief Backing list of recently spawned resources (newest first).
    public: std::vector<Resource> recentResources;

    /// \brief Set of favorite resource keys ("owner/name").
    public: std::unordered_set<std::string> favorites;

    /// \brief Set of currently downloading resource keys ("owner/name").
    public: std::unordered_set<std::string> activeDownloads;

    /// \brief Background threads performing Fuel model downloads.
    public: std::vector<std::thread> downloadThreads;

    /// \brief Set to true during destructor so background workers do not
    /// invoke slots on a destroyed object.
    public: std::atomic<bool> shuttingDown{false};

    /// \brief Build a canonical key for a resource from owner and name.
    public: static std::string ResourceKey(const std::string &_owner,
        const std::string &_name)
    {
      return _owner + "/" + _name;
    }

    /// \brief Load persisted favorites and recent spawns from QSettings.
    public: void LoadPersistedState()
    {
      QSettings settings("OpenRobotics", "GazeboResourceSpawner");
      const QStringList favList = settings.value("favorites").toStringList();
      for (const QString &key : favList)
        this->favorites.insert(key.toStdString());

      const int count = settings.beginReadArray("recent");
      for (int i = 0; i < count; ++i)
      {
        settings.setArrayIndex(i);
        Resource res;
        res.name = settings.value("name").toString().toStdString();
        res.owner = settings.value("owner").toString().toStdString();
        res.sdfPath = settings.value("sdfPath").toString().toStdString();
        res.thumbnailUrl =
          settings.value("thumbnailUrl").toString().toStdString();
        res.isFuel = !res.owner.empty();
        res.isDownloaded = true;
        res.isFavorite = this->favorites.count(
            ResourceKey(res.owner, res.name)) > 0;
        if (!res.sdfPath.empty() && common::exists(res.sdfPath))
          this->recentResources.push_back(res);
      }
      settings.endArray();
      this->recentModel.Clear();
      this->recentModel.AddResources(this->recentResources);
    }

    /// \brief Persist favorites to QSettings.
    public: void SaveFavorites() const
    {
      QSettings settings("OpenRobotics", "GazeboResourceSpawner");
      QStringList favList;
      favList.reserve(static_cast<int>(this->favorites.size()));
      for (const std::string &key : this->favorites)
        favList.append(QString::fromStdString(key));
      settings.setValue("favorites", favList);
    }

    /// \brief Persist recent spawns to QSettings.
    public: void SaveRecents() const
    {
      QSettings settings("OpenRobotics", "GazeboResourceSpawner");
      settings.beginWriteArray("recent");
      for (size_t i = 0; i < this->recentResources.size(); ++i)
      {
        settings.setArrayIndex(static_cast<int>(i));
        settings.setValue("name",
            QString::fromStdString(this->recentResources[i].name));
        settings.setValue("owner",
            QString::fromStdString(this->recentResources[i].owner));
        settings.setValue("sdfPath",
            QString::fromStdString(this->recentResources[i].sdfPath));
        settings.setValue("thumbnailUrl",
            QString::fromStdString(this->recentResources[i].thumbnailUrl));
      }
      settings.endArray();
    }
  };
}

namespace {

// Default owner to be fetched from Fuel. This owner cannot be removed.
constexpr const char *kDefaultOwner = "openrobotics";

// Maximum number of recently spawned resources kept in the quick strip.
constexpr size_t kMaxRecentResources = 5;

// Maximum disk cache size for Fuel thumbnails (50 MiB).
constexpr qint64 kThumbnailCacheMaxBytes = 50 * 1024 * 1024;
}
using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
PathModel::PathModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
PathModel::~PathModel()
{
  // Disconnect all signals/slots manually. This prevents Qt from printing
  // warnings when closing the plugin as it tries to disconnect signals/slots
  // from the (already deleted) model.
  this->disconnect();
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
ResourceModel::~ResourceModel()
{
  // Disconnect all signals/slots manually. This prevents Qt from printing
  // warnings when closing the plugin as it tries to disconnect signals/slots
  // from the (already deleted) model.
  this->disconnect();
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
  const auto roles = this->roleNames();
  resource->setData(_resource.isFuel, roles.key("isFuel"));
  resource->setData(_resource.isDownloaded, roles.key("isDownloaded"));
  resource->setData(_resource.isDownloading, roles.key("isDownloading"));
  resource->setData(_resource.isFavorite, roles.key("isFavorite"));
  resource->setData(QString::fromStdString(_resource.thumbnailPath),
      roles.key("thumbnail"));
  resource->setData(QString::fromStdString(_resource.thumbnailUrl),
      roles.key("thumbnailUrl"));
  resource->setData(QString::fromStdString(_resource.name),
      roles.key("name"));
  resource->setData(QString::fromStdString(_resource.sdfPath),
      roles.key("sdf"));
  resource->setData(QString::fromStdString(_resource.owner),
      roles.key("owner"));
  resource->setData(QString::fromStdString(_resource.description),
      roles.key("description"));
  resource->setData(static_cast<qulonglong>(_resource.fileSize),
      roles.key("fileSize"));
  resource->setData(static_cast<uint>(_resource.downloadCount),
      roles.key("downloadCount"));
  resource->setData(static_cast<uint>(_resource.likeCount),
      roles.key("likeCount"));
  resource->setData(this->gridIndex, roles.key("index"));
  this->gridIndex++;
  emit sizeChanged();

  this->appendRow(resource);
}

/////////////////////////////////////////////////
void ResourceModel::UpdateResourceModel(int index, Resource &_resource)
{
  QStandardItem *parentItem = this->invisibleRootItem();
  auto resource = parentItem->child(index);
  if (!resource)
    return;

  const auto roles = this->roleNames();
  resource->setData(_resource.isFuel, roles.key("isFuel"));
  resource->setData(_resource.isDownloaded, roles.key("isDownloaded"));
  resource->setData(_resource.isDownloading, roles.key("isDownloading"));
  resource->setData(_resource.isFavorite, roles.key("isFavorite"));
  resource->setData(QString::fromStdString(_resource.thumbnailPath),
      roles.key("thumbnail"));
  resource->setData(QString::fromStdString(_resource.thumbnailUrl),
      roles.key("thumbnailUrl"));
  resource->setData(QString::fromStdString(_resource.sdfPath),
      roles.key("sdf"));
}

/////////////////////////////////////////////////
bool ResourceModel::UpdateResourceByName(const std::string &_owner,
    const std::string &_name, const Resource &_resource)
{
  const auto roles = this->roleNames();
  const int ownerRole = roles.key("owner");
  const int nameRole = roles.key("name");
  const QString qOwner = QString::fromStdString(_owner);
  const QString qName = QString::fromStdString(_name);

  QStandardItem *parentItem = this->invisibleRootItem();
  for (int i = 0; i < this->rowCount(); ++i)
  {
    QStandardItem *item = parentItem->child(i);
    if (!item)
      continue;
    if (item->data(ownerRole).toString() == qOwner &&
        item->data(nameRole).toString() == qName)
    {
      item->setData(_resource.isFuel, roles.key("isFuel"));
      item->setData(_resource.isDownloaded, roles.key("isDownloaded"));
      item->setData(_resource.isDownloading, roles.key("isDownloading"));
      item->setData(_resource.isFavorite, roles.key("isFavorite"));
      item->setData(QString::fromStdString(_resource.thumbnailPath),
          roles.key("thumbnail"));
      item->setData(QString::fromStdString(_resource.thumbnailUrl),
          roles.key("thumbnailUrl"));
      item->setData(QString::fromStdString(_resource.sdfPath),
          roles.key("sdf"));
      return true;
    }
  }
  return false;
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
    std::pair(107, "thumbnailUrl"),
    std::pair(108, "isDownloading"),
    std::pair(109, "isFavorite"),
    std::pair(110, "fileSize"),
    std::pair(111, "downloadCount"),
    std::pair(112, "likeCount"),
    std::pair(113, "description"),
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
      "RecentResourceList", &this->dataPtr->recentModel);
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
      "PathList", &this->dataPtr->pathModel);
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
      "OwnerList", &this->dataPtr->ownerModel);

  // Enable persistent HTTP disk caching for remote Fuel thumbnails if the
  // QML engine does not already have a disk cache configured.
  if (auto *nam = gz::gui::App()->Engine()->networkAccessManager())
  {
    if (!nam->cache())
    {
      const QString cacheDir =
          QStandardPaths::writableLocation(QStandardPaths::CacheLocation) +
          "/gz-fuel-thumbnails";
      QDir().mkpath(cacheDir);
      auto *diskCache = new QNetworkDiskCache(nam);
      diskCache->setCacheDirectory(cacheDir);
      diskCache->setMaximumCacheSize(kThumbnailCacheMaxBytes);
      nam->setCache(diskCache);
    }
  }

  this->dataPtr->LoadPersistedState();
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
  this->dataPtr->shuttingDown = true;
  for (auto &workers : this->dataPtr->fetchResourceListWorkers)
  {
    workers.second.stopDownloading = true;
    if (workers.second.thread.joinable())
    {
      workers.second.thread.join();
    }
  }
  for (auto &t : this->dataPtr->downloadThreads)
  {
    if (t.joinable())
      t.join();
  }
}

/////////////////////////////////////////////////
std::string ResourceSpawner::BuildFuelThumbnailUrl(
    const std::string &_serverUrl, const std::string &_serverVersion,
    const std::string &_owner, const std::string &_name)
{
  if (_serverUrl.empty() || _owner.empty() || _name.empty())
    return "";

  std::string base = _serverUrl;
  while (!base.empty() && base.back() == '/')
    base.pop_back();

  const std::string version = _serverVersion.empty() ? "1.0" : _serverVersion;
  const QByteArray encodedOwner =
      QUrl::toPercentEncoding(QString::fromStdString(_owner));
  const QByteArray encodedName =
      QUrl::toPercentEncoding(QString::fromStdString(_name));

  return base + "/" + version + "/" + encodedOwner.toStdString() +
      "/models/" + encodedName.toStdString() + "/tip/files/thumbnails/1.png";
}

/////////////////////////////////////////////////
void ResourceSpawner::SetThumbnail(const std::string &_thumbnailPath,
    Resource &_resource)
{
  if (!common::exists(_thumbnailPath) || !common::isDirectory(_thumbnailPath))
    return;

  std::vector<std::string> candidates;
  for (common::DirIter file(_thumbnailPath);
      file != common::DirIter(); ++file)
  {
    const std::string current(*file);
    if (!common::isFile(current))
      continue;

    const std::string fileName = common::basename(current);
    const auto dotPos = fileName.rfind('.');
    if (dotPos == std::string::npos || dotPos + 1 >= fileName.size())
      continue;

    std::string ext = fileName.substr(dotPos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (ext == "png" || ext == "jpg" || ext == "jpeg" || ext == "svg")
      candidates.push_back(current);
  }

  if (!candidates.empty())
  {
    // Sort deterministically so 1.png is consistently preferred over 2.png.
    std::sort(candidates.begin(), candidates.end());
    _resource.thumbnailPath = candidates.front();
    _resource.thumbnailUrl =
        QUrl::fromLocalFile(QString::fromStdString(_resource.thumbnailPath))
            .toString().toStdString();
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

  // Get the name and description of the model
  if (modelXml)
  {
    auto modelName = modelXml->FirstChildElement("name");
    if (modelName && modelName->GetText())
      resource.name = modelName->GetText();

    auto modelDesc = modelXml->FirstChildElement("description");
    if (modelDesc && modelDesc->GetText())
      resource.description = modelDesc->GetText();
  }
  std::string sdfPath = sdf::getModelFilePath(resourcePath);
  resource.sdfPath = sdfPath;
  resource.isFavorite = this->dataPtr->favorites.count(
      ResourceSpawnerPrivate::ResourceKey(resource.owner, resource.name)) > 0;

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
  if (this->dataPtr->displayData.searchKeyword.empty())
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
    const QString &_name, const QString &_owner, int /*index*/,
    bool _spawnAfterDownload)
{
  const std::string ownerStr = _owner.toStdString();
  const std::string nameStr = _name.toStdString();
  const std::string uriStr = _path.toStdString();
  const std::string key = ResourceSpawnerPrivate::ResourceKey(ownerStr,
      nameStr);

  if (this->dataPtr->activeDownloads.count(key) > 0)
    return;

  this->dataPtr->activeDownloads.insert(key);

  // Mark as downloading immediately in both cache and visible model.
  auto ownerIt = this->dataPtr->ownerModelMap.find(ownerStr);
  if (ownerIt != this->dataPtr->ownerModelMap.end())
  {
    for (auto &resource : ownerIt->second)
    {
      if (resource.name == nameStr)
      {
        resource.isDownloading = true;
        this->dataPtr->resourceModel.UpdateResourceByName(
            ownerStr, nameStr, resource);
        break;
      }
    }
  }

  // Run download asynchronously on a worker thread so the GUI stays responsive.
  this->dataPtr->downloadThreads.emplace_back(
      [this, uriStr, ownerStr, nameStr, _spawnAfterDownload]
      {
        fuel_tools::FuelClient workerClient;
        std::string localPath;
        const bool ok = workerClient.DownloadModel(
            common::URI(uriStr, true), localPath);

        if (this->dataPtr->shuttingDown)
          return;

        QMetaObject::invokeMethod(
            this, "OnDownloadFinished", Qt::QueuedConnection,
            Q_ARG(QString, QString::fromStdString(ownerStr)),
            Q_ARG(QString, QString::fromStdString(nameStr)),
            Q_ARG(QString, QString::fromStdString(localPath)),
            Q_ARG(bool, ok),
            Q_ARG(bool, _spawnAfterDownload));
      });
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFinished(const QString &_owner,
    const QString &_name, const QString &_localPath, bool _success,
    bool _spawnAfterDownload)
{
  const std::string ownerStr = _owner.toStdString();
  const std::string nameStr = _name.toStdString();
  const std::string localPathStr = _localPath.toStdString();
  const std::string key = ResourceSpawnerPrivate::ResourceKey(ownerStr,
      nameStr);
  this->dataPtr->activeDownloads.erase(key);

  std::string spawnedSdf;
  std::string spawnedThumbUrl;

  auto ownerIt = this->dataPtr->ownerModelMap.find(ownerStr);
  if (ownerIt != this->dataPtr->ownerModelMap.end())
  {
    for (auto &resource : ownerIt->second)
    {
      if (resource.name == nameStr)
      {
        resource.isDownloading = false;
        if (_success)
        {
          resource.isDownloaded = true;
          resource.isFuel = true;
          std::string sdfPath = sdf::getModelFilePath(localPathStr);
          if (sdfPath.empty())
            sdfPath = common::joinPaths(localPathStr, "model.sdf");
          resource.sdfPath = sdfPath;
          const std::string thumbnailPath =
              common::joinPaths(localPathStr, "thumbnails");
          this->SetThumbnail(thumbnailPath, resource);
          spawnedSdf = resource.sdfPath;
          spawnedThumbUrl = resource.thumbnailUrl;
        }
        this->dataPtr->resourceModel.UpdateResourceByName(
            ownerStr, nameStr, resource);
        break;
      }
    }
  }

  if (!_success)
  {
    gzwarn << "Download failed for [" << ownerStr << "/" << nameStr
           << "]. Try again." << std::endl;
    emit resourceSpawnerError(
        QString("Failed to download \"%1\". Please try again.").arg(_name));
    return;
  }

  if (_spawnAfterDownload && !spawnedSdf.empty())
  {
    this->RecordRecentSpawn(QString::fromStdString(spawnedSdf),
        _name, _owner, QString::fromStdString(spawnedThumbUrl));
    this->OnResourceSpawn(QString::fromStdString(spawnedSdf));
  }
}

/////////////////////////////////////////////////
void ResourceSpawner::ToggleFavorite(const QString &_owner,
    const QString &_name)
{
  const std::string ownerStr = _owner.toStdString();
  const std::string nameStr = _name.toStdString();
  const std::string key = ResourceSpawnerPrivate::ResourceKey(ownerStr,
      nameStr);

  const bool nowFav = this->dataPtr->favorites.count(key) == 0;
  if (nowFav)
    this->dataPtr->favorites.insert(key);
  else
    this->dataPtr->favorites.erase(key);

  this->dataPtr->SaveFavorites();

  auto ownerIt = this->dataPtr->ownerModelMap.find(ownerStr);
  if (ownerIt != this->dataPtr->ownerModelMap.end())
  {
    for (auto &resource : ownerIt->second)
    {
      if (resource.name == nameStr)
      {
        resource.isFavorite = nowFav;
        this->dataPtr->resourceModel.UpdateResourceByName(
            ownerStr, nameStr, resource);
        break;
      }
    }
  }
  else
  {
    // Local resource: update visible model directly if present.
    Resource updated;
    updated.owner = ownerStr;
    updated.name = nameStr;
    // Preserve existing fields when updating local items in resourceModel.
    const auto roles = this->dataPtr->resourceModel.roleNames();
    const int ownerRole = roles.key("owner");
    const int nameRole = roles.key("name");
    const int favRole = roles.key("isFavorite");
    auto *root = this->dataPtr->resourceModel.invisibleRootItem();
    for (int i = 0; i < this->dataPtr->resourceModel.rowCount(); ++i)
    {
      if (auto *item = root->child(i))
      {
        if (item->data(ownerRole).toString() == _owner &&
            item->data(nameRole).toString() == _name)
        {
          item->setData(nowFav, favRole);
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
bool ResourceSpawner::IsFavorite(const QString &_owner,
    const QString &_name) const
{
  return this->dataPtr->favorites.count(
      ResourceSpawnerPrivate::ResourceKey(
          _owner.toStdString(), _name.toStdString())) > 0;
}

/////////////////////////////////////////////////
void ResourceSpawner::RecordRecentSpawn(const QString &_sdfPath,
    const QString &_name, const QString &_owner, const QString &_thumbnailUrl)
{
  const std::string sdfStr = _sdfPath.toStdString();
  if (sdfStr.empty())
    return;

  auto &recents = this->dataPtr->recentResources;
  recents.erase(
      std::remove_if(recents.begin(), recents.end(),
          [&sdfStr](const Resource &_r) { return _r.sdfPath == sdfStr; }),
      recents.end());

  Resource entry;
  entry.sdfPath = sdfStr;
  entry.name = _name.isEmpty() ? common::basename(sdfStr) : _name.toStdString();
  entry.owner = _owner.toStdString();
  entry.thumbnailUrl = _thumbnailUrl.toStdString();
  entry.isFuel = !entry.owner.empty();
  entry.isDownloaded = true;
  entry.isFavorite = this->IsFavorite(_owner, _name);

  recents.insert(recents.begin(), entry);
  if (recents.size() > kMaxRecentResources)
    recents.resize(kMaxRecentResources);

  this->dataPtr->recentModel.Clear();
  this->dataPtr->recentModel.AddResources(recents);
  this->dataPtr->SaveRecents();
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
    std::string sdfPath = sdf::getModelFilePath(path);
    if (sdfPath.empty())
      sdfPath = common::joinPaths(path, "model.sdf");
    _resource.sdfPath = sdfPath;
    std::string thumbnailPath = common::joinPaths(path, "thumbnails");
    this->SetThumbnail(thumbnailPath, _resource);
  }

  const std::string key = ResourceSpawnerPrivate::ResourceKey(
      _resource.owner, _resource.name);
  _resource.isFavorite = this->dataPtr->favorites.count(key) > 0;
  _resource.isDownloading = this->dataPtr->activeDownloads.count(key) > 0;

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
            resource.description = id.Description();
            resource.fileSize = id.FileSize();
            resource.downloadCount = id.DownloadCount();
            resource.likeCount = id.LikeCount();
            resource.thumbnailUrl = ResourceSpawner::BuildFuelThumbnailUrl(
                server.Url().Str(), server.Version(), id.Owner(), id.Name());

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
