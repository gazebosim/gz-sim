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
#include <exception>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <QThreadPool>
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

    /// \brief Set of currently downloading resource keys ("owner/name").
    public: std::unordered_set<std::string> activeDownloads;

    /// \brief Runs Fuel model downloads off the GUI thread. Each task only
    /// calls FuelClient::DownloadModel and posts the result back to the GUI
    /// thread.
    public: QThreadPool downloadPool;

    /// \brief Build a canonical key for a resource from owner and name.
    public: static std::string ResourceKey(const std::string &_owner,
        const std::string &_name)
    {
      return _owner + "/" + _name;
    }
  };
}

namespace {

// Default owner to be fetched from Fuel. This owner cannot be removed.
constexpr const char *kDefaultOwner = "openrobotics";

// Number of Fuel downloads that run at the same time. Downloads can't be
// cancelled, so this also bounds the wait when the plugin is closed.
constexpr int kMaxConcurrentDownloads = 2;
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
  this->InsertResource(this->rowCount(), _resource);
}

/////////////////////////////////////////////////
void ResourceModel::InsertResource(int _row, const Resource &_resource)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("GridModel::InsertResource");

  auto resource = new QStandardItem(QString::fromStdString(_resource.name));
  const auto roles = this->roleNames();
  resource->setData(_resource.isFuel, roles.key("isFuel"));
  resource->setData(_resource.isDownloaded, roles.key("isDownloaded"));
  resource->setData(_resource.isDownloading, roles.key("isDownloading"));
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

  this->insertRow(std::clamp(_row, 0, this->rowCount()), resource);
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
    std::pair(109, "fileSize"),
    std::pair(110, "downloadCount"),
    std::pair(111, "likeCount"),
    std::pair(112, "description"),
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

  this->dataPtr->downloadPool.setMaxThreadCount(kMaxConcurrentDownloads);

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

  // Fuel downloads can't be cancelled. Drop the ones that haven't started and
  // wait for the ones in progress, so no model is left half extracted in the
  // Fuel cache. Results posted to this object are discarded with it.
  this->dataPtr->downloadPool.clear();
  const int running = this->dataPtr->downloadPool.activeThreadCount();
  if (running > 0)
  {
    gzmsg << "Waiting for " << running << " Fuel download(s) to finish."
          << std::endl;
  }
  this->dataPtr->downloadPool.waitForDone();
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

namespace {
/////////////////////////////////////////////////
std::string ToLower(std::string _str)
{
  std::for_each(_str.begin(), _str.end(), [](char &_c)
      {
        _c = static_cast<char>(::tolower(static_cast<unsigned char>(_c)));
      });
  return _str;
}

/////////////////////////////////////////////////
/// \brief Whether the resource's name or owner contains the keyword.
/// \param[in] _resource Resource to check.
/// \param[in] _keyword Lowercase search keyword. An empty keyword matches
/// every resource.
bool MatchesKeyword(const Resource &_resource, const std::string &_keyword)
{
  return _keyword.empty() ||
      ToLower(_resource.name).find(_keyword) != std::string::npos ||
      ToLower(_resource.owner).find(_keyword) != std::string::npos;
}

/////////////////////////////////////////////////
/// \brief Whether _a is shown before _b with the given sort method.
/// Resources that are equal for the sort method keep the order in which
/// they were loaded, which is the "Most Recent" order for Fuel resources.
bool SortsBefore(const std::string &_sortMethod, const Resource &_a,
    const Resource &_b)
{
  if (_sortMethod == "Popular")
    return _a.downloadCount > _b.downloadCount;
  if (_sortMethod == "A - Z")
    return compareByAlphabet(_a, _b);
  if (_sortMethod == "Z - A")
    return compareByAlphabet(_b, _a);
  return false;
}
}  // namespace

/////////////////////////////////////////////////
void ResourceSpawner::FilterResources(std::vector<Resource> &_resources)
{
  // Remove any resources that don't contain the entered keyword in their
  // name or owner name
  const std::string keyword =
      ToLower(this->dataPtr->displayData.searchKeyword);
  _resources.erase(std::remove_if(_resources.begin(), _resources.end(),
      [&keyword](const Resource &_resource)
      {
        return !MatchesKeyword(_resource, keyword);
      }), _resources.end());
}

/////////////////////////////////////////////////
void ResourceSpawner::SortResources(std::vector<Resource> &_resources)
{
  const std::string &sortMethod = this->dataPtr->displayData.sortMethod;
  std::stable_sort(_resources.begin(), _resources.end(),
      [&sortMethod](const Resource &_a, const Resource &_b)
      {
        return SortsBefore(sortMethod, _a, _b);
      });
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
    auto &resources = ownerIt->second;
    auto it = std::find_if(resources.begin(), resources.end(),
        [&nameStr](const Resource &_resource)
        {
          return _resource.name == nameStr;
        });
    if (it != resources.end())
    {
      it->isDownloading = true;
      this->dataPtr->resourceModel.UpdateResourceByName(
          ownerStr, nameStr, *it);
    }
  }

  // Download off the GUI thread and report the result back on it.
  this->dataPtr->downloadPool.start(
      [this, uriStr, ownerStr, nameStr, _spawnAfterDownload]
      {
        std::string localPath;
        std::string error;
        bool success{false};
        try
        {
          fuel_tools::FuelClient client;
          const fuel_tools::Result result =
              client.DownloadModel(common::URI(uriStr, true), localPath);
          success = static_cast<bool>(result);
          if (!success)
            error = result.ReadableResult();
        }
        catch (const std::exception &_e)
        {
          error = _e.what();
        }

        QMetaObject::invokeMethod(this,
            [this, ownerStr, nameStr, localPath, success, error,
             _spawnAfterDownload]
            {
              this->OnDownloadFinished(ownerStr, nameStr, localPath, success,
                  error, _spawnAfterDownload);
            },
            Qt::QueuedConnection);
      });
}

/////////////////////////////////////////////////
void ResourceSpawner::OnDownloadFinished(const std::string &_owner,
    const std::string &_name, const std::string &_localPath, bool _success,
    const std::string &_error, bool _spawnAfterDownload)
{
  this->dataPtr->activeDownloads.erase(
      ResourceSpawnerPrivate::ResourceKey(_owner, _name));

  std::string spawnedSdf;

  auto ownerIt = this->dataPtr->ownerModelMap.find(_owner);
  if (ownerIt != this->dataPtr->ownerModelMap.end())
  {
    auto &resources = ownerIt->second;
    auto it = std::find_if(resources.begin(), resources.end(),
        [&_name](const Resource &_resource)
        {
          return _resource.name == _name;
        });
    if (it != resources.end())
    {
      it->isDownloading = false;
      if (_success)
      {
        it->isDownloaded = true;
        it->isFuel = true;
        std::string sdfPath = sdf::getModelFilePath(_localPath);
        if (sdfPath.empty())
          sdfPath = common::joinPaths(_localPath, "model.sdf");
        it->sdfPath = sdfPath;
        const std::string thumbnailPath =
            common::joinPaths(_localPath, "thumbnails");
        this->SetThumbnail(thumbnailPath, *it);
        spawnedSdf = it->sdfPath;
      }
      this->dataPtr->resourceModel.UpdateResourceByName(_owner, _name, *it);
    }
  }

  if (!_success)
  {
    gzerr << "Failed to download [" << _owner << "/" << _name << "]"
          << (_error.empty() ? "" : ": " + _error) << std::endl;
    const QString name = QString::fromStdString(_name);
    emit resourceSpawnerError(_error.empty() ?
        QString("Failed to download \"%1\". Please try again.").arg(name) :
        QString("Failed to download \"%1\": %2").arg(
            name, QString::fromStdString(_error)));
    return;
  }

  if (_spawnAfterDownload && !spawnedSdf.empty())
  {
    this->OnResourceSpawn(QString::fromStdString(spawnedSdf));
  }
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
  _resource.isDownloading = this->dataPtr->activeDownloads.count(key) > 0;

  auto &ownerResources = this->dataPtr->ownerModelMap[_resource.owner];
  const auto &display = this->dataPtr->displayData;
  const std::string keyword = ToLower(display.searchKeyword);
  if (display.ownerPath == _resource.owner &&
      MatchesKeyword(_resource, keyword))
  {
    // The owner is on screen while its resources are still arriving. Insert
    // the resource where DisplayResources would put it: after the shown
    // resources that don't sort after it.
    int row = 0;
    for (const Resource &resource : ownerResources)
    {
      if (MatchesKeyword(resource, keyword) &&
          !SortsBefore(display.sortMethod, _resource, resource))
      {
        ++row;
      }
    }
    this->dataPtr->resourceModel.InsertResource(row, _resource);
  }
  ownerResources.push_back(_resource);
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
