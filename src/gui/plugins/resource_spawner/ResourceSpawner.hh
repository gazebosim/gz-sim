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

#ifndef GZ_SIM_GUI_RESOURCE_SPAWNER_HH_
#define GZ_SIM_GUI_RESOURCE_SPAWNER_HH_

#include <qobjectdefs.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <gz/fuel_tools/ClientConfig.hh>
#include <gz/fuel_tools/FuelClient.hh>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class ResourceSpawnerPrivate;

  /// \brief Resource used to update the ResourceModel
  struct Resource
  {
    /// \brief The name of the resource.
    std::string name = "";

    /// \brief The owner of the resource, if the resource is local,
    /// owner will be empty.
    std::string owner = "";

    /// \brief The absolute path to the sdf corresponding to the local model
    std::string sdfPath = "";

    /// \brief The absolute path to the thumbnail of the local model, will be
    /// empty if no thumbnail is found
    std::string thumbnailPath = "";

    /// \brief Bool to indicate if this model is fuel or not
    // cppcheck-suppress unusedStructMember
    bool isFuel = false;

    /// \brief Bool to indicate if this model has been downloaded or not, will
    /// always be false with local models as it is irrelevant in this case
    // cppcheck-suppress unusedStructMember
    bool isDownloaded = false;
  };

  /// \brief Data used by the `DisplayData()` function to filter and sort
  /// the resources to be displayed.
  struct Display
  {
    /// \brief The currently entered keyword that the user wants to search,
    /// empty if there is currently no search query.
    std::string searchKeyword = "";

    /// \brief The currently chosen method of sorting, which includes "A - Z",
    /// "Z - A", "Most Recent", and "Downloaded."  The default sort method is
    /// "Most Recent" as that is the order the fuel models are initially loaded.
    std::string sortMethod = "";

    /// \brief The name of the owner if the user has Fuel resources chosen,
    /// and the name of the local path if the user has local resources chosen.
    std::string ownerPath = "";

    /// \brief True if the user is currently observing fuel resources and false
    /// if the user is currently observing local resources.
    // cppcheck-suppress unusedStructMember
    bool isFuel = false;
  };

  /// \brief Provides a model by which the resource spawner qml plugin pulls
  /// and updates from
  class PathModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit PathModel();

    /// \brief Destructor
    public: ~PathModel() override = default;

    /// \brief Add a path.
    /// param[in] _path The path to be added.
    public: void AddPath(const std::string &_path);

    /// \brief Remove a path.
    /// param[in] _path The path to be removed.
    public: void RemovePath(const std::string &_path);

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;
  };

  /// \brief Provides a model by which the resource spawner qml plugin pulls
  /// and updates from
  class ResourceModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Property used to display the total number of resources associated
    /// with an owner.
    Q_PROPERTY(int totalCount MEMBER gridIndex NOTIFY sizeChanged)

    /// \brief Constructor
    public: explicit ResourceModel();

    /// \brief Destructor
    public: ~ResourceModel() override = default;

    /// \brief Add a resource to the grid view.
    /// param[in] _resource The local resource to be added
    public: void AddResource(const Resource &_resource);

    /// \brief Add a vector of resources to the grid view.
    /// param[in] _resource The vector of local resources to be added
    public: void AddResources(std::vector<Resource> &_resources);

    /// \brief Clear the current resource model
    public: void Clear();

    /// \brief Updates the resource at the provided index with the values in
    /// the passed in resource.
    /// \param[in] index The index of the resources within the resource model
    /// \param[in] _resource The resource values with which to update the
    /// existing resource
    public: void UpdateResourceModel(int index, Resource &_resource);

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Signal used with the totalCount property
    public: signals: void sizeChanged();

    // \brief Index to keep track of the position of each resource in the qml
    // grid, used primarily to access currently loaded resources for updates.
    public: int gridIndex = 0;
  };

  /// \brief Provides interface for communicating to backend for generation
  /// of local models
  class ResourceSpawner : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: ResourceSpawner();

    /// \brief Destructor
    public: ~ResourceSpawner() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback when a resource is selected.
    /// \param[in] _sdfPath The absolute path to the resource's sdf file
    public slots: void OnResourceSpawn(const QString &_sdfPath);

    /// \brief Returns the resource corresponding to the model.config file
    /// \param[in] _path The path of the model.config file
    /// \return The local resource
    public: Resource LocalResource(const std::string &_path);

    /// \brief Adds a path to the path list model.
    /// \param[in] _path The path to add
    public: void AddPath(const std::string &_path);

    /// \brief Returns the local resources as a vector located under
    /// the passed in path.
    /// \param[in] _path The path to search
    /// \return The vector of resources
    public: std::vector<Resource> LocalResources(const std::string &_path);

    /// \brief Returns the fuel resources as a vector belonging to the
    /// passed in owner.
    /// \param[in] _owner The name of the owner
    /// \return The vector of resources
    public: std::vector<Resource> FuelResources(const std::string &_owner);

    /// \brief Populates the passed in `_resources` vector with the
    /// currently selected group of resources.
    /// \param[in,out] _resources The vector of resources to populate
    public: void Resources(std::vector<Resource> &_resources);

    /// \brief Filters the vector of resources by the previously entered
    /// search keyword.
    /// \param[in,out] _resources The vector of resources to filter
    public: void FilterResources(std::vector<Resource> &_resources);

    /// \brief Sorts the vector of resources by the previously entered
    /// sort method.  The sorting types as a string, are "Most Recent",
    /// "A - Z", "Z - A", and "Downloaded." The sort defaults to
    /// "Most Recent."
    /// \param[in,out] _resources The vector of resources to sort
    public: void SortResources(std::vector<Resource> &_resources);

    /// \brief Displays the resources to the qml grid abiding by all search
    /// and sort criteria.
    public slots: void DisplayResources();

    /// \brief Callback when a resource path is selected, will clear the
    /// currently loaded resources and load the ones at the specified path
    /// \param[in] _path The path to search resources
    public slots: void OnPathClicked(const QString &_path);

    /// \brief Callback when a fuel owner is selected, will clear the
    /// currently loaded resources and load the ones belonging to the
    /// specified owner.
    /// \param[in] _owner The name of the owner
    public slots: void OnOwnerClicked(const QString &_owner);

    /// \brief Callback when a request is made to download a fuel resource.
    /// \param[in] _path URI to the fuel resource
    /// \param[in] _name Name of the resource
    /// \param[in] _owner The name of the owner
    /// \param[in] index The index of the grid pane to update
    public slots: void OnDownloadFuelResource(const QString &_path,
        const QString &_name, const QString &_owner, int index);

    /// \brief Callback when a sort request is made.
    /// \param[in] _sortType The sorting type as a string, accepts
    /// "Most Recent", "A - Z", "Z - A", and "Downloaded." Defaults to
    /// "Most Recent."
    public slots: void OnSortChosen(const QString &_sortType);

    /// \brief Callback when a search request is made.
    /// \param[in] _searchKeyword The search keyword, applies to either the
    /// resource's name or owner
    public slots: void OnSearchEntered(const QString &_searchKeyword);

    /// \brief Finds a thumbnail on the provided thumbnail path and
    /// sets the model's thumbnail path attribute to it, no action is
    /// taken if no thumbnail is found.
    /// \param[in] _thumbnailPath The path to search for a thumbnail
    /// \param[in] _resource The model to update with the thumbnail information
    public: void SetThumbnail(const std::string &_thumbnailPath,
                Resource &_resource);

    /// \brief Called form a download thread to update the GUI's list of
    /// resources.
    /// \param[in] _resource The resource fetched from Fuel. Note that it is
    /// passed by value as a copy is necessary to update the resource if it's
    /// cached.
    public: Q_INVOKABLE void UpdateOwnerListModel(gz::sim::Resource _resource);

    /// \brief Add owner to the list of owners whose resources would be fetched
    /// from Fuel.
    /// \param[in] _owner Name of owner.
    /// \return True if the owner was successfully added.
    public: Q_INVOKABLE bool AddOwner(const QString &_owner);

    /// \brief Remove owner from the list of owners whose resources would be
    /// fetched from Fuel.
    /// \param[in] _owner Name of owner.
    public: Q_INVOKABLE void RemoveOwner(const QString &_owner);

    /// \brief Determine if owner is the default owner
    /// \param[in] _owner Name of owner.
    public: Q_INVOKABLE bool IsDefaultOwner(const QString &_owner) const;

    /// \brief Signal emitted when an error is encountered regarding an owner
    /// \param[in] _errorMsg Error message to be displayed.
    signals: void resourceSpawnerError(const QString &_errorMsg);

    /// \brief Starts a thread that fetches the resources list for a given owner
    /// \param[in] _owner Name of owner.
    private: void RunFetchResourceListThread(const std::string &_owner);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ResourceSpawnerPrivate> dataPtr;
  };
}
}

#endif
