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

#ifndef IGNITION_GAZEBO_GUI_RESOURCE_SPAWNER_HH_
#define IGNITION_GAZEBO_GUI_RESOURCE_SPAWNER_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class ResourceSpawnerPrivate;

  /// \brief Resource used to update the ResourceModel
  struct Resource
  {
    /// \brief The name of the local model
    std::string name = "";

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

  /// \brief Provides a model by which the resource spawner qml plugin pulls
  /// and updates from
  class PathModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit PathModel();

    /// \brief Destructor
    public: ~PathModel() override = default;

    /// \brief Add a local model to the grid view.
    /// param[in] _model The local model to be added
    public slots: void AddPath(const std::string &_path);

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;
  };

  /// \brief Provides a model by which the resource spawner qml plugin pulls
  /// and updates from
  class ResourceModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit ResourceModel();

    /// \brief Destructor
    public: ~ResourceModel() override = default;

    /// \brief Add a local resource to the grid view.
    /// param[in] _resource The local resource to be added
    public slots: void AddResource(Resource &_resource);

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

    // \brief Index to keep track of the position of each fuel resource,
    // used primarily to access currently loaded resources for updates
    public: int fuelGridIndex = 0;

    // \brief Index to keep track of the position of each local resource,
    // used primarily to access currently loaded resources for updates
    public: int localGridIndex = 0;
  };

  /// \brief Provides interface for communicating to backend for generation
  /// of local models
  class ResourceSpawner : public ignition::gui::Plugin
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

    /// \brief Loads a local model from an absolute path to a model.config,
    /// does nothing if a path not containing model.config is passed in
    /// \param[in] _path The path to search
    public: void LoadLocalResource(const std::string &_path);

    /// \brief Adds a path to the path list model.
    /// \param[in] _path The path to add
    public: void AddPath(const std::string &_path);

    /// \brief Recursively searches the provided path for all model.config's
    /// and populates a vector of local models with the information
    /// \param[in] _path The path to search
    public: void FindLocalResources(const std::string &_path);

    /// \brief Searches through the previously loaded fuel resources to locate
    /// the models belonging to the passed in owner.
    /// \param[in] _owner The name of the owner
    public: void FindFuelResources(const std::string &_owner);

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
    /// \param[in] index The index of the grid pane to update
    public slots: void OnDownloadFuelResource(const QString &_path, int index);

    /// \brief Finds a thumbnail on the provided thumbnail path and
    /// sets the model's thumbnail path attribute to it, no action is
    /// taken if no thumbnail is found.
    /// \param[in] _thumbnailPath The path to search for a thumbnail
    /// \param[in] _model The model to update with the thumbnail information
    public: void SetThumbnail(const std::string &_thumbnailPath,
                Resource &_resource);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ResourceSpawnerPrivate> dataPtr;
  };
}
}

#endif
