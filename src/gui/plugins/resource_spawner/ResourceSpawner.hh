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

    /// \brief Add a resource to the grid view.
    /// param[in] _model The resource to be added
    public slots: void AddResource(Resource &_resource);

    /// \brief Clear the current resource model
    public: void Clear();

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;
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

    /// \brief Add a path to the QML grid view.
    /// \param[in] _path The path to be added
    public: void AddPath(const std::string &_path);

    /// \brief Recursively searches the provided path for all model.config's
    /// and populates a vector of local models with the information
    /// \param[in] _path The path to search
    public: void FindLocalResources(const std::string &_path);

    /// \brief Callback when a resource path is selected, will clear the
    /// currently loaded resources and load the ones at the specified path
    /// \param[in] _path The path to search resources
    public slots: void OnPathClicked(const QString &_path);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ResourceSpawnerPrivate> dataPtr;
  };
}
}

#endif
