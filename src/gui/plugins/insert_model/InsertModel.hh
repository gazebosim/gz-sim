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

#ifndef IGNITION_GAZEBO_GUI_INSERT_MODEL_HH_
#define IGNITION_GAZEBO_GUI_INSERT_MODEL_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class InsertModelPrivate;

  /// \brief Local model used to update the GridModel
  struct LocalModel
  {
    std::string configPath = "";
    std::string sdfPath = "";
    std::string thumbnailPath = "";
    std::string name = "";
  };

  /// \brief Provides a model by which the insert model qml plugin pulls
  /// and updates from
  class GridModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit GridModel();

    /// \brief Destructor
    public: ~GridModel() override = default;

    /// \brief Add a local model to the grid view.
    /// param[in] _model The local model to be added
    public slots: void AddLocalModel(LocalModel &_model);

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;
  };

  /// \brief Provides interface for communicating to backend for generation
  /// of local models
  class InsertModel : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: InsertModel();

    /// \brief Destructor
    public: ~InsertModel() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback in Qt thread when mode changes.
    /// \param[in] _mode New transform mode
    public slots: void OnMode(const QString &_mode);

    /// \brief Recursively searches the provided paths for all model.config's
    /// and populates a vector of local models with the information
    /// \param[in] _paths The paths to search
    public: void FindLocalModels(const std::vector<std::string> &_paths);

    /// \brief Recursively searches the provided path for all model.config's
    /// and populates a vector of local models with the information
    /// \param[in] _path The path to search
    public: void FindLocalModels(const std::string &_path);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<InsertModelPrivate> dataPtr;
  };
}
}

#endif
