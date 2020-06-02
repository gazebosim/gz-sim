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

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class InsertModelPrivate;

  /// \brief Provides buttons for adding a box, sphere, or cylinder
  /// to the scene
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

    public: void FindLocalModels(const std::vector<std::string> &_paths);
    public: void FindLocalModels(const std::string &_path);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<InsertModelPrivate> dataPtr;
  };

  struct LocalModel
  {
    std::string configPath = "";
    std::string sdfPath = "";
    std::string thumbnailPath = "";
  };
}
}

#endif
