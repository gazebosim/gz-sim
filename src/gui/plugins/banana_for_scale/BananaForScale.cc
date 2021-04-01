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

#include "BananaForScale.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <ignition/utils/SuppressWarning.hh>

#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/ClientConfig.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

namespace ignition::gazebo
{
  class BananaPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;

    /// \brief Fuel client
    public:  std::unique_ptr<ignition::fuel_tools::FuelClient>
             fuelClient {nullptr};

  };
}

const char kBanana[] =
  "https://fuel.ignitionrobotics.org/1.0/mjcarroll/models/banana for scale";

const char kBigBanana[] =
  "https://fuel.ignitionrobotics.org/1.0/mjcarroll/models/big banana for scale";

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
BananaForScale::BananaForScale()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<BananaPrivate>())
{
  this->dataPtr->fuelClient =
    std::make_unique<ignition::fuel_tools::FuelClient>();
}

/////////////////////////////////////////////////
BananaForScale::~BananaForScale() = default;

/////////////////////////////////////////////////
void BananaForScale::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Banana for Scale";

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void BananaForScale::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  ignition::common::URI modelUri;
  if (_mode == "banana")
  {
    modelUri = ignition::common::URI(kBanana);
  }
  else if (_mode == "bigbanana")
  {
    modelUri = ignition::common::URI(kBigBanana);
  }

  std::string path;
  std::string sdfPath;
  if (this->dataPtr->fuelClient->CachedModel(modelUri, path))
  {
    sdfPath = ignition::common::joinPaths(path, "model.sdf");
  }
  else
  {
    std::string localPath;
    auto result = this->dataPtr->fuelClient->DownloadModel(modelUri, localPath);
    sdfPath = ignition::common::joinPaths(localPath, "model.sdf");
  }

  ignition::gui::events::SpawnFromPath event(sdfPath);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &event);

  IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  ignition::gazebo::gui::events::SpawnPreviewPath oldEvent(sdfPath);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &oldEvent);
  IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::BananaForScale,
                    ignition::gui::Plugin)
