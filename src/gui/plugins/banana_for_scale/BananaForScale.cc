/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <algorithm>
#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

#include <gz/fuel_tools/FuelClient.hh>
#include <gz/fuel_tools/ClientConfig.hh>

#include "gz/sim/gui/GuiEvents.hh"

namespace gz::sim
{
  class BananaPrivate
  {
    /// \brief Fuel client
    public:  std::unique_ptr<gz::fuel_tools::FuelClient>
             fuelClient {nullptr};

  };
}

const char kBanana[] =
  "https://fuel.gazebosim.org/1.0/mjcarroll/models/banana for scale";

const char kBigBanana[] =
  "https://fuel.gazebosim.org/1.0/mjcarroll/models/big banana for scale";

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
BananaForScale::BananaForScale()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<BananaPrivate>())
{
  this->dataPtr->fuelClient =
    std::make_unique<gz::fuel_tools::FuelClient>();
}

/////////////////////////////////////////////////
BananaForScale::~BananaForScale() = default;

/////////////////////////////////////////////////
void BananaForScale::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Banana for Scale";
}

/////////////////////////////////////////////////
void BananaForScale::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  gz::common::URI modelUri;
  if (_mode == "banana")
  {
    modelUri = gz::common::URI(kBanana);
  }
  else if (_mode == "bigbanana")
  {
    modelUri = gz::common::URI(kBigBanana);
  }

  std::string path;
  std::string sdfPath;
  if (this->dataPtr->fuelClient->CachedModel(modelUri, path))
  {
    sdfPath = gz::common::joinPaths(path, "model.sdf");
  }
  else
  {
    std::string localPath;
    auto result = this->dataPtr->fuelClient->DownloadModel(modelUri, localPath);
    sdfPath = gz::common::joinPaths(localPath, "model.sdf");
  }

  gz::gui::events::SpawnFromPath event(sdfPath);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::BananaForScale,
                    gz::gui::Plugin)
