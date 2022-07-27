/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "gz/sim/gui/EnvironmentalDataLoader.hh"

#include "gz/sim/components/EnvironmentalData.hh"

#include <atomic>
#include <mutex>
#include <string>

#include <gz/common/CSVFile.hh>

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
/// \brief Private data class for EnvironmentalDataLoader
class EnvironmentalDataLoaderPrivate
{
  public: QStringList dimensionsList;

  public: std::string dataPath{};

  public: int timeIndex{-1};

  public: int xIndex{-1};

  public: int yIndex{-1};

  public: int zIndex{-1};

  public: std::mutex mutex;

  public: std::atomic<bool> needsUpdate{false};
};
}
}
}

EnvironmentalDataLoader::EnvironmentalDataLoader()
  : GuiSystem(), dataPtr(new EnvironmentalDataLoaderPrivate)
{
  gui::App()->Engine()->rootContext()->setContextProperty(
      "EnvironmentalDataLoader", this);
}

EnvironmentalDataLoader::~EnvironmentalDataLoader()
{
}

void EnvironmentalDataLoader::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Environmental Data Loader";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

void EnvironmentalDataLoader::Update(const UpdateInfo &, EntityComponentManager &_ecm)
{
  if (this->dataPtr->needsUpdate)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    using ComponentT = components::EnvironmentalData;
    auto data = common::IO<ComponentT::Type>::ReadFrom(
        common::CSVFile(this->dataPtr->dataPath), this->dataPtr->timeIndex,
        {this->dataPtr->xIndex, this->dataPtr->yIndex, this->dataPtr->zIndex});
    _ecm.CreateComponent<ComponentT>(worldEntity(_ecm), data);
    this->dataPtr->needsUpdate = false;
  }
}

void EnvironmentalDataLoader::ScheduleUpdate()
{
  this->dataPtr->needsUpdate = this->IsConfigured();
}

void EnvironmentalDataLoader::SetDataPath(QUrl _dataPath)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->dataPath = _dataPath.path().toStdString();

  const common::CSVFile dataFile(this->dataPtr->dataPath);
  const std::vector<std::string> &header = dataFile.Header();
  this->dataPtr->dimensionList.clear();
  this->dataPtr->dimensionList.reserve(header.size());
  for (const std::string &dimension : header)
  {
    this->dataPtr->dimensionList.push_back(
        QString::fromStdString(dimension));
  }
  this->DimensionListChanged();

  if (!this->dataPtr->dimensionList.empty())
  {
    this->dataPtr->timeIndex = std::min(
        this->dataPtr->dimensionList.size(), 0);
    this->dataPtr->xIndex = std::min(
        this->dataPtr->dimensionList.size(), 1);
    this->dataPtr->yIndex = std::min(
        this->dataPtr->dimensionList.size(), 2);
    this->dataPtr->zIndex = std::min(
        this->dataPtr->dimensionList.size(), 3);
  }
  else
  {
    this->dataPtr->timeIndex = -1;
    this->dataPtr->xIndex = -1;
    this->dataPtr->yIndex = -1;
    this->dataPtr->zIndex = -1;
  }

  this->TimeIndexChanged();
  this->XIndexChanged();
  this->YIndexChanged();
  this->ZIndexChanged();

  this->IsConfiguredChanged();
}

QStringList EnvironmentalDataLoader::DimensionList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->dimensionList;
}

int EnvironmentalDataLoader::XIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->xIndex;
}

void EnvironmentalDataLoader::SetXIndex(int _xIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->xIndex = _xIndex;
  this->XIndexChanged();
}

int EnvironmentalDataLoader::YIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->yIndex;
}

void EnvironmentalDataLoader::SetYIndex(int _yIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->yIndex = _yIndex;
  this->YIndexChanged();
}

int EnvironmentalDataLoader::ZIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->zIndex;
}

void EnvironmentalDataLoader::SetZIndex(int _zIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->zIndex = _zIndex;
  this->ZIndexChanged();
}

bool EnvironmentalDataLoader::IsConfigured() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return (
      !this->dataPtr->dataPath.empty() &&
      this->dataPtr->xIndex != -1 &&
      this->dataPtr->yIndex != -1 &&
      this->dataPtr->zIndex != -1);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EnvironmentalDataLoader, gz::gui::Plugin)
