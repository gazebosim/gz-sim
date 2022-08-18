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

#include "EnvironmentalDataLoader.hh"

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/components/EnvironmentalData.hh>
#include <gz/sim/Util.hh>

#include <gz/plugin/Register.hh>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

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
  /// \brief Path to environmental data file to load.
  public: QString dataPath;

  /// \brief List of environmental data dimensions
  /// (ie. columns if dealing with CSV data).
  public: QStringList dimensionList;

  /// \brief Index of data dimension to be used as time.
  public: int timeIndex{-1};

  /// \brief Index of data dimension to be used as x coordinate.
  public: int xIndex{-1};

  /// \brief Index of data dimension to be used as y coordinate.
  public: int yIndex{-1};

  /// \brief Index of data dimension to be used as z coordinate.
  public: int zIndex{-1};

  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief Whether to attempt an environmental data load.
  public: std::atomic<bool> needsLoad{false};
};
}
}
}

/////////////////////////////////////////////////
EnvironmentalDataLoader::EnvironmentalDataLoader()
  : GuiSystem(), dataPtr(new EnvironmentalDataLoaderPrivate)
{
  gui::App()->Engine()->rootContext()->setContextProperty(
      "EnvironmentalDataLoader", this);
}

/////////////////////////////////////////////////
EnvironmentalDataLoader::~EnvironmentalDataLoader()
{
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Environmental Data Loader";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::Update(const UpdateInfo &,
                                     EntityComponentManager &_ecm)
{
  if (this->dataPtr->needsLoad)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->needsLoad = false;

    std::ifstream dataFile(this->dataPtr->dataPath.toStdString());
    gzmsg << "Loading environmental data from "
          << this->dataPtr->dataPath.toStdString()
          << std::endl;
    try
    {
      using ComponentT = components::EnvironmentalData;
      auto component = ComponentT{common::IO<ComponentT::Type>::ReadFrom(
          common::CSVIStreamIterator(dataFile), common::CSVIStreamIterator(),
          this->dataPtr->timeIndex, {
            static_cast<size_t>(this->dataPtr->xIndex),
            static_cast<size_t>(this->dataPtr->yIndex),
            static_cast<size_t>(this->dataPtr->zIndex)})};
      _ecm.CreateComponent<ComponentT>(worldEntity(_ecm), component);
    }
    catch (const std::invalid_argument &exc)
    {
      gzerr << "Failed to load environmental data" << std::endl
            << exc.what() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::ScheduleLoad()
{
  this->dataPtr->needsLoad = this->IsConfigured();
}

/////////////////////////////////////////////////
QString EnvironmentalDataLoader::DataPath() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->dataPath;
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetDataUrl(QUrl _dataUrl)
{
  this->SetDataPath(_dataUrl.path());
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetDataPath(QString _dataPath)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->dataPath = _dataPath;

    std::ifstream dataFile(_dataPath.toStdString());
    if (!dataFile.is_open())
    {
      gzerr << "No environmental data file was found at "
            << this->dataPtr->dataPath.toStdString()
            << std::endl;
      this->dataPtr->dataPath.clear();
      return;
    }
    const common::CSVIStreamIterator iterator(dataFile);
    if (iterator == common::CSVIStreamIterator())
    {
      gzerr << "Failed to load environmental data at "
            << this->dataPtr->dataPath.toStdString()
            << std::endl;
      this->dataPtr->dataPath.clear();
      return;
    }
    const std::vector<std::string> &header = *iterator;
    this->dataPtr->dimensionList.clear();
    this->dataPtr->dimensionList.reserve(header.size());
    for (const std::string &dimension : header)
    {
      this->dataPtr->dimensionList.push_back(
          QString::fromStdString(dimension));
    }
  }

  this->DataPathChanged();
  this->DimensionListChanged();
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
QStringList EnvironmentalDataLoader::DimensionList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->dimensionList;
}

/////////////////////////////////////////////////
int EnvironmentalDataLoader::TimeIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->timeIndex;
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetTimeIndex(int _timeIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->timeIndex = _timeIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentalDataLoader::XIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->xIndex;
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetXIndex(int _xIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->xIndex = _xIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentalDataLoader::YIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->yIndex;
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetYIndex(int _yIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->yIndex = _yIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentalDataLoader::ZIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->zIndex;
}

/////////////////////////////////////////////////
void EnvironmentalDataLoader::SetZIndex(int _zIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->zIndex = _zIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
bool EnvironmentalDataLoader::IsConfigured() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return (
      !this->dataPtr->dataPath.isEmpty() &&
      this->dataPtr->timeIndex != -1 &&
      this->dataPtr->xIndex != -1 &&
      this->dataPtr->yIndex != -1 &&
      this->dataPtr->zIndex != -1);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EnvironmentalDataLoader, gz::gui::Plugin)
