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

#include "EnvironmentLoader.hh"

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/components/Environment.hh>
#include <gz/sim/Util.hh>

#include <gz/plugin/Register.hh>

#include <atomic>
#include <mutex>
#include <string>
#include <utility>
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
/// \brief Private data class for EnvironmentLoader
class EnvironmentLoaderPrivate
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

  public: using ReferenceT = math::SphericalCoordinates::CoordinateType;

  /// \brief Map of supported spatial references.
  public: const QMap<QString, ReferenceT> referenceMap{
    {QString("global"), math::SphericalCoordinates::GLOBAL},
    {QString("spherical"), math::SphericalCoordinates::SPHERICAL},
    {QString("ecef"), math::SphericalCoordinates::ECEF}};

  /// \brief Spatial reference.
  public: QString reference;

  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief Whether to attempt an environmental data load.
  public: std::atomic<bool> needsLoad{false};
};
}
}
}

/////////////////////////////////////////////////
EnvironmentLoader::EnvironmentLoader()
  : GuiSystem(), dataPtr(new EnvironmentLoaderPrivate)
{
  gui::App()->Engine()->rootContext()->setContextProperty(
      "EnvironmentLoader", this);
}

/////////////////////////////////////////////////
EnvironmentLoader::~EnvironmentLoader()
{
}

/////////////////////////////////////////////////
void EnvironmentLoader::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Environment Loader";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void EnvironmentLoader::Update(const UpdateInfo &,
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
      using ComponentDataT = components::EnvironmentalData;
      auto data = ComponentDataT::MakeShared(
          common::IO<ComponentDataT::FrameT>::ReadFrom(
              common::CSVIStreamIterator(dataFile),
              common::CSVIStreamIterator(),
              this->dataPtr->timeIndex, {
                static_cast<size_t>(this->dataPtr->xIndex),
                static_cast<size_t>(this->dataPtr->yIndex),
                static_cast<size_t>(this->dataPtr->zIndex)}),
          this->dataPtr->referenceMap[this->dataPtr->reference]);

      using ComponentT = components::Environment;
      _ecm.CreateComponent(worldEntity(_ecm), ComponentT{std::move(data)});
    }
    catch (const std::invalid_argument &exc)
    {
      gzerr << "Failed to load environmental data" << std::endl
            << exc.what() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void EnvironmentLoader::ScheduleLoad()
{
  this->dataPtr->needsLoad = this->IsConfigured();
}

/////////////////////////////////////////////////
QString EnvironmentLoader::DataPath() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->dataPath;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetDataUrl(QUrl _dataUrl)
{
  this->SetDataPath(_dataUrl.path());
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetDataPath(QString _dataPath)
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
QStringList EnvironmentLoader::DimensionList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->dimensionList;
}

/////////////////////////////////////////////////
int EnvironmentLoader::TimeIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->timeIndex;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetTimeIndex(int _timeIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->timeIndex = _timeIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentLoader::XIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->xIndex;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetXIndex(int _xIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->xIndex = _xIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentLoader::YIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->yIndex;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetYIndex(int _yIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->yIndex = _yIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
int EnvironmentLoader::ZIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->zIndex;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetZIndex(int _zIndex)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->zIndex = _zIndex;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
QStringList EnvironmentLoader::ReferenceList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->referenceMap.keys();
}

/////////////////////////////////////////////////
QString EnvironmentLoader::Reference() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->reference;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetReference(QString _reference)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->reference = _reference;
  }
  this->IsConfiguredChanged();
}

/////////////////////////////////////////////////
bool EnvironmentLoader::IsConfigured() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return (
      !this->dataPtr->dataPath.isEmpty() &&
      this->dataPtr->timeIndex != -1 &&
      this->dataPtr->xIndex != -1 &&
      this->dataPtr->yIndex != -1 &&
      this->dataPtr->zIndex != -1 &&
      !this->dataPtr->reference.isEmpty());
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EnvironmentLoader, gz::gui::Plugin)
