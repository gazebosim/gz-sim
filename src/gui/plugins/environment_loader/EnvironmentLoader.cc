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
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>

#include <gz/plugin/Register.hh>
#include <gz/msgs/entity_plugin_v.pb.h>

#include <atomic>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/data_load_options.pb.h>
#include <gz/msgs/Utility.hh>

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
const char* preload_plugin_name{
  "gz::sim::systems::EnvironmentPreload"};
const char* preload_plugin_filename{
  "gz-sim-environment-preload-system"};
using Units = msgs::DataLoadPathOptions_DataAngularUnits;
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

  /// \brief Index of data dimension to be used as units.
  public: QString unit{"radians"};

  public: using ReferenceT = math::SphericalCoordinates::CoordinateType;

  /// \brief Map of supported spatial references.
  public: const QMap<QString, ReferenceT> referenceMap{
    {QString("global"), math::SphericalCoordinates::GLOBAL},
    {QString("spherical"), math::SphericalCoordinates::SPHERICAL},
    {QString("ecef"), math::SphericalCoordinates::ECEF}};

  /// \brief Map of supported spatial units.
  public: const QMap<QString, Units>
    unitMap{
      {QString("degree"),
        Units::DataLoadPathOptions_DataAngularUnits_DEGREES},
      {QString("radians"),
        Units::DataLoadPathOptions_DataAngularUnits_RADIANS}
    };

  /// \brief Spatial reference.
  public: QString reference;

  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief Whether to attempt an environmental data load.
  public: std::atomic<bool> needsLoad{false};

  /// \brief Gz transport node
  public: transport::Node node;

  /// \brief publisher
  public: std::optional<transport::Node::Publisher> pub{std::nullopt};
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
  auto world = worldEntity(_ecm);

  if (!this->dataPtr->pub.has_value())
  {
    auto topic = transport::TopicUtils::AsValidTopic(
      scopedName(world, _ecm) + "/" + "environment");
    this->dataPtr->pub =
      {this->dataPtr->node.Advertise<msgs::DataLoadPathOptions>(topic)};
  }

  static bool warned = false;
  if (!this->dataPtr->pub->HasConnections() && !warned)
  {
    warned = true;
    gzwarn << "Could not find a subscriber for the environment. "
      << "Attempting to load environmental preload plugin."
      << std::endl;

    auto nameComp = _ecm.Component<components::Name>(world);
    if (nullptr == nameComp) {
      gzerr << "Failed to get world name" << std::endl;
      return;
    }
    auto worldName = nameComp->Data();
    msgs::EntityPlugin_V req;
    req.mutable_entity()->set_id(world);
    auto plugin = req.add_plugins();
    plugin->set_name(preload_plugin_name);
    plugin->set_filename(preload_plugin_filename);
    plugin->set_innerxml("");
    msgs::Boolean res;
    bool result;
    const unsigned int timeout = 5000;
    const auto service = transport::TopicUtils::AsValidTopic(
      "/world/" + worldName + "/entity/system/add");
    if (service.empty())
    {
      gzerr << "Unable to request " << service << std::endl;
      return;
    }

    if (this->dataPtr->node.Request(service, req, timeout, res, result))
    {
      gzdbg << "Added plugin successfully" << std::endl;
    }
    else
    {
      gzerr << "Failed to load plugin" << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void EnvironmentLoader::ScheduleLoad()
{
  if(this->IsConfigured() && this->dataPtr->pub.has_value())
  {
    msgs::DataLoadPathOptions data;
    data.set_path(this->dataPtr->dataPath.toStdString());
    data.set_time(
      this->dataPtr->dimensionList[this->dataPtr->timeIndex].toStdString());
    data.set_x(
      this->dataPtr->dimensionList[this->dataPtr->xIndex].toStdString());
    data.set_y(
      this->dataPtr->dimensionList[this->dataPtr->yIndex].toStdString());
    data.set_z(
      this->dataPtr->dimensionList[this->dataPtr->zIndex].toStdString());
    auto referenceFrame = this->dataPtr->referenceMap[this->dataPtr->reference];

    data.set_coordinate_type(msgs::ConvertCoord(referenceFrame));
    data.set_units(this->dataPtr->unitMap[this->dataPtr->unit]);

    this->dataPtr->pub->Publish(data);
  }
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
QStringList EnvironmentLoader::UnitList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->unitMap.keys();
}

/////////////////////////////////////////////////
QString EnvironmentLoader::Unit() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->unit;
}

/////////////////////////////////////////////////
void EnvironmentLoader::SetUnit(QString _unit)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->unit = _unit;
  }
  this->IsConfiguredChanged();
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
