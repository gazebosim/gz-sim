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
#include "EnvironmentPreload.hh"
#include "VisualizationTool.hh"

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>
#include <gz/common/Filesystem.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/data_load_options.pb.h>
#include <gz/msgs/Utility.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

using Units = msgs::DataLoadPathOptions_DataAngularUnits;
//////////////////////////////////////////////////
class gz::sim::systems::EnvironmentPreloadPrivate
{
  /// \brief Is the file loaded
  public: bool loaded{false};

  /// \brief SDF Description
  public: std::shared_ptr<const sdf::Element> sdf;

  /// \brief GzTransport node
  public: transport::Node node;

  /// \brief Data descriptions
  public: msgs::DataLoadPathOptions dataDescription;

  /// \brief mutex to protect the samples and data description
  public: std::mutex mtx;

  /// \brief Do we need to reload the system.
  public: std::atomic<bool> needsReload{false};

  /// \brief Visualization Helper
  public: std::unique_ptr<EnvironmentVisualizationTool> visualizationPtr;

  /// \brief Are visualizations enabled
  public: bool visualize{false};

  /// \brief Sample resolutions
  public: math::Vector3<unsigned int> samples;

  /// \brief Is the file loaded
  public: bool fileLoaded{false};

  /// \brief File loading error logger
  public: bool logFileLoadError{true};

  /// \brief Reference to data
  public: std::shared_ptr<components::EnvironmentalData> envData;

  //////////////////////////////////////////////////
  public: EnvironmentPreloadPrivate() :
    visualizationPtr(new EnvironmentVisualizationTool) {}

  //////////////////////////////////////////////////
  public: void OnLoadCommand(const msgs::DataLoadPathOptions &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->dataDescription = _msg;
    this->needsReload = true;
    this->logFileLoadError = true;
    this->visualizationPtr->FileReloaded();
    gzdbg << "Loading file " << _msg.path() << "\n";
  }

  //////////////////////////////////////////////////
  public: void OnVisualResChanged(const msgs::Vector3d &_resChanged)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    if (!this->fileLoaded)
    {
      // Only visualize if a file exists
      return;
    }
    math::Vector3<unsigned int> converted{
      static_cast<unsigned int>(ceil(_resChanged.x())),
      static_cast<unsigned int>(ceil(_resChanged.y())),
      static_cast<unsigned int>(ceil(_resChanged.z()))};
    if (this->samples.X() == converted.X() &&
        this->samples.Y() == converted.Y() &&
        this->samples.Z() == converted.Z())
    {
      // If the sample has not changed return.
      // This is because resampling is expensive.
      return;
    }
    this->samples = converted;
    this->visualize = true;
    this->visualizationPtr->resample = true;
  }

  //////////////////////////////////////////////////
  public: void ReadSdf(EntityComponentManager &_ecm)
  {
    if (!this->sdf->HasElement("data"))
    {
      gzerr << "No environmental data file was specified" << std::endl;
      return;
    }

    std::lock_guard<std::mutex> lock(mtx);
    std::string dataPath =
        this->sdf->Get<std::string>("data");

    if (common::isRelativePath(dataPath))
    {
      auto *component =
          _ecm.Component<components::WorldSdf>(worldEntity(_ecm));
      const std::string rootPath =
          common::parentPath(component->Data().Element()->FilePath());
      dataPath = common::joinPaths(rootPath, dataPath);
    }
    this->dataDescription.set_path(dataPath);

    this->dataDescription.set_units(
      Units::DataLoadPathOptions_DataAngularUnits_RADIANS);
    std::string timeColumnName{"t"};
    bool ignoreTime = false;
    std::array<std::string, 3> spatialColumnNames{"x", "y", "z"};
    sdf::ElementConstPtr elem =
        this->sdf->FindElement("dimensions");
    msgs::SphericalCoordinatesType spatialReference =
      msgs::SphericalCoordinatesType::GLOBAL;
    if (elem)
    {
      if (elem->HasElement("ignore_time"))
      {
        ignoreTime = elem->Get<bool>("ignore_time");
      }
      if (elem->HasElement("time"))
      {
        timeColumnName = elem->Get<std::string>("time");
      }
      elem = elem->FindElement("space");
      if (elem)
      {
        if (elem->HasAttribute("reference"))
        {
          const std::string referenceName =
              elem->Get<std::string>("reference");
          if (referenceName == "global")
          {
            spatialReference = msgs::SphericalCoordinatesType::GLOBAL;
          }
          else if (referenceName == "spherical")
          {
            spatialReference = msgs::SphericalCoordinatesType::SPHERICAL;
            if (elem->HasAttribute("units"))
            {
              std::string unitName = elem->Get<std::string>("units");
              if (unitName == "degrees")
              {
                this->dataDescription.set_units(
                  Units::DataLoadPathOptions_DataAngularUnits_DEGREES);
              }
              else if (unitName != "radians")
              {
                gzerr << "Unrecognized unit " << unitName << "\n";
              }
            }
          }
          else if (referenceName == "ecef")
          {
            spatialReference = msgs::SphericalCoordinatesType::ECEF;
          }
          else
          {
            gzerr << "Unknown reference '" << referenceName << "'"
                  << std::endl;
            return;
          }
        }
        for (size_t i = 0; i < spatialColumnNames.size(); ++i)
        {
          if (elem->HasElement(spatialColumnNames[i]))
          {
            spatialColumnNames[i] =
                elem->Get<std::string>(spatialColumnNames[i]);
          }
        }
      }
    }

    this->dataDescription.set_static_time(ignoreTime);
    this->dataDescription.set_coordinate_type(spatialReference);
    this->dataDescription.set_time(timeColumnName);
    this->dataDescription.set_x(spatialColumnNames[0]);
    this->dataDescription.set_y(spatialColumnNames[1]);
    this->dataDescription.set_z(spatialColumnNames[2]);

    this->needsReload = true;
  }

  //////////////////////////////////////////////////
  public: components::EnvironmentalData::ReferenceUnits ConvertUnits(
    const Units &_unit)
  {
    switch (_unit)
    {
      case Units::DataLoadPathOptions_DataAngularUnits_DEGREES:
        return components::EnvironmentalData::ReferenceUnits::DEGREES;
      case Units::DataLoadPathOptions_DataAngularUnits_RADIANS:
        return components::EnvironmentalData::ReferenceUnits::RADIANS;
      default:
        gzerr << "Invalid unit conversion. Defaulting to radians." << std::endl;
        return components::EnvironmentalData::ReferenceUnits::RADIANS;
    }
  }

  //////////////////////////////////////////////////
  public: void LoadEnvironment(EntityComponentManager &_ecm)
  {
    try
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      std::array<std::string, 3> spatialColumnNames{
        this->dataDescription.x(),
        this->dataDescription.y(),
        this->dataDescription.z()};

      math::SphericalCoordinates::CoordinateType spatialReference =
        msgs::Convert(this->dataDescription.coordinate_type());
      auto units = this->ConvertUnits(this->dataDescription.units());

      std::ifstream dataFile(this->dataDescription.path());
      if (!dataFile.is_open())
      {
        if (this->logFileLoadError)
        {
          gzerr << "No environmental data file was found at " <<
            this->dataDescription.path() << std::endl;
          logFileLoadError = false;
        }
        return;
      }

      gzmsg << "Loading Environment Data " << this->dataDescription.path() <<
        std::endl;

      using ComponentDataT = components::EnvironmentalData;
      auto data = ComponentDataT::MakeShared(
          common::IO<ComponentDataT::FrameT>::ReadFrom(
              common::CSVIStreamIterator(dataFile),
              common::CSVIStreamIterator(),
              this->dataDescription.time(), spatialColumnNames),
          spatialReference, units, this->dataDescription.static_time());
      this->envData = data;
      using ComponentT = components::Environment;
      auto component = ComponentT{std::move(data)};
      _ecm.CreateComponent(worldEntity(_ecm), std::move(component));
      this->visualizationPtr->resample = true;
      this->fileLoaded = true;
    }
    catch (const std::invalid_argument &exc)
    {
      if (this->logFileLoadError)
      {
        gzerr << "Failed to load environment data" << std::endl
              << exc.what() << std::endl;
        this->logFileLoadError = false;
      }
    }

    this->needsReload = false;
  }
};

//////////////////////////////////////////////////
EnvironmentPreload::EnvironmentPreload()
  : System(), dataPtr(new EnvironmentPreloadPrivate)
{
}

//////////////////////////////////////////////////
EnvironmentPreload::~EnvironmentPreload() = default;

//////////////////////////////////////////////////
void EnvironmentPreload::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf;
}

//////////////////////////////////////////////////
void EnvironmentPreload::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!std::exchange(this->dataPtr->loaded, true))
  {
    auto world = worldEntity(_ecm);

    // See https://github.com/gazebosim/gz-sim/issues/1786
    this->dataPtr->node.Subscribe(
      transport::TopicUtils::AsValidTopic(
        scopedName(world, _ecm) + "/environment"),
      &EnvironmentPreloadPrivate::OnLoadCommand, this->dataPtr.get());
    this->dataPtr->node.Subscribe(
      transport::TopicUtils::AsValidTopic(
        scopedName(world, _ecm) + "/environment/visualize/res"),
      &EnvironmentPreloadPrivate::OnVisualResChanged, this->dataPtr.get());

    this->dataPtr->visualizationPtr->resample = true;
    this->dataPtr->ReadSdf(_ecm);
  }

  if (this->dataPtr->needsReload)
  {
    this->dataPtr->LoadEnvironment(_ecm);
  }

  if (this->dataPtr->visualize)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    auto samples = this->dataPtr->samples;
    this->dataPtr->visualizationPtr->Step(_info, _ecm, this->dataPtr->envData,
      samples.X(), samples.Y(), samples.Z());
  }
}

// Register this plugin
GZ_ADD_PLUGIN(EnvironmentPreload, System,
    EnvironmentPreload::ISystemConfigure,
    EnvironmentPreload::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EnvironmentPreload,
    "gz::sim::systems::EnvironmentPreload")
