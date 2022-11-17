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

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>
#include <gz/common/Filesystem.hh>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
class gz::sim::systems::EnvironmentPreloadPrivate
{
  public: bool loaded{false};

  public: std::shared_ptr<const sdf::Element> sdf;
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
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!std::exchange(this->dataPtr->loaded, true))
  {
    if (!this->dataPtr->sdf->HasElement("data"))
    {
      gzerr << "No environmental data file was specified";
      return;
    }

    std::string dataPath =
        this->dataPtr->sdf->Get<std::string>("data");
    if (common::isRelativePath(dataPath))
    {
      auto * component =
          _ecm.Component<components::WorldSdf>(worldEntity(_ecm));
      const std::string rootPath =
          common::parentPath(component->Data().Element()->FilePath());
      dataPath = common::joinPaths(rootPath, dataPath);
    }

    std::ifstream dataFile(dataPath);
    if (!dataFile.is_open())
    {
      gzerr << "No environmental data file was found at " << dataPath;
      return;
    }

    components::EnvironmentalData::ReferenceUnits unit{
      components::EnvironmentalData::ReferenceUnits::RADIANS};
    std::string timeColumnName{"t"};
    bool ignoreTime = false;
    std::array<std::string, 3> spatialColumnNames{"x", "y", "z"};
    auto spatialReference = math::SphericalCoordinates::GLOBAL;
    sdf::ElementConstPtr elem =
        this->dataPtr->sdf->FindElement("dimensions");
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
            spatialReference = math::SphericalCoordinates::GLOBAL;
          }
          else if (referenceName == "spherical")
          {
            spatialReference = math::SphericalCoordinates::SPHERICAL;
            if (elem->HasAttribute("units"))
            {
              std::string unitName = elem->Get<std::string>("units");
              if (unitName == "degrees")
              {
                unit = components::EnvironmentalData::ReferenceUnits::DEGREES;
              }
              else if (unitName != "radians")
              {
                ignerr << "Unrecognized unit " << unitName << "\n";
              }
            }
          }
          else if (referenceName == "ecef")
          {
            spatialReference = math::SphericalCoordinates::ECEF;
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

    try
    {
      gzmsg << "Loading Environment Data\n";
      using ComponentDataT = components::EnvironmentalData;
      auto data = ComponentDataT::MakeSharedWithUnits(
          common::IO<ComponentDataT::FrameT>::ReadFrom(
              common::CSVIStreamIterator(dataFile),
              common::CSVIStreamIterator(),
              timeColumnName, spatialColumnNames),
          spatialReference, unit, ignoreTime);

      using ComponentT = components::Environment;
      auto component = ComponentT{std::move(data)};
      _ecm.CreateComponent(worldEntity(_ecm), std::move(component));
    }
    catch (const std::invalid_argument &exc)
    {
      gzerr << "Failed to load environment data" << std::endl
            << exc.what() << std::endl;
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(EnvironmentPreload, System,
    EnvironmentPreload::ISystemConfigure,
    EnvironmentPreload::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EnvironmentPreload,
    "gz::sim::systems::EnvironmentPreload")
