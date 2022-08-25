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
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
EnvironmentPreload::EnvironmentPreload() : System()
{
}

//////////////////////////////////////////////////
EnvironmentPreload::~EnvironmentPreload() = default;

//////////////////////////////////////////////////
void EnvironmentPreload::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("data"))
  {
    gzerr << "No environmental data file was specified";
    return;
  }

  const std::string dataPath =
      _sdf->Get<std::string>("data");
  std::ifstream dataFile(dataPath);
  if (!dataFile.is_open())
  {
    gzerr << "No environmental data file was found at " << dataPath;
    return;
  }


  std::string timeColumnName{"t"};
  std::array<std::string, 3> spatialColumnNames{"x", "y", "z"};
  auto spatialReference = math::SphericalCoordinates::GLOBAL;

  sdf::ElementConstPtr elem = _sdf->FindElement("dimensions");
  if (elem)
  {
    if (elem->HasElement("time"))
    {
      timeColumnName = elem->Get<std::string>("time");
    }
    elem = elem->FindElement("space");
    if (elem)
    {
      if (elem->HasAttribute("reference"))
      {
        const std::string referenceName = elem->Get<std::string>("reference");
        if (referenceName == "global")
        {
          spatialReference = math::SphericalCoordinates::GLOBAL;
        }
        else if (referenceName == "spherical")
        {
          spatialReference = math::SphericalCoordinates::SPHERICAL;
        }
        else if (referenceName == "ecef")
        {
          spatialReference = math::SphericalCoordinates::ECEF;
        }
        else
        {
          gzerr << "Unknown reference '" << referenceName << "'" << std::endl;
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
    using ComponentDataT = components::EnvironmentalData;
    auto data = std::make_shared<ComponentDataT>(
      common::IO<ComponentDataT::FrameT>::ReadFrom(
          common::CSVIStreamIterator(dataFile),
          common::CSVIStreamIterator(),
          timeColumnName, spatialColumnNames),
      spatialReference);

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

// Register this plugin
GZ_ADD_PLUGIN(EnvironmentPreload, System,
    EnvironmentPreload::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(EnvironmentPreload,
    "gz::sim::systems::EnvironmentPreload")
