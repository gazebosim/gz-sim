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
#include "EnvironmentalDataPreload.hh"

#include <array>
#include <string>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/EnvironmentalData.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
EnvironmentalDataPreload::EnvironmentalDataPreload() : System()
{
}

//////////////////////////////////////////////////
EnvironmentalDataPreload::~EnvironmentalDataPreload() = default;

//////////////////////////////////////////////////
void EnvironmentalDataPreload::Configure(
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
    using ComponentT = components::EnvironmentalData;
    auto component = ComponentT{
      common::IO<ComponentT::Type>::ReadFrom(
          common::CSVIStreamIterator(dataFile),
          common::CSVIStreamIterator(),
          timeColumnName, spatialColumnNames)};
    _ecm.CreateComponent<ComponentT>(worldEntity(_ecm), component);
  }
  catch (const std::invalid_argument &exc)
  {
    gzerr << "Failed to load environmental data" << std::endl
          << exc.what() << std::endl;
  }
}

// Register this plugin
GZ_ADD_PLUGIN(EnvironmentalDataPreload, System,
    EnvironmentalDataPreload::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(EnvironmentalDataPreload,
    "gz::sim::systems::EnvironmentalDataPreload")
