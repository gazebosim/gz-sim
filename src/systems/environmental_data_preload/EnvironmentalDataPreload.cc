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

#include <gz/common/CSVFile.hh>

#include "gz/sim/components/EnvironmentalData.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void EnvironmentalDataPreload::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("data"))
  {
    gzerr << "No environmental data file was found";
    return;
  }
  common::CSVFile dataFile(_sdf->Get<std::string>("data"));

  std::string timeColumn;
  std::array<std::string, 3> coordinateColumns;

  sdf::ElementPtr elem = _sdf->GetElement("dimensions");
  elem->Get<std::string>("time", timeColumn, "t");
  elem = elem->GetElement("space");
  elem->Get<std::string>("x", coordinateColumns[0], "x");
  elem->Get<std::string>("y", coordinateColumns[1], "y");
  elem->Get<std::string>("z", coordinateColumns[2], "z");

  using ComponentT = components::EnvironmentalData;
  auto data = common::IO<ComponentT::Type>::ReadFrom(
      dataFile, timeColumn, coordinateColumns);
  _ecm.CreateComponent<ComponenT>(worldEntity(_ecm), data);
}
