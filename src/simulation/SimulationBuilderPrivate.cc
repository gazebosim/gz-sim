/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "SimulationBuilderPrivate.hh"
#include "SimulationDefaults.hh"

#include <gz/common/URI.hh>
#include <gz/fuel_tools/Interface.hh>

namespace gz::sim::simulation
{

sdf::Root WorldInfo::Create() const
{
  sdf::Root ret;

  auto fuelClient = fuel_tools::FuelClient(this->clientConfig);

  // Callback for retreiving resources.
  // If the user has overridden, then prefer that, otherwise default to fuel.
  auto fetchResource = [this, &fuelClient](const std::string &_uri){
    if (this->fetchResourceCb)
    {
      return this->fetchResourceCb(_uri);
    }
    else
    {
      auto path = gz::fuel_tools::fetchResourceWithClient(_uri, fuelClient);
      return path;
    }
  };

  auto fetchResourceUri = [this, &fetchResource](const gz::common::URI &_uri) {
    if (this->fetchResourceUriCb)
    {
      return this->fetchResourceUriCb(_uri);
    }
    else
    {
      return fetchResource(_uri.Str());
    }
  };

  // Configure SDF to fetch assets from Gazebo Fuel.
  sdf::setFindCallback(fetchResource);
  gz::common::addFindFileURICallback(fetchResourceUri);

  switch(this->source)
  {
    case WorldSource::kEmptyWorld:
      ret.LoadSdfString(defaultEmptyWorld());
      break;
    case WorldSource::kSdfRoot:
      ret = this->sdfRoot->Clone();
      break;
    case WorldSource::kSdfFilename:
      ret.LoadSdfString(this->sdfString);
      break;
    case WorldSource::kSdfString:
      ret.Load(this->sdfPath.string());
      break;
  }

  return ret;
}


}  // namespace gz::sim::simulation
