/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/gazebo/SystemConfig.hh>

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::SystemConfigPrivate
{
  public: explicit SystemConfigPrivate(
              std::shared_ptr<EntityComponentManager> _compMgr)
          : compMgr(_compMgr)
  {
  }

  public: explicit SystemConfigPrivate(const SystemConfigPrivate *_config)
          : compMgr(_config->compMgr)
  {
  }

  public: std::shared_ptr<EntityComponentManager> compMgr;
};

//////////////////////////////////////////////////
SystemConfig::SystemConfig(std::shared_ptr<EntityComponentManager> _compMgr)
  : dataPtr(new SystemConfigPrivate(_compMgr))
{
}

//////////////////////////////////////////////////
SystemConfig::SystemConfig(const SystemConfig &_config)
  : dataPtr(new SystemConfigPrivate(_config.dataPtr.get()))
{
}

//////////////////////////////////////////////////
SystemConfig::~SystemConfig()
{
}

//////////////////////////////////////////////////
EntityComponentManager &SystemConfig::EntityComponentMgr() const
{
  IGN_ASSERT(this->dataPtr->compMgr != nullptr,
      "SystemConfig does not ave a valid ComponentManager pointer.");
  return *(this->dataPtr->compMgr.get());
}

//////////////////////////////////////////////////
SystemConfig &SystemConfig::operator=(const SystemConfig &_config)
{
  this->dataPtr.reset(new SystemConfigPrivate(_config.dataPtr.get()));
  return *this;
}
