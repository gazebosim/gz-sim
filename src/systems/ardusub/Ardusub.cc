/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <mutex>

#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <sdf/sdf.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "Ardusub.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ArdusubPrivate
{
  /// \brief Model interface
  public: Model model{kNullEntity};

  // Your data and private functions.
};

//////////////////////////////////////////////////
Ardusub::Ardusub()
  : dataPtr(std::make_unique<ArdusubPrivate>())
{
}

//////////////////////////////////////////////////
void Ardusub::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "Ardusub plugin should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Additional configuration code.
}

//////////////////////////////////////////////////
void Ardusub::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Ardusub::PreUpdate");

  std::cout << "I'm ardu, ardusub\n";
  // This function is executed every iteration.
}

IGNITION_ADD_PLUGIN(Ardusub,
                    ignition::gazebo::System,
                    Ardusub::ISystemConfigure,
                    Ardusub::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Ardusub,
                          "ignition::gazebo::systems::Ardusub")
