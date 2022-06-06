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
#include "Null.hh"
#include <gz/sim/Entity.hh>
#include <gz/plugin/Register.hh>

using namespace gz::sim;
using namespace gz::sim::systems;

//////////////////////////////////////////////////
Null::Null()
  : System()
{
}

//////////////////////////////////////////////////
Null::~Null() = default;

//////////////////////////////////////////////////
void Null::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void Null::PreUpdate(const UpdateInfo &/*_info*/,
                     EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void Null::Update(const UpdateInfo &/*_info*/,
                  EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void Null::PostUpdate(const UpdateInfo &/*_info*/,
                  const EntityComponentManager &/*_ecm*/)
{
}

GZ_ADD_PLUGIN(Null,
                    gz::sim::System,
                    Null::ISystemConfigure,
                    Null::ISystemPreUpdate,
                    Null::ISystemUpdate,
                    Null::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Null, "gz::sim::systems::Null")
