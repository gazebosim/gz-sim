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
#include "ignition/gazebo/System.hh"

using namespace ignition::gazebo;

//////////////////////////////////////////////////
System::System()
{
}

System::~System()
{
}

//////////////////////////////////////////////////
void System::EntityAdded(const Entity& _entity,
                         const EntityComponentManager &_ecm)
{
  (void) _entity;
  (void) _ecm;
}

//////////////////////////////////////////////////
void System::EntityRemoved(const Entity& _entity,
                           const EntityComponentManager &_ecm)
{
  (void) _entity;
  (void) _ecm;
}

//////////////////////////////////////////////////
void System::PreUpdate(const UpdateInfo &_info,
                       EntityComponentManager &_ecm)
{
  (void) _info;
  (void) _ecm;
}

//////////////////////////////////////////////////
void System::Update(const UpdateInfo &_info,
                    EntityComponentManager &_ecm)
{
  (void) _info;
  (void) _ecm;
}

void System::PostUpdate(const UpdateInfo &_info,
//////////////////////////////////////////////////
                        const EntityComponentManager &_ecm)
{
  (void) _info;
  (void) _ecm;
}
