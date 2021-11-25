/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
 */
#include <pybind11/pybind11.h>

#include "entity_component_manager.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
/////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager(
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  _entity_component_manager = &_ecm;
}

/////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager(
  ignition::gazebo::EntityComponentManager &_ecm)
{
  _entity_component_manager_no_const = &_ecm;
}

/////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager()
{
}

/////////////////////////////////////////////////
void EntityComponentManager::Destroy()
{
}

/////////////////////////////////////////////////
void define_gazebo_entity_component_manager(pybind11::object module)
{
  pybind11::class_<ignition::gazebo::python::EntityComponentManager,
    ignition::gazebo::python::Destroyable,
    std::shared_ptr<ignition::gazebo::python::EntityComponentManager>>(
      module, "EntityComponentManager")
  .def(pybind11::init<const ignition::gazebo::EntityComponentManager &>());
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition
