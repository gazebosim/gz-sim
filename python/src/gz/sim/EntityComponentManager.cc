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

#include "EntityComponentManager.hh"

namespace gz
{
namespace sim
{
namespace python
{
/////////////////////////////////////////////////
void defineSimEntityComponentManager(pybind11::object module)
{
  pybind11::class_<gz::sim::EntityComponentManager>(
      module, "EntityComponentManager",
    "The Entity Component Manager (ECM) manages entities and their components "
    "in the simulation.")
    .def(pybind11::init<>())
    .def("entity_count", &gz::sim::EntityComponentManager::EntityCount,
      "Get total number of entities.")
    .def("create_entity",
      pybind11::overload_cast<>(&gz::sim::EntityComponentManager::CreateEntity),
      "Create a new entity.")
    .def("has_entity", &gz::sim::EntityComponentManager::HasEntity,
      pybind11::arg("entity"),
      "Check if an entity exists.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
