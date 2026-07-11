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
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "EntityComponentManager.hh"

#include "EntityIteration.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Types.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{

static std::vector<gz::sim::ComponentTypeId> parseComponentTypes(const pybind11::list &_comp_types)
{
  std::vector<gz::sim::ComponentTypeId> types;
  types.reserve(_comp_types.size());
  for (auto item : _comp_types)
  {
    if (pybind11::hasattr(item, "type_id"))
    {
      types.push_back(pybind11::cast<gz::sim::ComponentTypeId>(item.attr("type_id")));
    }
  }
  return types;
}

static std::optional<gz::sim::ComponentTypeId> getTypeId(const pybind11::object &_comp_type_proxy)
{
  if (pybind11::hasattr(_comp_type_proxy, "type_id"))
  {
    return pybind11::cast<gz::sim::ComponentTypeId>(_comp_type_proxy.attr("type_id"));
  }
  return std::nullopt;
}

/////////////////////////////////////////////////
void defineSimEntityComponentManager(pybind11::object module)
{
  pybind11::enum_<sim::ComponentState>(module, "ComponentState")
    .value("NoChange", sim::ComponentState::NoChange)
    .value("PeriodicChange", sim::ComponentState::PeriodicChange)
    .value("OneTimeChange", sim::ComponentState::OneTimeChange);

  py::class_<gz::sim::EntityComponentManager,
             std::shared_ptr<gz::sim::EntityComponentManager>>(
       module, "EntityComponentManager",
       "The Entity Component Manager (ECM) manages entities and their "
       "components "
       "in the simulation.")
      .def(py::init<>())
      .def("create_entity", &gz::sim::EntityComponentManager::CreateEntity)
      .def("each", [](gz::sim::EntityComponentManager &self, const py::list &comp_types)
           {
             return ECMPythonAccessor::EachList(self, parseComponentTypes(comp_types));
           },
           "Get all entities and components matching the query as a list at once.")
      .def(
           "component",
           [](gz::sim::EntityComponentManager &self,
              const gz::sim::Entity &_entity,
              const py::object &_comp_type_proxy) -> py::object
           {
             auto type_id = getTypeId(_comp_type_proxy);
             if (!type_id)
               return py::none();

             auto getter = gz::sim::python::ComponentPybindRegistry::Instance()->Getter(*type_id);
             return getter ? getter(self, _entity) : py::none();
           },
           "Get the data of a component for an entity and component type if it exists.")
      .def("create_component", [](gz::sim::EntityComponentManager &self,
                                  const gz::sim::Entity &_entity,
                                  const py::object &_comp_type_proxy,
                                  const py::object &_data)
           {
             if (auto type_id = getTypeId(_comp_type_proxy))
             {
               if (auto setter = gz::sim::python::ComponentPybindRegistry::Instance()->Setter(*type_id))
               {
                 setter(self, _entity, _data, false);
               }
             }
           },
           "Create a component for an entity with initial data.")
      .def("set_component_data", [](gz::sim::EntityComponentManager &self,
                                    const gz::sim::Entity &_entity,
                                    const py::object &_comp_type_proxy,
                                    const py::object &_data,
                                    bool _compare) -> bool
           {
             if (auto type_id = getTypeId(_comp_type_proxy))
             {
               if (auto setter = gz::sim::python::ComponentPybindRegistry::Instance()->Setter(*type_id))
               {
                 return setter(self, _entity, _data, _compare);
               }
             }
             return false;
           },
           py::arg("entity"),
           py::arg("comp_type"),
           py::arg("data"),
           py::arg("compare") = true,
           "Set the data for an entity's component. If compare is True (default), only sets if data changed and returns True if changed.")
      .def("set_changed", &gz::sim::EntityComponentManager::SetChanged)
      .def("request_remove_entity", &gz::sim::EntityComponentManager::RequestRemoveEntity,
           py::arg("entity"), py::arg("recursive") = true,
           "Request an entity deletion. The request is processed at the end of the simulation update step.")
      .def("has_entity", &gz::sim::EntityComponentManager::HasEntity,
           py::arg("entity"), "Get whether an Entity exists.")
      .def("parent_entity", &gz::sim::EntityComponentManager::ParentEntity,
           py::arg("entity"), "Get the parent entity or kNullEntity if there is none.")
      .def("set_parent_entity", &gz::sim::EntityComponentManager::SetParentEntity,
           py::arg("child"), py::arg("parent"), "Set the parent of an entity.")
      .def("children", [](const gz::sim::EntityComponentManager &self, const gz::sim::Entity &_entity) -> std::vector<gz::sim::Entity>
           {
             return self.EntitiesByComponents(gz::sim::components::ParentEntity(_entity));
           },
           py::arg("entity"), "Get all immediate child entities of the given entity.")
      .def("remove_component", [](gz::sim::EntityComponentManager &self,
                                  const gz::sim::Entity &_entity,
                                  const py::object &_comp_type_proxy) -> bool
           {
             auto type_id = getTypeId(_comp_type_proxy);
             return type_id ? self.RemoveComponent(_entity, *type_id) : false;
           },
           py::arg("entity"), py::arg("comp_type"),
           "Remove a component from an entity.")
      .def("has_component_type", [](const gz::sim::EntityComponentManager &self,
                                     const gz::sim::Entity &_entity,
                                     const py::object &_comp_type_proxy) -> bool
           {
             auto type_id = getTypeId(_comp_type_proxy);
             return type_id ? self.EntityHasComponentType(_entity, *type_id) : false;
           },
           py::arg("entity"), py::arg("comp_type"),
           "Check whether an entity has a specific component type.")
      .def("each_new", [](gz::sim::EntityComponentManager &self, const py::list &comp_types)
           {
             return ECMPythonAccessor::EachNewList(self, parseComponentTypes(comp_types));
           },
           py::arg("comp_types"),
           "Get all newly created entities and components matching the query as a list at once.")
      .def("each_removed", [](gz::sim::EntityComponentManager &self, const py::list &comp_types)
           {
             return ECMPythonAccessor::EachRemovedList(self, parseComponentTypes(comp_types));
           },
           py::arg("comp_types"),
           "Get all entities and components about to be removed matching the query as a list at once.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
