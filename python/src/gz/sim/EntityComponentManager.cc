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
#include <pybind11/stl.h>

#include "EntityComponentManager.hh"

#include "EntityIteration.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Types.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{

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
             std::vector<gz::sim::ComponentTypeId> types;
             for (auto item : comp_types)
             {
               if (py::hasattr(item, "type_id"))
               {
                 types.push_back(py::cast<gz::sim::ComponentTypeId>(item.attr("type_id")));
               }
             }
             return ECMPythonAccessor::EachList(self, types);
           },
           "Get all entities and components matching the query as a list at once.")
      .def(
          "component",
          [](gz::sim::EntityComponentManager &self,
             const gz::sim::Entity &_entity,
             const py::object &_comp_type_proxy) -> py::object
          {
            if (!py::hasattr(_comp_type_proxy, "type_id"))
            {
              return py::none();
            }
            auto type_id = py::cast<gz::sim::ComponentTypeId>(
                _comp_type_proxy.attr("type_id"));
            
            auto getter = gz::sim::python::ComponentPybindRegistry::Instance()->Getter(type_id);
            if (getter)
            {
              return getter(self, _entity);
            }
            return py::none();
          },
          "Get the data of a component for an entity and component type if it exists.")
      .def("create_component", [](gz::sim::EntityComponentManager &self,
                                  const gz::sim::Entity &_entity,
                                  const py::object &_comp_type_proxy,
                                  const py::object &_data)
           {
             if (py::hasattr(_comp_type_proxy, "type_id"))
             {
               auto type_id = py::cast<gz::sim::ComponentTypeId>(_comp_type_proxy.attr("type_id"));
               if (auto setter = gz::sim::python::ComponentPybindRegistry::Instance()->Setter(type_id))
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
             if (py::hasattr(_comp_type_proxy, "type_id"))
             {
               auto type_id = py::cast<gz::sim::ComponentTypeId>(_comp_type_proxy.attr("type_id"));
               if (auto setter = gz::sim::python::ComponentPybindRegistry::Instance()->Setter(type_id))
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
      .def("set_changed", &gz::sim::EntityComponentManager::SetChanged);
}
}  // namespace python
}  // namespace sim
}  // namespace gz

