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
#include "gz/sim/Types.hh"
#include "gz/sim/python/ComponentPybindRegistry.hh"

namespace gz
{
namespace sim
{
namespace python
{
using detail::ComponentProxy;
using detail::ComponentPybindRegistry;
/////////////////////////////////////////////////
void defineSimEntityComponentManager(pybind11::object module)
{
  pybind11::enum_<sim::ComponentState>(module, "ComponentState")
    .value("NoChange", sim::ComponentState::NoChange)
    .value("PeriodicChange", sim::ComponentState::PeriodicChange)
    .value("OneTimeChange", sim::ComponentState::OneTimeChange);

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
    .def("request_remove_entity",
         &gz::sim::EntityComponentManager::RequestRemoveEntity,
         pybind11::arg("entity"), pybind11::arg("recursive") = true,
         "Request an entity deletion. The request is processed at the end "
         "of the simulation update step.")
    .def("has_entity", &gz::sim::EntityComponentManager::HasEntity,
      pybind11::arg("entity"),
      "Check if an entity exists.")
    .def("parent_entity", &gz::sim::EntityComponentManager::ParentEntity,
         pybind11::arg("entity"),
         "Get the parent entity or kNullEntity if there is none.")
    .def("set_parent_entity",
         &gz::sim::EntityComponentManager::SetParentEntity,
         pybind11::arg("child"), pybind11::arg("parent"),
         "Set the parent of an entity.")
    .def("entity_has_component_type",
         [](const gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp) -> bool
         {
           return self.EntityHasComponentType(_entity, _comp.typeId);
         },
         pybind11::arg("entity"), pybind11::arg("comp_type"),
         "Check whether an entity has a specific component type.")
    .def("remove_component",
         [](gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp) -> bool
         {
           return self.RemoveComponent(_entity, _comp.typeId);
         },
         pybind11::arg("entity"), pybind11::arg("comp_type"),
         "Remove a component from an entity.")
    .def("create_component",
         [](gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp,
            const pybind11::object &_data)
         {
           auto creator =
               ComponentPybindRegistry::Instance()->Creator(_comp.typeId);
           if (!creator)
           {
             throw pybind11::type_error(
                 "Component type '" + _comp.name + "' is not registered for "
                 "Python manipulation");
           }
           if (!creator(self, _entity, _data))
           {
             throw pybind11::key_error(
                 "Failed to create component on entity (entity may not exist)");
           }
         },
         pybind11::arg("entity"),
         pybind11::arg("comp_type"),
         pybind11::arg("data") = pybind11::none(),
         "Create a component for an entity, replacing it if it already "
         "exists, and mark it changed. For data components, initial data is "
         "required; for tag (NoData) components, data must not be provided.")
    .def("_create_default_component",
         [](gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp)
         {
           auto registry = ComponentPybindRegistry::Instance();
           auto creator = registry->DefaultCreator(_comp.typeId);
           if (!creator)
           {
             throw pybind11::type_error(
                 "Component type is not registered for Python manipulation");
           }
           if (!creator(self, _entity))
           {
             throw pybind11::key_error(
                 "Failed to create default component on entity "
                 "(entity may not exist)");
           }
         },
         pybind11::arg("entity"),
         pybind11::arg("comp_type"),
         "Create a default-initialized component on an entity.")
    .def("component_data",
         [](const gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp) -> pybind11::object
         {
           auto getter =
               ComponentPybindRegistry::Instance()->Getter(_comp.typeId);
           if (!getter)
           {
             throw pybind11::type_error(
                 "Component type '" + _comp.name + "' is not registered for "
                 "Python data access");
           }
           return getter(self, _entity);
         },
         pybind11::arg("entity"), pybind11::arg("comp_type"),
         "Get a snapshot of a component's data for an entity and component "
         "type, or None if it does not exist. The returned value is a copy; "
         "modifying it does not affect the ECM. Use set_component_data() to "
         "write, and set_changed() to mark the component changed. "
         "EXCEPTION: components whose data is a pointer (an sdf.Element, "
         "for example) return a handle that aliases ECM storage rather "
         "than a copy. Treat those as read-only: mutating the returned "
         "object writes through to the ECM and bypasses change detection.")
    .def("set_component_data",
         [](gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp,
            const pybind11::object &_data) -> bool
         {
           if (auto setter =
                   ComponentPybindRegistry::Instance()->Setter(_comp.typeId))
           {
             return setter(self, _entity, _data);
           }
           throw pybind11::type_error(
               "Component type '" + _comp.name + "' is not registered for "
               "Python data manipulation");
         },
         pybind11::arg("entity"),
         pybind11::arg("comp_type"),
         pybind11::arg("data"),
         "Set the data for an entity's component. The new data is compared "
         "against the current data and only written if it differs; returns "
         "True if the data changed. "
         "This does not mark the component as changed in the ECM; call "
         "set_changed() separately if downstream systems need to be "
         "notified. EXCEPTION: for components whose data is a pointer (see "
         "component_data()), the ECM stores the handle you pass rather than a "
         "copy, so the object remains a live view onto ECM state after the "
         "call.")
    .def("set_changed",
         [](gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp,
            sim::ComponentState _c)
         {
           self.SetChanged(_entity, _comp.typeId, _c);
         },
         pybind11::arg("entity"), pybind11::arg("comp_type"),
         pybind11::arg("state") = sim::ComponentState::OneTimeChange,
         "Set the changed state of a component.")
    .def("component_state",
         [](const gz::sim::EntityComponentManager &self,
            const gz::sim::Entity &_entity,
            const ComponentProxy &_comp)
         {
           return self.ComponentState(_entity, _comp.typeId);
         },
         pybind11::arg("entity"), pybind11::arg("comp_type"),
         "Get the changed state of a component. Returns "
         "ComponentState.NoChange if the component does not exist.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
