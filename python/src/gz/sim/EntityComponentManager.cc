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

#include <string>
#include <vector>

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

namespace
{
/////////////////////////////////////////////////
/// \brief Convert the positional arguments of a query into component type
/// ids.
/// \param[in] _compTypes Python arguments, each of which must be a
/// ComponentProxy.
/// \return The component type ids, in the order they were given.
/// \throws pybind11::type_error if any argument is not a component type.
std::vector<gz::sim::ComponentTypeId> ParseComponentTypes(
    const pybind11::args &_compTypes)
{
  std::vector<gz::sim::ComponentTypeId> types;
  types.reserve(_compTypes.size());
  for (auto item : _compTypes)
  {
    if (!pybind11::isinstance<ComponentProxy>(item))
    {
      throw pybind11::type_error(
          "All arguments must be component types (did you forget to "
          "unpack a list with *?)");
    }
    types.push_back(pybind11::cast<ComponentProxy>(item).typeId);
  }
  return types;
}

/////////////////////////////////////////////////
/// \brief Run a runtime-typed ECM query and collect the result as a list of
/// tuples.
///
/// Each tuple is (entity, value...) with one value per requested component
/// type. Data components yield a snapshot of their data; tag (NoData)
/// components yield their ComponentProxy, since they have no data.
///
/// \param[in] _ecm The EntityComponentManager to query.
/// \param[in] _types Component types every matched entity must have.
/// \param[in] _each Callable that invokes the desired ECM query.
/// \return A list of tuples, each 1 + _types.size() wide.
/// \throws pybind11::type_error if a component type has no Python bindings.
template <typename EachFn>
pybind11::list EachToList(const gz::sim::EntityComponentManager &_ecm,
                          const std::vector<gz::sim::ComponentTypeId> &_types,
                          EachFn _each)
{
  namespace py = pybind11;
  auto *reg = ComponentPybindRegistry::Instance();

  // Hoisted once per query: registry lookups, mutex acquisitions and tag
  // proxy construction all stay out of the entity loop.
  std::vector<ComponentPybindRegistry::RawGetterFn> getters;
  std::vector<py::object> tagProxies;
  getters.reserve(_types.size());
  tagProxies.reserve(_types.size());

  for (const auto typeId : _types)
  {
    if (!reg->HasBindings(typeId))
    {
      throw py::type_error(
          "Component type with type_id " + std::to_string(typeId) +
          " is not registered for Python manipulation");
    }

    // A null raw getter means a tag (NoData) component: there is no data to
    // return, so the proxy itself is handed back instead.
    auto getter = reg->RawGetter(typeId);
    getters.push_back(getter);
    tagProxies.push_back(getter
        ? py::none()
        : py::cast(ComponentProxy{reg->ComponentName(typeId), typeId}));
  }

  py::list result;
  _each(_ecm, _types,
      [&](gz::sim::Entity _entity,
          const std::vector<const gz::sim::components::BaseComponent *>
              &_comps)
      {
        py::tuple row(1 + _comps.size());
        row[0] = py::cast(_entity);
        for (size_t i = 0; i < _comps.size(); ++i)
        {
          row[1 + i] = getters[i] ? getters[i](_comps[i]) : tagProxies[i];
        }
        result.append(row);
        return true;
      });
  return result;
}
}  // namespace

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
         "ComponentState.NoChange if the component does not exist.")
    .def("each_data",
         [](const gz::sim::EntityComponentManager &_self,
            const pybind11::args &_compTypes)
         {
           return EachToList(_self, ParseComponentTypes(_compTypes),
               [](const auto &_e, const auto &_t, const auto &_f)
               { _e.Each(_t, _f); });
         },
         "Get all entities and their component data matching the given "
         "component types, as a list of tuples. Each tuple is "
         "(entity, value...) with one value per component type; tag (NoData) "
         "components yield their component type. Component values follow the "
         "same copy semantics as component_data(), including the "
         "pointer-payload exception; use set_component_data() to write. "
         "Entities marked for removal but not yet processed are included.")
    .def("each_new_data",
         [](const gz::sim::EntityComponentManager &_self,
            const pybind11::args &_compTypes)
         {
           return EachToList(_self, ParseComponentTypes(_compTypes),
               [](const auto &_e, const auto &_t, const auto &_f)
               { _e.EachNew(_t, _f); });
         },
         "Same as each_data(), restricted to entities created during this "
         "simulation step.")
    .def("each_removed_data",
         [](const gz::sim::EntityComponentManager &_self,
            const pybind11::args &_compTypes)
         {
           return EachToList(_self, ParseComponentTypes(_compTypes),
               [](const auto &_e, const auto &_t, const auto &_f)
               { _e.EachRemoved(_t, _f); });
         },
         "Same as each_data(), restricted to entities marked for removal.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
