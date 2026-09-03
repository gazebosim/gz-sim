/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "Components.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <string>
#include <vector>

#include <gz/common/Util.hh>
#include <gz/math/Temperature.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/TemperatureRange.hh>
#include <gz/sim/python/ComponentPybindRegistry.hh>

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{

/////////////////////////////////////////////////
void defineComponentStructs(pybind11::module &m)
{
  namespace comps = gz::sim::components;
  // Bind internal C++ struct data types used by components
  py::class_<comps::DetachableJointInfo>(m, "DetachableJointInfo")
      .def(py::init<>())
      .def_readwrite("parent_link", &comps::DetachableJointInfo::parentLink)
      .def_readwrite("child_link", &comps::DetachableJointInfo::childLink)
      .def_readwrite("joint_type", &comps::DetachableJointInfo::jointType)
      .def(py::self == py::self)
      .def(py::self != py::self);

  py::class_<comps::TemperatureRangeInfo>(m, "TemperatureRangeInfo")
      .def(py::init<>())
      .def_readwrite("min", &comps::TemperatureRangeInfo::min)
      .def_readwrite("max", &comps::TemperatureRangeInfo::max)
      .def(py::self == py::self)
      .def(py::self != py::self);

  py::class_<comps::RayInfo>(m, "RayInfo")
      .def(py::init<>())
      .def_readwrite("start", &comps::RayInfo::start)
      .def_readwrite("end", &comps::RayInfo::end);

  py::class_<comps::RaycastResultInfo>(m, "RaycastResultInfo")
      .def(py::init<>())
      .def_readwrite("point", &comps::RaycastResultInfo::point)
      .def_readwrite("fraction", &comps::RaycastResultInfo::fraction)
      .def_readwrite("normal", &comps::RaycastResultInfo::normal);

  py::class_<comps::RaycastDataInfo>(m, "RaycastDataInfo")
      .def(py::init<>())
      .def_readwrite("rays", &comps::RaycastDataInfo::rays)
      .def_readwrite("results", &comps::RaycastDataInfo::results);
}

/////////////////////////////////////////////////
/// \brief Helper function to retrieve all components registered in the core
/// C++ ComponentFactory.
///
/// This function queries gz::sim::components::Factory::Instance() directly
/// rather than ComponentPybindRegistry. While ComponentPybindRegistry only
/// tracks components with active Python getter/setter bindings, the core
/// Factory contains every C++ component registered in Gazebo Sim.
///
/// Querying the Factory directly serves two essential purposes:
/// 1. Module Pre-Population: Pre-populates proxy tokens on the
///    gz.sim.components module for all known components at import time.
/// 2. Registration Parity Auditing: Enables the test suite
///    (e.g., test_component_registration_parity in components_TEST.py) to
///    introspect every C++ component and verify that all components with
///    bound data types have registered Python bindings or are explicitly
///    accounted for in the known skipped list.
///
/// \return A list of ComponentProxy objects for all components registered in
/// the C++ ComponentFactory with their names normalized via CleanName.
static std::vector<ComponentProxy> allFactoryComponents()
{
  auto factory = gz::sim::components::Factory::Instance();
  std::vector<ComponentProxy> result;
  for (const auto &typeId : factory->TypeIds())
  {
    std::string name = factory->Name(typeId);
    if (!name.empty())
    {
      result.push_back(ComponentProxy{
          ComponentPybindRegistry::CleanName(name), typeId});
    }
  }
  return result;
}

/////////////////////////////////////////////////
void populateComponentsModule(pybind11::module &m)
{
  // Bind the ComponentProxy struct
  py::class_<ComponentProxy>(m, "ComponentProxy")
      .def_readonly("name", &ComponentProxy::name)
      .def_readonly("type_id", &ComponentProxy::typeId)
      .def("__repr__", [](const ComponentProxy &self)
           { return "<gz.sim.components." + self.name + ">"; })
      .def("__eq__", [](const ComponentProxy &self, const py::object &other)
           {
             if (py::isinstance<ComponentProxy>(other))
             {
               return self.typeId == other.cast<ComponentProxy>().typeId;
             }
             return false;
           })
      .def("__ne__", [](const ComponentProxy &self, const py::object &other)
           {
             if (py::isinstance<ComponentProxy>(other))
             {
               return self.typeId != other.cast<ComponentProxy>().typeId;
             }
             return true;
           })
      .def("__hash__", [](const ComponentProxy &self)
           { return std::hash<gz::sim::ComponentTypeId>{}(self.typeId); });

  // Implement PEP 562 dynamic component resolution
  m.def("__getattr__", [m](const std::string &_name) -> ComponentProxy
  {
    auto factory = gz::sim::components::Factory::Instance();
    auto checkAndCache =
        [&](const std::string &_key) -> std::optional<ComponentProxy>
    {
      auto typeId = gz::common::hash64(_key);
      if (factory->HasType(typeId))
      {
        ComponentProxy comp{_name, typeId};
        m.attr(_name.c_str()) = comp;
        return comp;
      }
      return std::nullopt;
    };

    if (auto comp = checkAndCache("gz_sim_components." + _name))
    {
      return *comp;
    }
    if (auto comp = checkAndCache(_name))
    {
      return *comp;
    }

    throw py::attribute_error(
        "module 'gz.sim.components' has no attribute '" + _name + "'");
  });

  // Test and introspection helpers
  m.def("has_python_bindings", [](const ComponentProxy &_comp) -> bool
  {
    return ComponentPybindRegistry::Instance()->HasBindings(_comp.typeId);
  }, py::arg("comp_type"),
  "Check if a component type has registered Python getter/setter bindings.");

  m.def("all_factory_components", &allFactoryComponents,
        "Return all component types registered in the C++ ComponentFactory.");

  // Pre-populate proxy objects for all currently registered components
  for (const auto &comp : allFactoryComponents())
  {
    m.attr(comp.name.c_str()) = comp;
  }
}
}  // namespace python
}  // namespace sim
}  // namespace gz
