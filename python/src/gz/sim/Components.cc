/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/sim/Types.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/detail/ComponentPybindRegistry.hh>

namespace py = pybind11;
namespace gz
{
namespace sim
{
namespace python
{

// A simple struct to hold component proxy information
struct ComponentType
{
  std::string name;
  gz::sim::ComponentTypeId typeId;
};

class PyCustomComponent : public gz::sim::components::BaseComponent
{
  public: PyCustomComponent(gz::sim::ComponentTypeId _typeId, py::object _obj)
      : typeId(_typeId), obj(std::move(_obj))
  {
  }

  public: ~PyCustomComponent() override = default;

  public: gz::sim::ComponentTypeId TypeId() const override
  {
    return this->typeId;
  }

  public: std::unique_ptr<gz::sim::components::BaseComponent> Clone() const override
  {
    py::module_ copy_module = py::module_::import("copy");
    py::object deepcopy = copy_module.attr("deepcopy");
    py::object cloned_obj = deepcopy(this->obj);
    
    return std::make_unique<PyCustomComponent>(this->typeId, cloned_obj);
  }

  public: py::object Object() const
  {
    return this->obj;
  }

  public: void SetObject(py::object _obj)
  {
    this->obj = std::move(_obj);
  }

  private: gz::sim::ComponentTypeId typeId;
  private: py::object obj;
};

class PyCustomDescriptor : public gz::sim::components::ComponentDescriptorBase
{
  public: PyCustomDescriptor(gz::sim::ComponentTypeId _typeId)
      : typeId(_typeId)
  {
  }

  public: std::unique_ptr<gz::sim::components::BaseComponent> Create() const override
  {
    return std::make_unique<PyCustomComponent>(this->typeId, py::none());
  }

  public: std::unique_ptr<gz::sim::components::BaseComponent> Create(
              const gz::sim::components::BaseComponent *_data) const override
  {
    const auto *pyComp = static_cast<const PyCustomComponent *>(_data);
    return pyComp->Clone();
  }

  private: gz::sim::ComponentTypeId typeId;
};

class ECMPythonAccessor
{
  public: static const gz::sim::components::BaseComponent *Component(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }

  public: static gz::sim::components::BaseComponent *Component(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }

  public: static gz::sim::components::BaseComponent *CreateComponent(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId,
      const gz::sim::components::BaseComponent *_data)
  {
    return _ecm.CreateComponent(_entity, _typeId, _data);
  }
};

ComponentType registerCustomComponent(const std::string &_name)
{
  auto factory = gz::sim::components::Factory::Instance();
  auto typeHash = gz::common::hash64(_name.c_str());
  auto desc = new PyCustomDescriptor(typeHash);
  factory->RegisterCustom(_name.c_str(), desc, gz::sim::components::RegistrationObjectId(desc));

  // Register getter and setter in ComponentPybindRegistry
  auto getter = [typeHash](const gz::sim::EntityComponentManager &_ecm,
                           const gz::sim::Entity &_entity) -> py::object
  {
    auto compBase = ECMPythonAccessor::Component(_ecm, _entity, typeHash);
    if (compBase)
    {
      const auto *pyComp = static_cast<const PyCustomComponent *>(compBase);
      return pyComp->Object();
    }
    return py::none();
  };

  auto setter = [typeHash](gz::sim::EntityComponentManager &_ecm,
                           const gz::sim::Entity &_entity,
                           const py::object &_obj)
  {
    auto compBase = ECMPythonAccessor::Component(_ecm, _entity, typeHash);
    if (!compBase)
    {
      PyCustomComponent stackComp(typeHash, _obj);
      ECMPythonAccessor::CreateComponent(_ecm, _entity, typeHash, &stackComp);
    }
    else
    {
      auto *pyComp = static_cast<PyCustomComponent *>(compBase);
      pyComp->SetObject(_obj);
    }
  };

  gz::sim::python::ComponentPybindRegistry::Instance()->Register(
      typeHash, reinterpret_cast<uintptr_t>(desc), getter, setter);

  return ComponentType{_name, typeHash};
}

/////////////////////////////////////////////////
void populateComponentsModule(pybind11::module &m)
{
  // Bind the ComponentType struct
  py::class_<ComponentType>(m, "ComponentType")
      .def(py::init<std::string, gz::sim::ComponentTypeId>())
      .def_property_readonly("name", [](const ComponentType &self)
                             { return self.name; })
      .def_property_readonly("type_id", [](const ComponentType &self)
                             { return self.typeId; })
      .def("__repr__", [](const ComponentType &self)
           { return "<gz.sim.components." + self.name + ">"; });

  m.def("register_custom_component", &registerCustomComponent, "Register a custom component type");

  // Get the factory instance
  auto factory = gz::sim::components::Factory::Instance();
  auto typeIds = factory->TypeIds();

  // Create proxy objects for all registered components
  for (const auto &typeId : typeIds)
  {
    std::string name = factory->Name(typeId);
    if (!name.empty())
    {
      const std::string prefix = "gz_sim_components.";
      if (name.rfind(prefix, 0) == 0)
      {
        name.erase(0, prefix.length());
      }
      m.attr(name.c_str()) = ComponentType{name, typeId};
    }
  }
}
}  // namespace python
}  // namespace sim
}  // namespace gz
