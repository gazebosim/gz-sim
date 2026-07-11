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
 *
 */
#ifndef GZ_SIM_PYTHON_COMPONENTREGISTRATION_HH_
#define GZ_SIM_PYTHON_COMPONENTREGISTRATION_HH_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/python/ComponentPybindRegistry.hh>

/// \def GZ_SIM_REGISTER_PYTHON_COMPONENT
/// \brief Register a component type so that it can be inspected via Python.
#define GZ_SIM_REGISTER_PYTHON_COMPONENT(_classname) \
  class GzSimPythonComponents##_classname { \
    public: GzSimPythonComponents##_classname() { \
      gz::sim::python::AddPybindGetterSetter<_classname>::Register( \
          reinterpret_cast<uintptr_t>(this)); \
    } \
    public: ~GzSimPythonComponents##_classname() { \
      gz::sim::python::AddPybindGetterSetter<_classname>::Unregister( \
          reinterpret_cast<uintptr_t>(this)); \
    } \
  }; \
  static GzSimPythonComponents##_classname GzSimPythonComponentInitializer##_classname;

/// \def GZ_SIM_REGISTER_COMPONENT_WITH_PYTHON
/// \brief Register a component type with the core factory and also expose it
/// to the Python bindings.
#define GZ_SIM_REGISTER_COMPONENT_WITH_PYTHON(_compType, _classname) \
  GZ_SIM_REGISTER_COMPONENT(_compType, _classname) \
  GZ_SIM_REGISTER_PYTHON_COMPONENT(_classname)

#endif
