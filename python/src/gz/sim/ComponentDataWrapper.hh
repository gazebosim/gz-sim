/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_PYTHON_COMPONENTDATAWRAPPER_HH_
#define GZ_SIM_PYTHON_COMPONENTDATAWRAPPER_HH_

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <pybind11/pybind11.h>

namespace gz
{
namespace sim
{
namespace python
{
  /// \brief A wrapper class for getting and setting component data.
  class ComponentDataWrapper
  {
    public: ComponentDataWrapper(
        gz::sim::EntityComponentManager *_ecm,
        const gz::sim::Entity &_entity,
        const gz::sim::ComponentTypeId &_typeId);

    /// \brief Get the component data as a python object.
    /// \return The python object.
    public: pybind11::object Data() const;

    /// \brief Set the component data from a python object.
    /// \param[in] _obj The python object.
    public: void SetData(const pybind11::object &_obj);

    public: gz::sim::ComponentTypeId typeId;

    private: gz::sim::EntityComponentManager *ecm;
    private: gz::sim::Entity entity;
  };
}  // namespace python
}  // namespace sim
}  // namespace gz

#endif
