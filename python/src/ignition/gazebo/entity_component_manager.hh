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

#ifndef IGNITION_GAZEBO_PYTHON__ENTITY_COMPONENT_MANAGER_HH_
#define IGNITION_GAZEBO_PYTHON__ENTITY_COMPONENT_MANAGER_HH_

#include <pybind11/pybind11.h>

#include "ignition/gazebo/EntityComponentManager.hh"

#include "destroyable.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{

  class EntityComponentManager :
    public ignition::gazebo::python::Destroyable,
    public std::enable_shared_from_this<EntityComponentManager>
  {
    /// \brief Constructor
    public: EntityComponentManager(
      const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Constructor
    public: EntityComponentManager(
      ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Destructor
    public: ~EntityComponentManager();

    /// Force an early destruction of this object
    void
    Destroy() override;

    /// Get rcl_client_t pointer
    ignition::gazebo::EntityComponentManager *
    rcl_ptr() const
    {
      return _entity_component_manager_no_const;
    }

  private:
    const ignition::gazebo::EntityComponentManager * _entity_component_manager;
    ignition::gazebo::EntityComponentManager *
      _entity_component_manager_no_const;
  };

/// Define a pybind11 wrapper for an ignition::gazebo::EntityComponentManager
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_gazebo_entity_component_manager(pybind11::object module);

}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_PYTHON__ENTITY_COMPONENT_MANAGER_HH_
