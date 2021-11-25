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

#ifndef IGNITION_GAZEBO_PYTHON__WORLD_HPP_
#define IGNITION_GAZEBO_PYTHON__WORLD_HPP_

#include <pybind11/pybind11.h>

#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include "entity_component_manager.hh"

#include <memory>

namespace ignition
{
namespace gazebo
{
namespace python
{
class World : public ignition::gazebo::python::Destroyable,
              public std::enable_shared_from_this<World>
{
  public: World(ignition::gazebo::Entity _entity = kNullEntity);
  public: ~World();

  /// \brief Get the gravity in m/s^2.
  /// \param[in] _ecm Entity-component manager.
  /// \return Gravity vector or nullopt if the entity does not
  /// have a components::Gravity component.
  public: ignition::math::Vector3<double>
    Gravity(const EntityComponentManager &_ecm);

  /// \brief Get the ID of a model entity which is an immediate child of
  /// this world.
  /// \param[in] _ecm Entity-component manager.
  /// \param[in] _name Model name.
  /// \return Model entity.
  public: ignition::gazebo::Entity ModelByName(
    ignition::gazebo::python::EntityComponentManager &_ecm,
    std::string _name);

  /// Force an early destruction of this object
  void Destroy() override;

private:
  std::shared_ptr<ignition::gazebo::World> _world;
};

/// Define a pybind11 wrapper for an ignition::gazebo::Server
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_gazebo_world(pybind11::object module);

}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_PYTHON__SERVER_CONFIG_HPP_
