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

#include <iostream>

#include "World.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
void defineGazeboWorld(pybind11::object module)
{
  pybind11::class_<ignition::gazebo::World>(module, "World")
  .def(pybind11::init<ignition::gazebo::Entity>())
  .def(
    "model_by_name", &ignition::gazebo::World::ModelByName,
    "Get the ID of a model entity which is an immediate child of "
    " this world.")
  .def(
    "gravity", &ignition::gazebo::World::Gravity,
    "Get the gravity in m/s^2.");
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition
