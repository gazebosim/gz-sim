// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/pybind11.h>

#include "destroyable.hh"
#include "entity_component_manager.hh"
#include "event_manager.hh"
#include "exceptions.hh"
#include "helper_system.hh"
#include "server.hh"
#include "server_config.hh"
#include "update_info.hh"
#include "world.hh"

PYBIND11_MODULE(gazebo, m) {
  m.doc() = "Ignition Gazebo Python Library.";

  ignition::gazebo::python::define_destroyable(m);

  pybind11::register_exception<ignition::gazebo::python::InvalidHandle>(
    m, "InvalidHandle", PyExc_RuntimeError);

  ignition::gazebo::python::define_gazebo_entity_component_manager(m);
  ignition::gazebo::python::define_gazebo_event_manager(m);
  ignition::gazebo::python::define_gazebo_helper_fixture(m);
  ignition::gazebo::python::define_gazebo_server(m);
  ignition::gazebo::python::define_gazebo_update_info(m);
  ignition::gazebo::python::define_gazebo_world(m);
  ignition::gazebo::python::define_server_config(m);
}
