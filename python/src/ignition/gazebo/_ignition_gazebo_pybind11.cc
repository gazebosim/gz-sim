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

#include "../utils/destroyable.hh"
#include "../utils/exceptions.hh"
#include "server_config.hh"
#include "server.hh"

namespace py = pybind11;

PYBIND11_MODULE(ignition_gazebo, m) {
  m.doc() = "Ignition Gazebo Python Library.";

  ignition::utils::python::define_destroyable(m);

  py::register_exception<ignition::utils::python::InvalidHandle>(
    m, "InvalidHandle", PyExc_RuntimeError);

  ignition::gazebo::python::define_server_config(m);
  ignition::gazebo::python::define_gazebo_server(m);
}
