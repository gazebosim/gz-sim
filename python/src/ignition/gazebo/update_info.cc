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
#include <pybind11/chrono.h>

#include <iostream>

#include "update_info.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
void define_gazebo_update_info(py::object module)
{
  py::class_<ignition::gazebo::UpdateInfo>(module, "UpdateInfo")
  .def(py::init<>())
  .def_readwrite("sim_time", &ignition::gazebo::UpdateInfo::simTime)
  .def_readwrite("real_time", &ignition::gazebo::UpdateInfo::realTime)
  .def_readwrite("dt", &ignition::gazebo::UpdateInfo::dt)
  .def_readwrite("paused", &ignition::gazebo::UpdateInfo::paused)
  .def_readwrite("iterations", &ignition::gazebo::UpdateInfo::iterations);
}

}  // namespace python
}  // namespace gazebo
}  // namespace ignition
