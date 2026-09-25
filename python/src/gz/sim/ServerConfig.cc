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

#include <gz/sim/ServerConfig.hh>

#include "ServerConfig.hh"

namespace gz
{
namespace sim
{
namespace python
{
void defineSimServerConfig(pybind11::object module)
{
  pybind11::class_<gz::sim::ServerConfig>(module, "ServerConfig",
    "Configuration for a simulation server. This class provides options "
    "for setting up and initializing a server, such as specifying the SDF file to use. ")
  .def(pybind11::init<>())
  .def(
    "set_sdf_file", &gz::sim::ServerConfig::SetSdfFile,
    pybind11::arg("file"),
    "Set an SDF file to be used with the server. This overrides any value "
    "set by set_sdf_string. Returns False if the path is empty.")
  .def(
    "sdf_file", &gz::sim::ServerConfig::SdfFile,
    "Get the SDF file that has been set. An empty string will be returned "
    "if an SDF file has not been set.")
  .def(
    "set_sdf_string", &gz::sim::ServerConfig::SetSdfString,
    pybind11::arg("sdf_string"),
    "Set an SDF string to be used with the server. This overrides any value "
    "set by set_sdf_file.")
  .def(
    "sdf_string", &gz::sim::ServerConfig::SdfString,
    "Get the SDF String that has been set. An empty string will be returned "
    "if an SDF string has not been set.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
