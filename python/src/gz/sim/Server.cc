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

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

#include "Server.hh"

namespace gz
{
namespace sim
{
namespace python
{
void defineSimServer(pybind11::object module)
{
  pybind11::class_<gz::sim::Server,
    std::shared_ptr<gz::sim::Server>>(module, "Server")
  .def(pybind11::init<gz::sim::ServerConfig &>())
  .def(
    "run", &gz::sim::Server::Run,
    pybind11::call_guard<pybind11::gil_scoped_release>(),
    "Run the server. By default this is a non-blocking call, "
    " which means the server runs simulation in a separate thread. Pass "
    " in true to the _blocking argument to run the server in the current "
    " thread.")
  .def(
    "has_entity", &gz::sim::Server::HasEntity,
    "Return true if the specified world has an entity with the provided name.")
  .def(
    "is_running",
    pybind11::overload_cast<>(&gz::sim::Server::Running, pybind11::const_),
    "Get whether the server is running.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
