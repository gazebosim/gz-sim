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
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

#include "Server.hh"
#include "wrap_functions.hh"

namespace gz
{
namespace sim
{
namespace python
{
void defineSimServer(pybind11::object module)
{
  pybind11::class_<gz::sim::Server,
    std::shared_ptr<gz::sim::Server>>(module, "Server",
    "The main simuulation server class. This class manages the simulation "
    "execution and provides control over running, pausing, and stepping "
    "through simulations.")
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
    "Get whether the server is running.")
  .def("reset_all", &gz::sim::Server::ResetAll,
    "Resets all simulation runners under this server.")
  .def("reset", &gz::sim::Server::Reset,
    "Resets a specific simulation runner under this server.")
  .def("peek_ecm", &gz::sim::Server::PeekEcm,
    pybind11::arg("_func"), pybind11::arg("_runnerID") = 0,
    "This function allows one to introspect the state of the ecm when the "
    "server is not running. This may be useful for unit tests.")
  .def("poke_ecm", WrapCallbacks(&gz::sim::Server::PokeEcm),
    pybind11::arg("_func"), pybind11::arg("_runnerID") = 0,
    "This function allows one to modify the state of the ecm when the "
    "server is not running. This may be useful for unit tests.")
  .def("iteration_count", &gz::sim::Server::IterationCount,
    pybind11::arg("_worldIndex") = 0,
    "Get the number of iterations the server has executed.")
  .def("sim_time", &gz::sim::Server::SimTime,
    pybind11::arg("_worldIndex") = 0,
    "Get the current simulation time.")
  .def("entity_count", &gz::sim::Server::EntityCount,
    pybind11::arg("_worldIndex") = 0,
    "Get the number of entities on the server.")
  .def("system_count", &gz::sim::Server::SystemCount,
    pybind11::arg("_worldIndex") = 0,
    "Get the number of systems on the server.");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
