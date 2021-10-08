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

#include <iostream>

#include "server.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
void
Server::destroy()
{
  server_.reset();
}

Server::~Server()
{

}

Server::Server(ignition::gazebo::python::ServerConfig &_config)
{
  server_ = std::make_shared<ignition::gazebo::Server>(*_config.rcl_ptr());
}

bool Server::Run(const bool _blocking, const uint64_t _iterations,
  const bool _paused)
{
  return server_->Run(_blocking, _iterations, _paused);
}

bool Server::IsRunning()
{
  return server_->Running();
}

bool Server::HasEntity(const std::string &_name,
                       const unsigned int _worldIndex) const
{
  return server_->HasEntity(_name, _worldIndex);
}

void define_gazebo_server(py::object module)
{
  py::class_<Server, ignition::utils::python::Destroyable, std::shared_ptr<Server>>(module, "Server")
  .def(py::init<ignition::gazebo::python::ServerConfig &>())
  .def(
    "run", &Server::Run,
    "Run server")
  .def(
    "has_entity", &Server::HasEntity,
    "Return true if the specified world has an entity with the provided name.")
  .def(
    "is_running", &Server::IsRunning,
    "");
}

}  // namespace python
}  // namespace gazebo
}  // namespace ignition
