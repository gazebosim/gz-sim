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

#ifndef RCLPY__SERVER_HPP_
#define RCLPY__SERVER_HPP_

#include <pybind11/pybind11.h>

#include <ignition/gazebo/Server.hh>

#include <memory>

#include "../utils/destroyable.hh"
#include "server_config.hh"

namespace py = pybind11;

namespace ignition
{
namespace gazebo
{
namespace python
{

// class Server : public ignition::utils::python::Destroyable, public std::enable_shared_from_this<Server>
// {
// public:
//   Server(ignition::gazebo::python::ServerConfig &_config);
//
//   ~Server();
//
//   bool Run(const bool _blocking = false,
//            const uint64_t _iterations = 0,
//            const bool _paused = true);
//
//  bool IsRunning();
//
//  public: bool HasEntity(const std::string &_name,
//                         const unsigned int _worldIndex = 0) const;
//
//   /// Force an early destruction of this object
//   void
//   destroy() override;
//
// private:
//   std::shared_ptr<ignition::gazebo::Server> server_;
// };

/// Define a pybind11 wrapper for an ignition::gazebo::Server
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_gazebo_server(py::object module);

}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // RCLPY__SERVER_CONFIG_HPP_
