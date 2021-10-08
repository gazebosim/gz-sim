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

#ifndef RCLPY__SERVER_CONFIG_HPP_
#define RCLPY__SERVER_CONFIG_HPP_

#include <pybind11/pybind11.h>

#include <ignition/gazebo/ServerConfig.hh>

#include <memory>

#include "../utils/destroyable.hh"

namespace py = pybind11;

namespace ignition
{
namespace gazebo
{
namespace python
{

class ServerConfig : public ignition::utils::python::Destroyable, public std::enable_shared_from_this<ServerConfig>
{
public:
  ServerConfig();

  ~ServerConfig();

  bool SetSdfFile(const std::string &_file);

  /// Force an early destruction of this object
  void
  destroy() override;

  /// Get rcl_wait_set_t pointer
  ignition::gazebo::ServerConfig * rcl_ptr() const
  {
    return server_config_.get();
  }

private:
  std::shared_ptr<ignition::gazebo::ServerConfig> server_config_;
};

/// Define a pybind11 wrapper for an ignition::gazebo::ServerConfig
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_server_config(py::object module);
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // RCLPY__SERVER_CONFIG_HPP_
