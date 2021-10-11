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

#ifndef RCLPY__NODE_HPP_
#define RCLPY__NODE_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include <ignition/transport/Node.hh>

#include <ignition/common/Console.hh>

#include <memory>
#include <thread>

#include "../msgs/pose_v.hh"
#include <ignition/msgs/pose_v.pb.h>

#include "../utils/destroyable.hh"

namespace py = pybind11;

namespace ignition
{
namespace transport
{
namespace python
{

class Node : public ignition::utils::python::Destroyable, public std::enable_shared_from_this<Node>
{
public:
  Node();

  ~Node();

  /// Force an early destruction of this object
  void
  destroy() override;

  bool SubscribePoseV(
      const std::string &_topic,
      std::function<void(const ignition::msgs::python::PoseV &_msg)> &_cb);
private:
  std::shared_ptr<ignition::transport::Node> node_;

  std::function<void(const ignition::msgs::python::PoseV &_msg)> cb_;

  std::mutex mutex;

};

/// Define a pybind11 wrapper for an ignition::gazebo::Node
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_transport_node(py::object module);

}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // RCLPY__NODE_HPP_
