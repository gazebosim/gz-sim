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
#include <chrono>

#include "node.hh"

namespace ignition
{
namespace transport
{
namespace python
{
void
Node::destroy()
{
  node_.reset();
}

Node::~Node()
{
}

Node::Node()
{
  node_ = std::make_shared<ignition::transport::Node>();
}

bool Node::SubscribePoseV(
    const std::string &_topic,
    std::function<void(const ignition::msgs::python::PoseV &_msg)> &_cb)
{
  this->cb_ = _cb;
  auto poseVCb = [&](const msgs::Pose_V &_ignMsg) -> void
  {
    this->cb_(_ignMsg);
  };
  auto callbackFunc = std::function<void(const msgs::Pose_V &)>(poseVCb);

  return this->node_->Subscribe(_topic, callbackFunc);
}

void define_transport_node(py::object module)
{
  py::class_<Node, ignition::utils::python::Destroyable, std::shared_ptr<Node>>(module, "Node")
  .def(py::init<>())
  .def("subscribe", &Node::SubscribePoseV,
  "");
}

}  // namespace python
}  // namespace gazebo
}  // namespace ignition
