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
  this->run = false;
  workerThread.join();
}

void Node::worker()
{
  std::cerr << "worker" << '\n';
  while(this->run)
  {
    mutex.lock();
    // std::cerr << "this->vectorPoses.size() " << this->vectorPoses.size() << '\n';
    if (this->vectorPoses.size() > 0)
    {
      this->vectorPoses.erase(this->vectorPoses.begin());
      // this->cb_(ignition::msgs::python::PoseV(_ignMsg));
    }
    mutex.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // std::cerr << "worker step" << '\n';

  }
}

Node::Node()
{
  node_ = std::make_shared<ignition::transport::Node>();
  // workerThread = std::thread(&Node::worker, this);
}

void Node::PoseVCallback(const msgs::Pose_V &_ignMsg)
{
  // std::cout << "Node::PoseVCallback" << std::endl;
  // // mutex.lock();
  // // this->vectorPoses.push_back(1);
  // // mutex.unlock();
  // // const auto msg = ignition::msgs::python::PoseV(_ignMsg);
  // this->cb_(ignition::msgs::python::PoseV(_ignMsg));
  // // auto t = std::thread([&]()-> void {this->cb_(); } );
  // // t.join();
  // return;
}

bool Node::SubscribePoseV(
    const std::string &_topic,
    std::function<void(const ignition::msgs::python::PoseV &_msg)> &_cb)
{
  this->cb_ = _cb;//[&]() -> void { std::cout << " function() " << std::endl; }; //std::bind(_cb);
  auto poseVCb = [&](const msgs::Pose_V &_ignMsg) -> void
  {
    std::cout << "callback" << std::endl;
    // mutex.lock();
    // this->vectorPoses.push_back(1);
    // mutex.unlock();
    // const ignition::msgs::python::PoseV msgPython(_ignMsg);
    py::object obj;// = py::cast(msgPython);
    this->cb_(_ignMsg);
    // _cb(ignition::msgs::python::PoseV(_ignMsg));
    // this->cb_();
    // std::lock_guard<std::mutex> lock(contactMutex);
    // contactMsgs.push_back(_msg);
  };
  //
  auto callbackFunc = std::function<void(const msgs::Pose_V &)>(poseVCb);

  return this->node_->Subscribe(_topic, callbackFunc);
  // return this->node_->Subscribe(_topic, &Node::PoseVCallback, this);
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
