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

#ifndef RCLPY__POSE_V_HPP_
#define RCLPY__POSE_V_HPP_

#include <pybind11/pybind11.h>

#include <ignition/msgs/pose_v.pb.h>

#include <ignition/math/Vector3.hh>

#include "../utils/destroyable.hh"
#include "swig.hh"

namespace py = pybind11;

namespace ignition
{
namespace msgs
{
namespace python
{
class PoseV : public ignition::utils::python::Destroyable, public std::enable_shared_from_this<PoseV>
{
public:
  PoseV(const ignition::msgs::Pose_V &_msg);
  PoseV();

  ~PoseV();

  void show();

  std::vector<std::string> GetNames();
  int GetSize();
  std::vector<ignition::math::Vector3d> GetPoses();
  py::object Pose();
  void SetPose(py::object _v);

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  ignition::msgs::Pose_V msgs_;
};

/// Define a pybind11 wrapper for an ignition::gazebo::Node
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_msgs_pose_v(py::object module);
}
}
}

#endif
