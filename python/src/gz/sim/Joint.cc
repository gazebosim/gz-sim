/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include <pybind11/stl.h>

#include <iostream>

#include "Joint.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimJoint(py::object module)
{
  py::class_<gz::sim::Joint>(module, "Joint")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Joint>())
  .def("entity", &gz::sim::Joint::Entity,
      "Get the entity which this Joint is related to.")
  .def("reset_entity", &gz::sim::Joint::ResetEntity,
      py::arg("new_entity"),
      "Reset Entity to a new one.")
  .def("valid", &gz::sim::Joint::Valid,
      py::arg("ecm"),
      "Check whether this joint correctly refers to an entity that"
      "has a components::Joint.")
  .def("name", &gz::sim::Joint::Name,
      py::arg("ecm"),
      "Get the joint's unscoped name.")
  .def("parent_link_name", &gz::sim::Joint::ParentLinkName,
      py::arg("ecm"),
      "Get the parent link name.")
  .def("child_link_name", &gz::sim::Joint::ChildLinkName,
      py::arg("ecm"),
      "Get the child link name.")
  .def("pose", &gz::sim::Joint::Pose,
      py::arg("ecm"),
      "Get the pose of the joint.")
  .def("thread_pitch", &gz::sim::Joint::ThreadPitch,
      py::arg("ecm"),
      "Get the thread pitch of the joint.")
  .def("axis", &gz::sim::Joint::Axis,
      py::arg("ecm"),
      "Get the joint axis.")
  .def("type", &gz::sim::Joint::Type,
      py::arg("ecm"),
      "Get the joint type.")
  .def("sensor_by_name", &gz::sim::Joint::SensorByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a sensor entity which is an immediate child of "
      "this joint.")
  .def("sensors", &gz::sim::Joint::Sensors,
      py::arg("ecm"),
      "Get all sensors which are immediate children of this joint.")
  .def("sensor_count", &gz::sim::Joint::SensorCount,
      py::arg("ecm"),
      "Get the number of sensors which are immediate children of this "
      "joint.")
  .def("set_velocity", &gz::sim::Joint::SetVelocity,
      py::arg("ecm"),
      py::arg("velocities"),
      "Set velocity on this joint. Only applied if no forces are set.")
  .def("set_force", &gz::sim::Joint::SetForce,
      py::arg("ecm"),
      py::arg("forces"),
      "Set force on this joint. If both forces and velocities are set, "
      "only forces are applied.")
  .def("set_velocity_limits", &gz::sim::Joint::SetVelocityLimits,
      py::arg("ecm"),
      py::arg("limits"),
      "Set the velocity limits on a joint axis.")
  .def("set_effort_limits", &gz::sim::Joint::SetEffortLimits,
      py::arg("ecm"),
      py::arg("limits"),
      "Set the effort limits on a joint axis.")
  .def("set_position_imits", &gz::sim::Joint::SetPositionLimits,
      py::arg("ecm"),
      py::arg("limits"),
      "Set the position limits on a joint axis.")
  .def("reset_position", &gz::sim::Joint::ResetPosition,
      py::arg("ecm"),
      py::arg("positions"),
      "Reset the joint positions.")
  .def("reset_velocity", &gz::sim::Joint::ResetVelocity,
      py::arg("ecm"),
      py::arg("velocities"),
      "Reset the joint velocities.")
  .def("enable_velocity_check", &gz::sim::Joint::EnableVelocityCheck,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report velocities for a joint, so "
      "functions like `Velocity` will return nullopt. This "
      "function can be used to enable joint velocity check.")
  .def("enable_position_check", &gz::sim::Joint::EnablePositionCheck,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report positions for a joint, so "
      "functions like `Position` will return nullopt. This "
      "function can be used to enable joint position check.")
  .def("enable_transmitted_wrench_check",
      &gz::sim::Joint::EnableTransmittedWrenchCheck,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report transmitted wrench for a "
      "joint, so functions like `TransmittedWrench` will return nullopt. This "
      "function can be used to enable joint transmitted wrench check.")
  .def("velocity", &gz::sim::Joint::Velocity,
      py::arg("ecm"),
      "Get the velocity of the joint.")
  .def("position", &gz::sim::Joint::Position,
      py::arg("ecm"),
      "Get the position of the joint.")
  .def("transmitted_wrench", &gz::sim::Joint::TransmittedWrench,
      py::arg("ecm"),
      "Get the transmitted wrench of the joint.")
  .def("parent_model", &gz::sim::Joint::ParentModel,
      py::arg("ecm"),
      "Get the parent model of the joint.")
  .def("__copy__",
      [](const gz::sim::Joint &self)
      {
        return gz::sim::Joint(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Joint &self, pybind11::dict)
      {
        return gz::sim::Joint(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
