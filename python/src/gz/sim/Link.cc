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

#include "Link.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimLink(py::object module)
{
  py::class_<gz::sim::Link>(module, "Link")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Link>())
  .def("entity", &gz::sim::Link::Entity,
      "Get the entity which this Link is related to.")
  .def("reset_entity", &gz::sim::Link::ResetEntity,
      "Reset Entity to a new one.")
  .def("valid", &gz::sim::Link::Valid,
      py::arg("ecm"),
      "Check whether this link correctly refers to an entity that "
      "has a components::Link.")
  .def("name", &gz::sim::Link::Name,
      py::arg("ecm"),
      "Get the link's unscoped name.")
  .def("parent_model", &gz::sim::Link::ParentModel,
      py::arg("ecm"),
      "Get the parent model.")
  .def("is_canonical", &gz::sim::Link::IsCanonical,
      py::arg("ecm"),
      "Check if this is the canonical link.")
  .def("wind_mode", &gz::sim::Link::WindMode,
      py::arg("ecm"),
      "Get whether this link has wind enabled.")
  .def("collision_by_name", &gz::sim::Link::CollisionByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a collision entity which is an immediate child of "
      "this link.")
  .def("sensor_by_name", &gz::sim::Link::SensorByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a sensor entity which is an immediate child of "
      "this link.")
  .def("visual_by_name", &gz::sim::Link::VisualByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a visual entity which is an immediate child of "
      "this link.")
  .def("collisions", &gz::sim::Link::Collisions,
      py::arg("ecm"),
      "Get all collisions which are immediate children of this link.")
  .def("sensors", &gz::sim::Link::Sensors,
      py::arg("ecm"),
      "Get all sensors which are immediate children of this link.")
  .def("visuals", &gz::sim::Link::Visuals,
      py::arg("ecm"),
      "Get all visuals which are immediate children of this link.")
  .def("collision_count", &gz::sim::Link::CollisionCount,
      py::arg("ecm"),
      "Get the number of collisions which are immediate children of "
      "this link.")
  .def("sensor_count", &gz::sim::Link::SensorCount,
      py::arg("ecm"),
      "Get the number of sensors which are immediate children of this "
      "link.")
  .def("visual_count", &gz::sim::Link::VisualCount,
      py::arg("ecm"),
      "Get the number of visuals which are immediate children of this "
      "link.")
  .def("world_pose", &gz::sim::Link::WorldPose,
      py::arg("ecm"),
      "Get the pose of the link frame in the world coordinate frame.")
  .def("world_inertial", &gz::sim::Link::WorldInertial,
      py::arg("ecm"),
      "Get the inertia of the link in the world frame.")
  .def("world_inertial_pose", &gz::sim::Link::WorldInertialPose,
      py::arg("ecm"),
      "Get the world pose of the link inertia.")
  .def("world_linear_velocity",
      py::overload_cast<const EntityComponentManager &>
        (&gz::sim::Link::WorldLinearVelocity, py::const_),
      py::arg("ecm"),
      "Get the linear velocity at the origin of of the link frame "
      "expressed in the world frame, using an offset expressed in a "
      "body-fixed frame. If no offset is given, the velocity at the origin of "
      "the Link frame will be returned.")
  .def("world_linear_velocity",
      py::overload_cast<const EntityComponentManager &, const math::Vector3d &>
        (&gz::sim::Link::WorldLinearVelocity, py::const_),
      py::arg("ecm"),
      py::arg("offset"),
      "Get the linear velocity of a point on the body in the world "
      "frame, using an offset expressed in a body-fixed frame.")
  .def("world_angular_velocity", &gz::sim::Link::WorldAngularVelocity,
      py::arg("ecm"),
      "Get the angular velocity of the link in the world frame.")
  .def("enable_velocity_checks", &gz::sim::Link::EnableVelocityChecks,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report velocities for a link, so "
      "functions like `world_linear_velocity` will return nullopt. This "
      "function can be used to enable all velocity checks.")
  .def("set_linear_velocity", &gz::sim::Link::SetLinearVelocity,
      py::arg("ecm"),
      py::arg("velocity"),
      "Set the linear velocity on this link. If this is set, wrenches "
      "on this link will be ignored for the current time step.")
  .def("set_angular_velocity", &gz::sim::Link::SetAngularVelocity,
      py::arg("ecm"),
      py::arg("velocity"),
      "Set the angular velocity on this link. If this is set, wrenches "
      "on this link will be ignored for the current time step.")
  .def("world_angular_acceleration", &gz::sim::Link::WorldAngularAcceleration,
      py::arg("ecm"),
      "Get the linear acceleration of the body in the world frame.")
  .def("world_linear_acceleration", &gz::sim::Link::WorldLinearAcceleration,
      py::arg("ecm"),
      "Get the angular velocity of the link in the world frame.")
  .def("enable_acceleration_checks", &gz::sim::Link::EnableAccelerationChecks,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report accelerations for a link, so "
      "functions like `world_linear_acceleration` will return nullopt. This "
      "function can be used to enable all acceleration checks.")
  .def("world_inertia_matrix", &gz::sim::Link::WorldInertiaMatrix,
      py::arg("ecm"),
      "Get the inertia matrix in the world frame.")
  .def("world_kinetic_energy", &gz::sim::Link::WorldKineticEnergy,
      py::arg("ecm"),
      "Get the rotational and translational kinetic energy of the "
      "link with respect to the world frame.")
  .def("add_world_force",
      py::overload_cast<EntityComponentManager &, const math::Vector3d &>
        (&gz::sim::Link::AddWorldForce, py::const_),
      py::arg("ecm"),
      py::arg("force"),
      "Add a force expressed in world coordinates and applied at the "
      "center of mass of the link.")
  .def("add_world_force",
      py::overload_cast<EntityComponentManager &,
                        const math::Vector3d &,
                        const math::Vector3d &>
                        (&gz::sim::Link::AddWorldForce, py::const_),
      py::arg("ecm"),
      py::arg("force"),
      py::arg("position"),
      "Add a force expressed in world coordinates and applied at "
      "an offset from the center of mass of the link.")
  .def("add_world_wrench",
      py::overload_cast<EntityComponentManager &,
                        const math::Vector3d &,
                        const math::Vector3d &>
        (&gz::sim::Link::AddWorldWrench, py::const_),
      py::arg("ecm"),
      py::arg("force"),
      py::arg("torque"),
      "Add a wrench expressed in world coordinates and applied to "
      "the link at the link's origin. This wrench is applied for one "
      "simulation step.")
  .def("add_world_wrench",
      py::overload_cast<EntityComponentManager &,
                        const math::Vector3d &,
                        const math::Vector3d &,
                        const math::Vector3d &>
        (&gz::sim::Link::AddWorldWrench, py::const_),
      py::arg("ecm"),
      py::arg("force"),
      py::arg("torque"),
      py::arg("offset"),
      "Add a wrench expressed in world coordinates and applied to "
      "the link at an offset from the link's origin. This wrench is applied "
      "for one simulation step.")
  .def("enable_bounding_box_checks", &gz::sim::Link::EnableBoundingBoxChecks,
      py::arg("ecm"),
      py::arg("enable") = true,
      "By default, Gazebo will not report bounding box for a link, so "
      "functions like `axis_aligned_box` and `world_axis_aligned_box` will "
      "return nullopt. This function can be used to enable bounding box checks "
      "and it also initializes the bounding box based on the link's collision "
      "shapes.")
  .def("axis_aligned_box", &gz::sim::Link::AxisAlignedBox,
      py::arg("ecm"),
      "Get the Link's axis-aligned box represented in the link frame.")
  .def("world_axis_aligned_box", &gz::sim::Link::WorldAxisAlignedBox,
      py::arg("ecm"),
      "Get the Link's axis-aligned box represented in the world frame.")
  .def("compute_axis_aligned_box", &gz::sim::Link::ComputeAxisAlignedBox,
      py::arg("ecm"),
      "Compute the Link's axis-aligned box represented in the link frame. "
      "It generates an axis-aligned bounding box for a link based on the "
      "collision shapes attached to the it. The link bounding box is "
      "generated by merging all the bounding boxes of the collision "
      "geometry shapes attached to the link.")
  .def("__copy__",
      [](const gz::sim::Link &self)
      {
        return gz::sim::Link(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Link &self, pybind11::dict)
      {
        return gz::sim::Link(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
