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

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>

#include "Actor.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimActor(py::object module)
{
  py::class_<gz::sim::Actor>(module, "Actor")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Actor>())
  .def("entity", &gz::sim::Actor::Entity,
      "Get the entity which this actor is related to.")
  .def("reset_entity", &gz::sim::Actor::ResetEntity,
      "Reset Entity to a new one.")
  .def("valid", &gz::sim::Actor::Valid,
      py::arg("ecm"),
      "Check whether this actor correctly refers to an entity that "
      "has a components::Actor.")
  .def("name", &gz::sim::Actor::Name,
      py::arg("ecm"),
      "Get the actor's unscoped name.")
  .def("pose", &gz::sim::Actor::Pose,
      py::arg("ecm"),
      "Get the pose of the actor."
      "If the actor has a trajectory, this will only return the origin "
      "pose of the trajectory and not the actual world pose of the actor.")
  .def("trajectory_pose", &gz::sim::Actor::TrajectoryPose,
      py::arg("ecm"),
      "Get the trajectory pose of the actor. There are two "
      "ways that the actor can follow a trajectory: 1) SDF script, "
      "2) manually setting trajectory pose. This function retrieves 2) the "
      "manual trajectory pose set by the user. The Trajectory pose is "
      "given relative to the trajectory pose origin returned by Pose().")
  .def("set_trajectory_pose", &gz::sim::Actor::SetTrajectoryPose,
      py::arg("ecm"),
      py::arg("pose"),
      "Set the trajectory pose of the actor. There are two "
      "ways that the actor can follow a trajectory: 1) SDF script, "
      "2) manually setting trajectory pose. This function enables option 2). "
      "Manually setting the trajectory pose will override the scripted "
      "trajectory specified in SDF.")
  .def("world_pose", &gz::sim::Actor::WorldPose,
      py::arg("ecm"),
      "Get the world pose of the actor."
      "This returns the current world pose of the actor computed by gazebo."
      "The world pose is the combination of the actor's pose and its "
      "trajectory pose. The currently trajectory pose is either manually set "
      "via set_trajectory_pose or interpolated from waypoints in the SDF "
      "script based on the current time.")
  .def("set_animation_name", &gz::sim::Actor::SetAnimationName,
      py::arg("ecm"),
      py::arg("name"),
      "Set the name of animation to use for this actor.")
  .def("set_animation_time", &gz::sim::Actor::SetAnimationTime,
      py::arg("ecm"),
      py::arg("time"),
      "Set the time of animation to use for this actor (the time argument "
      "is expected in ms).")
  .def("animation_name", &gz::sim::Actor::AnimationName,
      py::arg("ecm"),
      "Get the name of animation used by the actor.")
  .def("animation_time", &gz::sim::Actor::AnimationTime,
      py::arg("ecm"),
      "Get the time of animation for this actor.")
  .def("__copy__",
      [](const gz::sim::Actor &self)
      {
        return gz::sim::Actor(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Actor &self, pybind11::dict)
      {
        return gz::sim::Actor(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
