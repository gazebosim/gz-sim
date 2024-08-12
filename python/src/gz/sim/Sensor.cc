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

#include "Sensor.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimSensor(py::object module)
{
  py::class_<gz::sim::Sensor>(module, "Sensor")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Sensor>())
  .def("entity", &gz::sim::Sensor::Entity,
      "Get the entity which this sensor is related to.")
  .def("reset_entity", &gz::sim::Sensor::ResetEntity,
      "Reset Entity to a new one.")
  .def("valid", &gz::sim::Sensor::Valid,
      py::arg("ecm"),
      "Check whether this sensor correctly refers to an entity that"
      "has a components::Sensor.")
  .def("name", &gz::sim::Sensor::Name,
      py::arg("ecm"),
      "Get the sensor's unscoped name.")
  .def("pose", &gz::sim::Sensor::Pose,
      py::arg("ecm"),
      "Get the pose of the sensor. "
      "If the sensor has a trajectory, this will only return the origin"
      "pose of the trajectory and not the actual world pose of the sensor.")
  .def("topic", &gz::sim::Sensor::Topic,
      py::arg("ecm"),
      "Get the topic of the sensor.")
  .def("parent", &gz::sim::Sensor::Parent,
      py::arg("ecm"),
      "Get the parent entity. This can be a link or a joint.")
  .def("__copy__",
      [](const gz::sim::Sensor &self)
      {
        return gz::sim::Sensor(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Sensor &self, pybind11::dict)
      {
        return gz::sim::Sensor(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
