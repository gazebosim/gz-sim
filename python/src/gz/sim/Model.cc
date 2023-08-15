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

#include "Model.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimModel(py::object module)
{
  py::class_<gz::sim::Model>(module, "Model")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Model>())
  .def("entity", &gz::sim::Model::Entity,
      "Get the entity which this Model is related to.")
  .def("valid", &gz::sim::Model::Valid,
      py::arg("ecm"),
      "Check whether this model correctly refers to an entity that "
      "has a components::Model.")
  .def("name", &gz::sim::Model::Name,
      py::arg("ecm"),
      "Get the model's unscoped name.")
  .def("static", &gz::sim::Model::Static,
      py::arg("ecm"),
      "Get whether this model is static.")
  .def("self_collide", &gz::sim::Model::SelfCollide,
      py::arg("ecm"),
      "Get whether this model has self-collide enabled.")
  .def("wind_mode", &gz::sim::Model::WindMode,
      py::arg("ecm"),
      "Get whether this model has wind enabled.")
  .def("source_file_path", &gz::sim::Model::SourceFilePath,
      py::arg("ecm"),
      "Get the source file where this model came from. If empty,"
      "the model wasn't loaded directly from a file, probably from an SDF "
      "string.")
  .def("joint_by_name", &gz::sim::Model::JointByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a joint entity which is an immediate child of "
      "this model.")
  .def("link_by_name", &gz::sim::Model::LinkByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a link entity which is an immediate child of "
      "this model.")
  .def("joints", &gz::sim::Model::Joints,
      py::arg("ecm"),
      "Get all joints which are immediate children of this model.")
  .def("links", &gz::sim::Model::Links,
      py::arg("ecm"),
      "Get all links which are immediate children of this model.")
  .def("models", &gz::sim::Model::Models,
      py::arg("ecm"),
      "Get all models which are immediate children of this model.")
  .def("joint_count", &gz::sim::Model::JointCount,
      py::arg("ecm"),
      "Get the number of joints which are immediate children of this "
      "model.")
  .def("link_count", &gz::sim::Model::LinkCount,
      py::arg("ecm"),
      "Get the number of links which are immediate children of this "
      "model.")
  .def("set_world_pose_cmd", &gz::sim::Model::SetWorldPoseCmd,
      py::arg("ecm"),
      py::arg("pose"),
      "Set a command to change the model's pose.")
  .def("canonical_link", &gz::sim::Model::CanonicalLink,
      py::arg("ecm"),
      "Get the model's canonical link entity.")
  .def("__copy__",
      [](const gz::sim::Model &self)
      {
        return gz::sim::Model(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Model &self, pybind11::dict)
      {
        return gz::sim::Model(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
