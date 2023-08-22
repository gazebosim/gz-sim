/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "World.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimWorld(py::object module)
{
  py::class_<gz::sim::World>(module, "World")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::World>())
  .def("entity", &gz::sim::World::Entity,
      "Get the entity which this World is related to.")
  .def("valid", &gz::sim::World::Valid,
      py::arg("ecm"),
      "Check whether this world correctly refers to an entity that "
      "has a components::World.")
  .def("name", &gz::sim::World::Name,
      py::arg("ecm"),
      "Get the world's unscoped name.")
  .def("gravity", &gz::sim::World::Gravity,
      py::arg("ecm"),
      "Get the gravity in m/s^2.")
  .def("magnetic_field", &gz::sim::World::MagneticField,
      py::arg("ecm"),
      "Get the magnetic field in Tesla.")
  .def("atmosphere", &gz::sim::World::Atmosphere,
      py::arg("ecm"),
      "Get atmosphere information.")
  .def("spherical_coordinates", &gz::sim::World::SphericalCoordinates,
      py::arg("ecm"),
      "Get spherical coordinates for the world origin.")
  .def("set_spherical_coordinates", &gz::sim::World::SetSphericalCoordinates,
      py::arg("ecm"),
      py::arg("spherical_coordinates"),
      "Set spherical coordinates for the world origin.")
  .def("light_by_name", &gz::sim::World::LightByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a light entity which is an immediate child of "
      "this world.")
  .def("actor_by_name", &gz::sim::World::ActorByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a actor entity which is an immediate child of "
      "this world.")
  .def("model_by_name", &gz::sim::World::ModelByName,
      py::arg("ecm"),
      py::arg("name"),
      "Get the ID of a model entity which is an immediate child of "
      "this world.")
  .def("lights", &gz::sim::World::Lights,
      py::arg("ecm"),
      "Get all lights which are immediate children of this world.")
  .def("actors", &gz::sim::World::Actors,
      py::arg("ecm"),
      "Get all actors which are immediate children of this world.")
  .def("models", &gz::sim::World::Models,
      py::arg("ecm"),
      "Get all models which are immediate children of this world.")
  .def("light_count", &gz::sim::World::LightCount,
      py::arg("ecm"),
      "Get the number of lights which are immediate children of this "
      "world.")
  .def("actor_count", &gz::sim::World::ActorCount,
      py::arg("ecm"),
      "Get the number of actors which are immediate children of this "
      "world.")
  .def("model_count", &gz::sim::World::ModelCount,
      py::arg("ecm"),
      "Get the number of models which are immediate children of this "
      "world.")
  .def("__copy__",
      [](const gz::sim::World &self)
      {
        return gz::sim::World(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::World &self, pybind11::dict)
      {
        return gz::sim::World(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz
