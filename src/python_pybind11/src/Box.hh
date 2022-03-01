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
 *
*/

#ifndef IGNITION_MATH_PYTHON__BOX_HH_
#define IGNITION_MATH_PYTHON__BOX_HH_

#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <ignition/math/Box.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Box
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathBox(py::module &m, const std::string &typestr)
{

  using Class = ignition::math::Box<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const T, const T, const T>())
    .def(py::init<const T, const T, const T, const ignition::math::Material&>())
    .def(py::init<const T, const T, const T, const ignition::math::Material&>())
    .def(py::init<const ignition::math::Vector3<T>&>())
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Material&>())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("size",
         &Class::Size,
         "Get the size of the box.")
    .def("set_size",
         py::overload_cast<const ignition::math::Vector3<T>&>
          (&Class::SetSize),
         "Set the size of the box.")
    .def("set_size",
         py::overload_cast<const T, const T, const T>
          (&Class::SetSize),
         "Set the size of the box.")
    .def("material",
         &Class::Material,
         "Get the material associated with this box.")
    .def("set_material",
         &Class::SetMaterial,
         "Set the material associated with this box.")
    .def("volume",
         &Class::Volume,
         "Get the volume of the box in m^3.")
    .def("volume_below",
         &Class::VolumeBelow,
         "Get the volume of the box below a plane.")
    .def("center_of_volume_below",
         &Class::CenterOfVolumeBelow,
         "Center of volume below the plane. This is useful when "
         "calculating where buoyancy should be applied, for example.")
    .def("vertices_below",
         [](const Class &self, const Plane<T> &_plane)
         {
           std::vector<ignition::math::Vector3<T>> result;
           auto vertices = self.VerticesBelow(_plane);
           for (auto & v : vertices)
           {
             result.push_back(v);
           }
           return result;
         },
         "All the vertices which are on or below the plane.")
    .def("density_from_mass",
         &Class::DensityFromMass,
         "Compute the box's density given a mass value.")
    .def("set_density_from_mass",
         &Class::SetDensityFromMass,
         "Set the density of this box based on a mass value.")
    .def("mass_matrix",
         &Class::MassMatrix,
         "Get the mass matrix for this box. This function "
         "is only meaningful if the box's size and material "
         "have been set.")
    .def("intersections",
         [](const Class &self, const Plane<T> &_plane)
         {
           std::vector<ignition::math::Vector3<T>> result;
           auto vertices = self.Intersections(_plane);
           for (auto & v : vertices)
           {
             result.push_back(v);
           }
           return result;
         },
         "Get intersection between a plane and the box's edges. "
         "Edges contained on the plane are ignored.")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a);
}

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__BOX_HH_
