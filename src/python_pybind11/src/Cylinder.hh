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

#ifndef GZ_MATH_PYTHON__CYLINDER_HH_
#define GZ_MATH_PYTHON__CYLINDER_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <gz/math/Cylinder.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an gz::math::Cylinder
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathCylinder(py::module &m, const std::string &typestr)
{

  using Class = gz::math::Cylinder<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const T, const T,
                  const gz::math::Quaternion<T>&>(),
         py::arg("_length") = 0,
         py::arg("_radius") = 0,
         py::arg("_rotOffset") = gz::math::Quaternion<T>::Identity)
    .def(py::init<const T, const T,
                  const gz::math::Material&,
                  const gz::math::Quaternion<T>&>(),
         py::arg("_length") = 0,
         py::arg("_radius") = 0,
         py::arg("_material") = gz::math::Material(),
         py::arg("_rotOffset") = gz::math::Quaternion<T>::Identity)
    .def(py::self == py::self)
    .def("radius",
         &Class::Radius,
         "Get the radius in meters.")
    .def("set_radius",
         &Class::SetRadius,
         "Set the radius in meters.")
    .def("length",
         &Class::Length,
         "Get the length in meters.")
    .def("set_length",
         &Class::SetLength,
         "Set the length in meters.")
    .def("rotational_offset",
         &Class::RotationalOffset,
         "Get the rotation offset.")
    .def("set_rotational_offset",
         &Class::SetRotationalOffset,
         "Set the rotation offset.")
    .def("mat",
         &Class::Mat,
         "Get the material associated with this box.")
    .def("set_mat",
         &Class::SetMat,
         "Set the material associated with this box.")
    .def("volume",
         &Class::Volume,
         "Get the volume of the box in m^3.")
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
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a);
}

}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__CYLINDER_HH_
