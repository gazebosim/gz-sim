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

#ifndef IGNITION_MATH_PYTHON__SPHERE_HH_
#define IGNITION_MATH_PYTHON__SPHERE_HH_

#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Sphere.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Sphere
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathSphere(py::module &m, const std::string &typestr)
{

  using Class = ignition::math::Sphere<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const T>())
    .def(py::init<const T, const ignition::math::Material&>())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("radius",
         &Class::Radius,
         "Get the radius in meters.")
    .def("set_radius",
         &Class::SetRadius,
         "Set the radius in meters.")
    .def("material",
         &Class::Material,
         "Get the material associated with this sphere.")
    .def("set_material",
         &Class::SetMaterial,
         "Set the material associated with this sphere.")
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
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__SPHERE_HH_
