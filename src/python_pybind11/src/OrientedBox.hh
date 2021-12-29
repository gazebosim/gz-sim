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

#ifndef IGNITION_MATH_PYTHON__ORIENTEDBOX_HH_
#define IGNITION_MATH_PYTHON__ORIENTEDBOX_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/OrientedBox.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::OrientedBox
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathOrientedBox(py::module &m, const std::string &typestr)
{

  using Class = ignition::math::OrientedBox<T>;
  std::string pyclass_name = typestr;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Pose3<T>&>())
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Pose3<T>&,
                  const ignition::math::Material&>())
    .def(py::init<const ignition::math::Vector3<T>&>())
    .def(py::init<const Class&>())
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Material&>())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("pose",
         py::overload_cast<ignition::math::Pose3<T>&>(&Class::Pose),
         "Set the box pose, which is the pose of its center.")
    .def("pose",
         py::overload_cast<>(&Class::Pose, py::const_),
         py::return_value_policy::reference,
         "Get the box pose, which is the pose of its center.")
    .def("size",
         py::overload_cast<>(&Class::Size, py::const_),
         "Get the size of the OrientedBox.")
    .def("size",
         py::overload_cast<ignition::math::Vector3<T>&>
          (&Class::Size),
         "Set the size of the OrientedBox.")
    .def("x_length",
         &Class::XLength,
         "Get the length along the x dimension")
    .def("y_length",
         &Class::YLength,
         "Get the length along the y dimension")
    .def("z_length",
         &Class::ZLength,
         "Get the length along the z dimension")
    .def("material",
         &Class::Material,
         "Get the material associated with this OrientedBox.")
    .def("set_material",
         &Class::SetMaterial,
         "Set the material associated with this OrientedBox.")
    .def("volume",
         &Class::Volume,
         "Get the volume of the OrientedBox in m^3.")
    .def("contains",
         &Class::Contains,
         "Check if a point lies inside the box.")
    .def("density_from_mass",
         &Class::DensityFromMass,
         "Compute the OrientedBox's density given a mass value.")
    .def("set_density_from_mass",
         &Class::SetDensityFromMass,
         "Set the density of this OrientedBox based on a mass value.")
    .def("mass_matrix",
         &Class::MassMatrix,
         "Get the mass matrix for this OrientedBox. This function "
         "is only meaningful if the OrientedBox's size and material "
         "have been set.")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a)
    .def("__str__", toString)
    .def("__repr__", toString);
}

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__ORIENTEDBOX_HH_
