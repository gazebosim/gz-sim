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

#ifndef IGNITION_MATH_PYTHON__MATRIX3_HH_
#define IGNITION_MATH_PYTHON__MATRIX3_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Matrix3.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Matrix3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathMatrix3(py::module &m, const std::string &typestr);

/// Help define a pybind11 wrapper for an ignition::math::Matrix3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathMatrix3(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Matrix3<T>;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<Class>())
    .def(py::init<T, T, T, T, T, T, T, T, T>())
    .def(py::init<const ignition::math::Quaternion<T>&>())
    .def(py::self - py::self)
    .def(py::self + py::self)
    .def(py::self * py::self)
    .def(py::self * float())
    .def(py::self * ignition::math::Vector3<T>())
    // .def(py::self * py::self)
    // .def(py::self += py::self)
    // .def(-py::self)
    // .def(py::self -= py::self)
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def("__call__",
         py::overload_cast<size_t, size_t>(&Class::operator()),
         py::return_value_policy::reference)
    // .def(py::self *= py::self)
    .def("set",
         &Class::Set,
         "Set values")
    .def("axes",
         &Class::Axes,
         "Set the matrix from three axis (1 per column)")
    .def("axis",
        &Class::Axis,
        "Set the matrix from an axis and angle")
    .def("from_2_axes",
         &Class::From2Axes,
         "Set the matrix to represent rotation from "
         "vector _v1 to vector _v2, so that")
    .def("col",
         &Class::Col,
         "Set a column.")
    .def("equal",
         &Class::Equal,
         "Equality test operator")
    .def("determinant",
         &Class::Determinant,
         "Return the determinant of the matrix")
    .def("inverse",
         &Class::Inverse,
         "Return the inverse matrix")
    .def("transpose",
         &Class::Transpose,
         "Transpose this matrix.")
    .def("transposed",
         &Class::Transposed,
         "Return the transpose of this matrix")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a)
    .def_readonly_static("IDENTITY", &Class::Identity, "Identity matrix")
    .def_readonly_static("ZERO", &Class::Zero, "Zero matrix")
    .def("__str__", toString)
    .def("__repr__", toString);
}
}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__MATRIX3_HH_
