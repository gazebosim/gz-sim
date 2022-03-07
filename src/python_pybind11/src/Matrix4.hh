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

#ifndef IGNITION_MATH_PYTHON__MATRIX4_HH_
#define IGNITION_MATH_PYTHON__MATRIX4_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Matrix4.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Matrix4
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathMatrix4(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::Matrix4
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathMatrix4(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Matrix4<T>;
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
    .def(py::init<const Class&>())
    .def(py::init<T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T>())
    .def(py::init<const ignition::math::Quaternion<T>&>())
    .def(py::init<const ignition::math::Pose3<T>&>())
    .def(py::self * py::self)
    .def(py::self * ignition::math::Vector3<T>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def("__call__",
         py::overload_cast<size_t, size_t>(&Class::operator()),
         py::return_value_policy::reference)
    .def("set",
         &Class::Set,
         "Set values")
    .def("set_translation",
         py::overload_cast<T, T, T>(&Class::SetTranslation),
         "Set the translational values [ (0, 3) (1, 3) (2, 3) ]")
    .def("set_translation",
         py::overload_cast<const ignition::math::Vector3<T>&>(
            &Class::SetTranslation),
         "Set the translational values [ (0, 3) (1, 3) (2, 3) ]")
    .def("translation",
        &Class::Translation,
        "Get the translational values as a Vector3")
    .def("scale",
         py::overload_cast<>(&Class::Scale, py::const_),
         "Get the scale values as a Vector3<T>")
    .def("rotation",
         &Class::Rotation,
         "Get the rotation as a quaternion")
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
    .def("euler_rotation",
         &Class::EulerRotation,
         "Get the rotation as a Euler angles")
    .def("pose",
         &Class::Pose,
         "Get the transformation as math::Pose")
    .def("scale",
         py::overload_cast<const ignition::math::Vector3<T>&>(
           &Class::Scale),
         "Get the scale values as a Vector3<T>")
    .def("scale",
         py::overload_cast<T, T, T>(&Class::Scale),
         "Get the scale values as a Vector3<T>")
    .def("is_affine",
        &Class::IsAffine,
         "Get the scale values as a Vector3<T>")
    .def("transform_affine",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           ignition::math::Vector3<T>&>(
                             &Class::TransformAffine, py::const_),
         "Perform an affine transformation")
    .def("equal",
         &Class::Equal,
         "Equality test operator")
    .def("look_at",
         &Class::LookAt,
         // py::arg("_eye") = ignition::math::Vector3<T>::Zero,
         py::arg("_target") = ignition::math::Vector3<T>::Zero,
         py::arg("_up") = ignition::math::Vector3<T>::UnitZ,
         "Get transform which translates to _eye and rotates the X axis "
         "so it faces the _target. The rotation is such that Z axis is in the"
         "_up direction, if possible. The coordinate system is right-handed")
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
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__MATRIX4_HH_
