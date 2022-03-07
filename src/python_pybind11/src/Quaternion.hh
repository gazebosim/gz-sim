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

#ifndef IGNITION_MATH_PYTHON__QUATERNION_HH_
#define IGNITION_MATH_PYTHON__QUATERNION_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Quaternion
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathQuaternion(py::module &m, const std::string &typestr);

/// Help define a pybind11 wrapper for an ignition::math::Quaternion
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathQuaternion(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Quaternion<T>;
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
    .def(py::init<T, T, T, T>())
    .def(py::init<T, T, T>())
    .def(py::init<const ignition::math::Vector3<T>&, T>())
    .def(py::init<const ignition::math::Vector3<T>&>())
    .def(py::init<const ignition::math::Matrix3<T>&>())
    .def(py::init<const Class&>())
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(-py::self)
    .def(py::self - py::self)
    .def(py::self -= py::self)
    .def(py::self * py::self)
    .def(py::self * float())
    .def(py::self * ignition::math::Vector3<T>())
    .def(py::self *= py::self)
    .def(py::self == py::self)
    .def(py::self != py::self)
    // .def(py::self * py::self)
    // .def(py::self *= py::self)
    .def("invert",
         &Class::Invert,
         "Invert the quaternion")
    .def("inverse",
         &Class::Inverse,
         "Get the inverse of this quaternion")
    .def("log",
         &Class::Log,
         "Return the logarithm")
    .def("exp",
        &Class::Exp,
        "Return the exponent")
    .def("normalize",
         &Class::Normalize,
         "Normalize the quaternion")
    .def("normalized",
         &Class::Normalized,
         "Gets a normalized version of this quaternion")
    .def("axis",
         py::overload_cast<T, T, T, T>(&Class::Axis),
         "Set the quaternion from an axis and angle")
    .def("axis",
         py::overload_cast<const ignition::math::Vector3<T>&, T>(&Class::Axis),
         "Set the quaternion from an axis and angle")
    .def("set",
         &Class::Set,
         "Set this quaternion from 4 floating numbers")
    .def("euler",
         py::overload_cast<const ignition::math::Vector3<T>&>(&Class::Euler),
         "Set the quaternion from Euler angles. The order of operations "
         "is roll, pitch, yaw around a fixed body frame axis "
         "(the original frame of the object before rotation is applied).")
    .def("euler",
         py::overload_cast<T, T, T>(&Class::Euler),
         "Set the quaternion from Euler angles.")
    .def("euler",
         py::overload_cast<>(&Class::Euler, py::const_),
         "Return the rotation in Euler angles")
    .def("euler_to_quaternion",
         py::overload_cast<const ignition::math::Vector3<T>&>(
           &Class::EulerToQuaternion),
         "Convert euler angles to quatern.")
    .def("euler_to_quaternion",
         py::overload_cast<T, T, T>(&Class::EulerToQuaternion),
         "Convert euler angles to quatern.")
    .def("roll", &Class::Roll, "Get the Euler roll angle in radians")
    .def("pitch", &Class::Pitch, "Get the Euler pitch angle in radians")
    .def("yaw", &Class::Yaw, "Get the Euler yaw angle in radians")
    .def("to_axis",
         [](const Class &self) {
            ignition::math::Vector3<T> _axis;
            T _angle;
            self.ToAxis(_axis, _angle);
            return std::make_tuple(_axis, _angle);
         },
         "Return rotation as axis and angle")
    .def("matrix",
         &Class::Matrix,
         "Set from a rotation matrix.")
    .def("from_2_axes",
         &Class::From2Axes,
         "Set this quaternion to represent rotation from "
         "vector _v1 to vector _v2, so that")
    .def("scale",
         &Class::Scale,
         "Scale a Quaternion<T>")
    .def("equal",
         &Class::Equal,
         "Equality test with tolerance.")
    .def("rotate_vector",
         &Class::RotateVector,
         "Rotate a vector using the quaternion")
    .def("rotate_vector_reverse",
         &Class::RotateVectorReverse,
         "Do the reverse rotation of a vector by this quaternion")
    .def("is_finite",
         &Class::IsFinite,
         "See if a quaternion is finite (e.g., not nan)")
    .def("correct",
         &Class::Correct,
         "Correct any nan values in this quaternion")
    .def("x_axis",
         &Class::XAxis,
         "Return the X axis")
    .def("y_axis",
         &Class::YAxis,
         "Return the Y axis")
    .def("z_axis",
         &Class::ZAxis,
         "Return the Z axis")
    .def("round",
         &Class::Round,
         "Round all values to _precision decimal places")
    .def("dot",
         &Class::Dot,
         "Dot product")
    .def("squad",
         &Class::Squad,
         "Spherical quadratic interpolation "
         "given the ends and an interpolation parameter between 0 and 1")
    .def("slerp",
         &Class::Slerp,
         "Spherical linear interpolation between 2 quaternions, "
         " given the ends and an interpolation parameter between 0 and 1")
    .def("integrate",
         &Class::Integrate,
         "Integrate quaternion for constant angular velocity vector "
         "along specified interval `_deltaT`.")
    .def("x", py::overload_cast<>(&Class::X), "Get the x value.")
    .def("y", py::overload_cast<>(&Class::Y), "Get the y value.")
    .def("z", py::overload_cast<>(&Class::Z), "Get the z value.")
    .def("w", py::overload_cast<>(&Class::W), "Get the w value.")
    .def("x", py::overload_cast<T>(&Class::X), "Set the x value.")
    .def("y", py::overload_cast<T>(&Class::Y), "Set the y value.")
    .def("z", py::overload_cast<T>(&Class::Z), "Set the z value.")
    .def("w", py::overload_cast<T>(&Class::W), "Set the w value.")
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

#endif  // IGNITION_MATH_PYTHON__QUATERNION_HH_
