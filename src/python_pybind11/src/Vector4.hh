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

#ifndef IGNITION_MATH_PYTHON__VECTOR4_HH_
#define IGNITION_MATH_PYTHON__VECTOR4_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Vector4.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Help define a pybind11 wrapper for an ignition::math::Vector4
/**
 * \param[in] module a pybind11 module to add the definition to
 */
template<typename T>
void helpDefineMathVector4(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Vector4<T>;
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
    .def(py::init<const T&, const T&, const T&, const T&>())
    .def(py::init<const Class>())
    .def("sum", &Class::Sum, "Return the sum of the values")
    .def("distance",
         py::overload_cast<T, T, T, T>(&Class::Distance, py::const_),
         "Calc distance to the given point")
    .def("distance",
         py::overload_cast<const Class&>(&Class::Distance, py::const_),
         "Calc distance to the given point")
    .def("length",
         &Class::Length,
         "Returns the length (magnitude) of the vector")
    .def("squared_length",
         &Class::SquaredLength,
         "Return the square of the length (magnitude) of the vector")
    .def("normalize", &Class::Normalize, "Normalize the vector length")
    .def("normalized", &Class::Normalized, "Return a normalized vector")
    .def("round",
         &Class::Round,
         "Round to near whole number, return the result.")
    .def("rounded", &Class::Rounded, "Get a rounded version of this vector")
    .def("set", &Class::Set, "Set the contents of the vector")
    .def("dot",
         &Class::Dot,
         "Return the dot product of this vector and another vector")
    .def("abs_dot", &Class::AbsDot,
         "Return the absolute dot product of this vector and "
         "another vector. This is similar to the Dot function, except the "
         "absolute value of each component of the vector is used.")
    .def("abs", &Class::Abs, " Get the absolute value of the vector")
    .def("max",
         py::overload_cast<const Class&>(&Class::Max),
         "Set this vector's components to the maximum of itself and the "
         "passed in vector")
    .def("max", py::overload_cast<>(&Class::Max, py::const_),
         "Get the maximum value in the vector")
    .def("min", py::overload_cast<const Class&>(&Class::Min),
         "Set this vector's components to the minimum of itself and the "
         "passed in vector")
    .def("min", py::overload_cast<>(&Class::Min, py::const_),
         "Get the minimum value in the vector")
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(py::self + T())
    .def(py::self += T())
    .def(py::self * py::self)
    .def(py::self *= py::self)
    .def(py::self * T())
    .def(py::self *= T())
    .def(py::self - py::self)
    .def(py::self -= py::self)
    .def(py::self - T())
    .def(py::self -= T())
    .def(py::self / py::self)
    .def(py::self /= py::self)
    .def(py::self / T())
    .def(py::self /= T())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def(-py::self)
    .def("equal", &Class::Equal, "Equal to operator")
    .def("is_finite",
         &Class::IsFinite,
         "See if a point is finite (e.g., not nan)")
    .def("correct", &Class::Correct, "Corrects any nan values")
    .def("x", py::overload_cast<>(&Class::X), "Get the x value.")
    .def("y", py::overload_cast<>(&Class::Y), "Get the y value.")
    .def("z", py::overload_cast<>(&Class::Z), "Get the z value.")
    .def("w", py::overload_cast<>(&Class::W), "Get the w value.")
    .def("x", py::overload_cast<const T&>(&Class::X), "Set the x value.")
    .def("y", py::overload_cast<const T&>(&Class::Y), "Set the y value.")
    .def("z", py::overload_cast<const T&>(&Class::Z), "Set the z value.")
    .def("w", py::overload_cast<const T&>(&Class::W), "Set the w value.")
    .def_readonly_static("ZERO", &Class::Zero, "math::Vector4(0, 0, 0, 0)")
    .def_readonly_static("ONE", &Class::One, "math::Vector4(1, 1, 1, 1)")
    .def_readonly_static("NAN",
                         &Class::NaN,
                         "math::Vector4(NaN, NaN, NaN, NaN)")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a)
    .def("__getitem__",
         py::overload_cast<const std::size_t>(&Class::operator[], py::const_))
    .def("__setitem__",
         [](Class* vec, unsigned index, T val) { (*vec)[index] = val; })
    .def("__str__", toString)
    .def("__repr__", toString);
}

/// Define a pybind11 wrapper for an ignition::math::Vector4
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineMathVector4(py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__VECTOR4_HH_
