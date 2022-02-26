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

#ifndef IGNITION_MATH_PYTHON__LINE3_HH_
#define IGNITION_MATH_PYTHON__LINE3_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Line3.hh>
#include <ignition/math/Vector3.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Help define a pybind11 wrapper for an ignition::math::Line3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathLine3(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Line3<T>;
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
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Vector3<T>&>(),
         "Constructor")
    .def(py::init<const double, const double, const double, const double>(),
         "2D Constructor where Z coordinates are 0")
    .def(py::init<const double, const double, const double, const double,
                  const double, const double>(),
         "Constructor")
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("set",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           const ignition::math::Vector3<T>&>(&Class::Set),
         "Set the start and end point of the line segment")
    .def("set_a",
         &Class::SetA,
         "Set the start point of the line segment")
    .def("set_b",
         &Class::SetB,
         "Set the end point of the line segment")
    .def("set",
         py::overload_cast<const double, const double, const double,
                           const double, const double>(&Class::Set),
         py::arg("_x1"), py::arg("_y1"), py::arg("_x2"),
         py::arg("_y2"), py::arg("_z") = 0,
         "Set the start and end point of the line segment, assuming that "
         "both points have the same height.")
    .def("set",
         py::overload_cast<const double, const double, const double,
                           const double, const double, const double>(
                             &Class::Set),
         "Set the start and end point of the line segment")
    .def("direction",
         &Class::Direction,
         "Get the direction of the line")
    .def("length",
         &Class::Length,
         "Get the length of the line")
    .def("distance",
         py::overload_cast<const Class&, Class&, const double>(
           &Class::Distance, py::const_),
         py::arg("_line"), py::arg("_result"), py::arg("_epsilon") = 1e-6,
         "Get the shortest line between this line and the provided line.")
    .def("distance",
         py::overload_cast<const ignition::math::Vector3<T>&>(
           &Class::Distance),
         "Calculate shortest distance between line and point")
    .def("intersect",
         py::overload_cast<
           const Class&,
           ignition::math::Vector3<T>&,
           double>(
             &Class::Intersect, py::const_),
         py::arg("_line") = Class(0, 0, 0, 0),
         py::arg("_pt") = ignition::math::Vector3<T>::Zero,
         py::arg("_epsilon") = 1e-6,
         "Check if this line intersects the given line segment.")
    .def("intersect",
         py::overload_cast<const Class&, double>(&Class::Intersect, py::const_),
         py::arg("_line") = Class(), py::arg("_epsilon") = 1e-6,
         "Check if this line intersects the given line segment.")
    .def("coplanar",
         &Class::Coplanar,
         py::arg("_line") = Class(), py::arg("_epsilon") = 1e-6,
         "Check if the given point is collinear with this line.")
    .def("parallel",
         &Class::Parallel,
         py::arg("_line") = Class(), py::arg("_epsilon") = 1e-6,
         "Check if the given line is parallel with this line.")
    .def("within",
         &Class::Within,
         py::arg("_pt") = ignition::math::Vector3<T>::Zero,
         py::arg("_epsilon") = 1e-6,
         "Check if the given point is between the start and end "
         "points of the line segment.")
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

/// Define a pybind11 wrapper for an ignition::math::Line3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathLine3(py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__LINE3_HH_
