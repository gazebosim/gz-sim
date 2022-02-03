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

#ifndef IGNITION_MATH_PYTHON__LINE2_HH_
#define IGNITION_MATH_PYTHON__LINE2_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Line2.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Help define a pybind11 wrapper for an ignition::math::Line2
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathLine2(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Line2<T>;
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
    .def(py::init<const ignition::math::Vector2<T>&,
                  const ignition::math::Vector2<T>&>())
    .def(py::init<double, double, double, double>())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("set",
         py::overload_cast<const ignition::math::Vector2<T>&,
                           const ignition::math::Vector2<T>&>(&Class::Set),
         "Set the start and end point of the line segment")
    .def("set",
         py::overload_cast<double, double, double, double>(&Class::Set),
         "Set the start and end point of the line segment")
    .def("cross_product",
         py::overload_cast<const Class&>(&Class::CrossProduct, py::const_),
         "Return the cross product of this line and the given line.")
    .def("cross_product",
         py::overload_cast<const ignition::math::Vector2<T>&>(
           &Class::CrossProduct, py::const_),
         "Return the cross product of this line and the given line.")
    .def("collinear",
         py::overload_cast<const ignition::math::Vector2<T>&, double>(
         &Class::Collinear, py::const_),
         py::arg("_pt") = ignition::math::Vector2<T>::Zero,
         py::arg("_epsilon") = 1e-6,
         "Check if the given point is collinear with this line.")
    .def("collinear",
         py::overload_cast<const Class&, double>(&Class::Collinear, py::const_),
         py::arg("_pt") = Class(0, 0, 0, 0), py::arg("_epsilon") = 1e-6,
         "Check if the given point is collinear with this line.")
    .def("parallel",
         &Class::Parallel,
         py::arg("_line") = Class(0, 0, 0, 0), py::arg("_epsilon") = 1e-6,
         "Check if the given line is parallel with this line.")
    .def("on_segment",
         &Class::OnSegment,
         "Return whether the given point is on this line segment.")
    .def("within",
         &Class::Within,
         "Check if the given point is between the start and end "
         "points of the line segment. This does not imply that the point is "
         "on the segment.")
    .def("intersect",
         py::overload_cast<
           const Class&,
           ignition::math::Vector2<T>&,
           double>(
             &Class::Intersect, py::const_),
         py::arg("_line") = Class(0, 0, 0, 0),
         py::arg("_pt") = ignition::math::Vector2<T>::Zero,
         py::arg("_epsilon") = 1e-6,
         "Check if this line intersects the given line segment.")
    .def("intersect",
         py::overload_cast<const Class&, double>(&Class::Intersect, py::const_),
         py::arg("_line") = Class(0, 0, 0, 0), py::arg("_epsilon") = 1e-6,
         "Check if this line intersects the given line segment.")
    .def("length",
         &Class::Length,
         "Get the length of the line")
    .def("slope",
         &Class::Slope,
         "Get the slope of the line")
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

/// Define a pybind11 wrapper for an ignition::math::Line2
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathLine2(py::module &m, const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__LINE2_HH_
