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

#include <sstream>
#include <string>

#include <pybind11/operators.h>

#include "AxisAlignedBox.hh"

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Vector3.hh>

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathAxisAlignedBox(py::module &m, const std::string &typestr)
{
  using Class = gz::math::AxisAlignedBox;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  py::class_<Class>(
    m,
    typestr.c_str(),
    py::buffer_protocol(),
    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const Class&>())
    .def(py::init<double, double, double,
                  double, double, double>())
    .def(py::init<const gz::math::Vector3d&,
                  const gz::math::Vector3d>())
    .def("x_length",
         &Class::XLength,
         "Get the length along the x dimension")
    .def("y_length",
         &Class::YLength,
         "Get the length along the y dimension")
    .def("z_length",
         &Class::ZLength,
         "Get the length along the z dimension")
    .def("size",
         &Class::Size,
         "Get the size of the box")
    .def("center",
         &Class::Center,
         "Get the box center")
    .def("merge",
         &Class::Merge,
         "Merge a box with this box")
    .def("volume",
         &Class::Volume,
         "Get the volume of the box in m^3.")
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(py::self + gz::math::Vector3d())
    .def(py::self - gz::math::Vector3d())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def("min",
         py::overload_cast<>(&Class::Min),
         py::return_value_policy::reference_internal,
         "Get the minimum corner.")
    .def("max",
         py::overload_cast<>(&Class::Max),
         py::return_value_policy::reference_internal,
         "Get the maximum corner.")
    .def("intersects",
         &Class::Intersects,
         "Test box intersection. This test will only work if "
         " both box's minimum corner is less than or equal to their "
         " maximum corner.")
    .def("contains",
         &Class::Contains,
         "Check if a point lies inside the box.")
    .def("intersect_check",
         &Class::IntersectCheck,
         "Check if a ray (origin, direction) intersects the box.")
    .def("intersect_dist",
         &Class::IntersectDist,
         "Check if a ray (origin, direction) intersects the box.")
    .def("intersect",
         py::overload_cast<const Vector3d &, const Vector3d &,
                           const double, const double>
          (&Class::Intersect, py::const_),
         "Check if a ray (origin, direction) intersects the box.")
    .def("intersect",
         py::overload_cast<const Line3d&>(&Class::Intersect, py::const_),
         "Check if a ray (origin, direction) intersects the box.")
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
}  // namespace gz
