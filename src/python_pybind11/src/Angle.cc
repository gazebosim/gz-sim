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

#include <gz/math/Angle.hh>
#include <pybind11/operators.h>

#include "Angle.hh"

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathAngle(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Angle;
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
    .def(py::init<const double>())
    .def(py::init<const Class>())
    .def("radian",
         py::overload_cast<>(&Class::Radian, py::const_),
         "Get the value from an angle in radians.")
    .def("set_radian",
         &Class::SetRadian,
         "Set the value from an angle in radians.")
    .def("degree",
         py::overload_cast<>(&Class::Degree, py::const_),
         "Get the value from an angle in degree.")
    .def("set_degree",
         &Class::SetDegree,
         "Set the value from an angle in degree.")
    .def("normalize",
         &Class::Normalize,
         "Normalize the vector length")
    .def("normalized",
         &Class::Normalized,
         "Return a normalized vector")
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(py::self * py::self)
    .def(py::self *= py::self)
    .def(py::self - py::self)
    .def(py::self -= py::self)
    .def(py::self / py::self)
    .def(py::self /= py::self)
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def(py::self < py::self)
    .def(py::self <= py::self)
    .def(py::self > py::self)
    .def(py::self >= py::self)
    .def_readonly_static("ZERO", &Class::Zero, "math::Angle(0)")
    .def_readonly_static("PI", &Class::Pi, "math::Angle(GZ_PI)")
    .def_readonly_static("HALF_PI", &Class::HalfPi, "math::Angle(GZ_PI * 0.5)")
    .def_readonly_static("TWO_PI", &Class::TwoPi, "math::Angle(GZ_PI * 2)")
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
