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

#ifndef IGNITION_MATH_PYTHON__PLANE_HH_
#define IGNITION_MATH_PYTHON__PLANE_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <ignition/math/Plane.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Plane
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathPlane(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Plane<T>;
  std::string pyclass_name = typestr;
  py::class_<Class> plane(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr());
  plane.def(py::init<>())
    .def(py::init<const ignition::math::Vector3<T>&, T>(),
         py::arg("_normal") = ignition::math::Vector3<T>::Zero,
         py::arg("_offset") = 0.0)
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Vector2<T>&, T>())
    .def(py::init<const Class&>())
    .def("set",
         py::overload_cast<const ignition::math::Vector3<T>&, T>
             (&Class::Set),
         "Set the plane")
    .def("set",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           const ignition::math::Vector2<T>&, T>
             (&Class::Set),
         "Set the plane")
    .def("distance",
         py::overload_cast<const ignition::math::Vector3<T>&>
             (&Class::Distance, py::const_),
         "The distance to the plane from the given point. The "
         "distance can be negative, which indicates the point is on the "
         "negative side of the plane.")
    .def("distance",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           const ignition::math::Vector3<T>&>
             (&Class::Distance, py::const_),
         "Get distance to the plane give an origin and direction.")
    .def("intersection",
         &Class::Intersection,
         py::arg("_point") = ignition::math::Vector3<T>::Zero,
         py::arg("_gradient") = ignition::math::Vector3<T>::Zero,
         py::arg("_tolerance") = 1e-6,
         "Get the intersection of an infinite line with the plane, "
         "given the line's gradient and a point in parametrized space.")
    .def("side",
         py::overload_cast<const ignition::math::Vector3<T>&>
             (&Class::Side, py::const_),
         "The side of the plane a point is on.")
    .def("side",
         py::overload_cast<const ignition::math::AxisAlignedBox&>
             (&Class::Side, py::const_),
         "The side of the plane a point is on.")
    .def("size",
         py::overload_cast<>(&Class::Size),
         py::return_value_policy::reference,
         "Get the plane size")
    .def("normal",
         py::overload_cast<>(&Class::Normal),
         py::return_value_policy::reference,
         "Get the plane size")
    .def("offset",
         &Class::Offset,
         "Get the plane offset")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a);

    py::enum_<ignition::math::Planed::PlaneSide>(m, "PlaneSide")
      .value("NEGATIVE_SIDE", ignition::math::Planed::PlaneSide::NEGATIVE_SIDE)
      .value("POSITIVE_SIDE", ignition::math::Planed::PlaneSide::POSITIVE_SIDE)
      .value("NO_SIDE", ignition::math::Planed::PlaneSide::NO_SIDE)
      .value("BOTH_SIDE", ignition::math::Planed::PlaneSide::BOTH_SIDE)
      .export_values();
}

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__PLANE_HH_
