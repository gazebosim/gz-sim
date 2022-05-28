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
#include <string>
#include "RotationSpline.hh"
#include <gz/math/RotationSpline.hh>

namespace gz
{
namespace math
{
namespace python
{
void defineMathRotationSpline(py::module &m, const std::string &typestr)
{
  using Class = gz::math::RotationSpline;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("add_point",
       &Class::AddPoint,
       "Adds a control point to the end of the spline.")
   .def("point",
        &Class::Point,
        "Gets the detail of one of the control points of the spline.")
   .def("point_count",
        &Class::PointCount,
        "Gets the number of control points in the spline.")
   .def("clear",
        &Class::Clear,
        "Clears all the points in the spline.")
   .def("update_point",
        &Class::UpdatePoint,
        "Updates a single point in the spline.")
   .def("interpolate",
        py::overload_cast<double, const bool>(&Class::Interpolate),
        py::arg("_t") = 0, py::arg("_useShortestPath") = true,
        "Returns an interpolated point based on a parametric "
        "value over the whole series.")
   .def("interpolate",
        py::overload_cast<const unsigned int, const double, const bool>
          (&Class::Interpolate),
        py::arg("_fromIndex") = 0,
        py::arg("_t") = 0, py::arg("_useShortestPath") = true,
        "Interpolates a single segment of the spline "
        "given a parametric value.")
   .def("auto_calculate", &Class::AutoCalculate,
        "Tells the spline whether it should automatically calculate "
        "tangents on demand as points are added.")
   .def("recalc_tangents", &Class::RecalcTangents,
        "Recalculates the tangents associated with this spline.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
