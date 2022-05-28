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
#include "Spline.hh"
#include <gz/math/Spline.hh>

namespace gz
{
namespace math
{
namespace python
{
void defineMathSpline(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Spline;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("tension",
       py::overload_cast<double>(&Class::Tension),
       "Sets the tension parameter.")
  .def("tension",
       py::overload_cast<>(&Class::Tension, py::const_),
       "Gets the tension value.")
  .def("arc_length",
       py::overload_cast<const double>(&Class::ArcLength, py::const_),
       "Sets the tension parameter.")
  .def("arc_length",
       py::overload_cast<>(&Class::ArcLength, py::const_),
       "Gets spline arc length up to")
  .def("arc_length",
       py::overload_cast<const unsigned int,
                         const double>(&Class::ArcLength, py::const_),
       "Sets the tension parameter.")
  .def("add_point",
       py::overload_cast<const Vector3d&>(&Class::AddPoint),
       "Adds a single control point to the "
       "end of the spline.")
  .def("add_point",
       py::overload_cast<const Vector3d&, const Vector3d&>(&Class::AddPoint),
       "Adds a single control point to the end "
       " of the spline with fixed tangent.")
  .def("point",
       &Class::Point,
       "Gets the value for one of the control points "
       " of the spline.")
  .def("tangent",
       &Class::Tangent,
       "Gets the tangent value for one of the control points "
       " of the spline.")
  .def("MthDerivative",
       &Class::MthDerivative,
       "Gets the mth derivative for one of the control points "
       " of the spline.")
  .def("mth_derivative",
       &Class::MthDerivative,
       "Gets the mth derivative for one of the control points "
       " of the spline.")
  .def("point_count",
       &Class::PointCount,
       "Gets the number of control points in the spline.")
  .def("clear",
       &Class::Clear,
       "Clears all the points in the spline.")
  .def("update_point",
       py::overload_cast<const unsigned int,
                         const Vector3d&>(&Class::UpdatePoint),
       "Updates a single control point value in the spline, "
       " keeping its tangent.")
  .def("update_point",
       py::overload_cast<const unsigned int,
                         const Vector3d&,
                         const Vector3d&>(&Class::UpdatePoint),
       "Updates a single control point in the spline, along "
       " with its tangent.")
  .def("interpolate",
       py::overload_cast<const double>(&Class::Interpolate, py::const_),
       "Interpolates a point on the spline "
       "at parameter value p _t.")
  .def("interpolate",
       py::overload_cast<const unsigned int,
                         const double>(&Class::Interpolate, py::const_),
       "Interpolates a point on the spline "
       "at parameter value p _t.")
  .def("interpolate_tangent",
       py::overload_cast<const double>
           (&Class::InterpolateTangent, py::const_),
       "Interpolates a tangent on the spline "
       "at parameter value p _t.")
  .def("interpolate_tangent",
       py::overload_cast<const unsigned int,
                         const double>(
           &Class::InterpolateTangent, py::const_),
       "Interpolates a tangent on the spline "
       "at parameter value p _t.")
  .def("interpolate_mth_derivative",
       py::overload_cast<const unsigned int,
                         const double>(
           &Class::InterpolateMthDerivative, py::const_),
       "Interpolates the mth derivative on the spline "
       "at parameter value p _t.")
  .def("interpolate_mth_derivative",
       py::overload_cast<const unsigned int,
                         const unsigned int,
                         const double>(
           &Class::InterpolateMthDerivative, py::const_),
       "Interpolates the mth derivative on the spline "
       "at parameter value p _t.")
  .def("auto_calculate", &Class::AutoCalculate,
       "Tells the spline whether it should automatically "
       "calculate tangents on demand as points are added.")
  .def("recalc_tangents", &Class::RecalcTangents,
       "Recalculates the tangents associated with this spline.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
