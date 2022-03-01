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
#include <pybind11/operators.h>

#include "SphericalCoordinates.hh"
#include <ignition/math/Angle.hh>
#include <ignition/math/SphericalCoordinates.hh>

namespace ignition
{
namespace math
{
namespace python
{
void defineMathSphericalCoordinates(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::SphericalCoordinates;
  std::string pyclass_name = typestr;

  py::class_<Class> sphericalCoordinates(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr());
  sphericalCoordinates
    .def(py::init<>())
    .def(py::init<const Class&>())
    .def(py::init<const Class::SurfaceType>())
    .def(py::init<const Class::SurfaceType, const ignition::math::Angle &,
                  const ignition::math::Angle &, const double,
                  const ignition::math::Angle &>())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def("spherical_from_local_position",
         &Class::SphericalFromLocalPosition,
         "Convert a Cartesian position vector to geodetic coordinates.")
    .def("global_from_local_velocity",
         &Class::GlobalFromLocalVelocity,
         "Convert a Cartesian velocity vector in the local frame "
         " to a global Cartesian frame with components East, North, Up")
    .def("convert",
         py::overload_cast<const std::string &>(&Class::Convert),
         "Convert a string to a SurfaceType.")
    .def("convert",
         py::overload_cast<Class::SurfaceType>(&Class::Convert),
         "Convert a SurfaceType to a string.")
    .def("distance",
         &Class::Distance,
         "Get the distance between two points expressed in geographic "
         "latitude and longitude. It assumes that both points are at sea level."
         " Example: _latA = 38.0016667 and _lonA = -123.0016667) represents "
         "the point with latitude 38d 0'6.00\"N and longitude 123d 0'6.00\"W.")
    .def("surface",
         &Class::Surface,
         "Get SurfaceType currently in use.")
    .def("latitude_reference",
         &Class::LatitudeReference,
         "Get reference geodetic latitude.")
    .def("longitude_reference",
         &Class::LongitudeReference,
         "Get reference longitude.")
    .def("elevation_reference",
         &Class::ElevationReference,
         "Get reference elevation in meters.")
    .def("heading_offset",
         &Class::HeadingOffset,
         "Get heading offset for the reference frame, expressed as "
         "angle from East to x-axis, or equivalently "
         "from North to y-axis.")
    .def("set_surface",
         &Class::SetSurface,
         "Set SurfaceType for planetary surface model.")
    .def("set_latitude_reference",
         &Class::SetLatitudeReference,
         "Set reference geodetic latitude.")
    .def("set_longitude_reference",
         &Class::SetLongitudeReference,
         "Set reference longitude.")
    .def("set_elevation_reference",
         &Class::SetElevationReference,
         "Set reference elevation above sea level in meters.")
    .def("set_heading_offset",
         &Class::SetHeadingOffset,
         "Set heading angle offset for the frame.")
    .def("local_from_spherical_position",
         &Class::LocalFromSphericalPosition,
         "Convert a geodetic position vector to Cartesian coordinates.")
    .def("local_from_global_velocity",
         &Class::LocalFromGlobalVelocity,
         "Convert a Cartesian velocity vector with components East, "
         "North, Up to a local cartesian frame vector XYZ.")
    .def("update_transformation_matrix",
         &Class::UpdateTransformationMatrix,
         "Update coordinate transformation matrix with reference location")
    .def("position_transform",
         &Class::PositionTransform,
         "Convert between velocity in SPHERICAL/ECEF/LOCAL/GLOBAL frame "
         "Spherical coordinates use radians, while the other frames use "
         "meters.")
    .def("velocity_transform",
         &Class::VelocityTransform,
         "Convert between velocity in SPHERICAL/ECEF/LOCAL/GLOBAL frame "
         "Spherical coordinates use radians, while the other frames use "
         "meters.");

   py::enum_<Class::CoordinateType>(sphericalCoordinates, "CoordinateType")
       .value("SPHERICAL", Class::CoordinateType::SPHERICAL)
       .value("ECEF", Class::CoordinateType::ECEF)
       .value("GLOBAL", Class::CoordinateType::GLOBAL)
       .value("LOCAL", Class::CoordinateType::LOCAL)
       .value("LOCAL2", Class::CoordinateType::LOCAL2)
       .export_values();
   py::enum_<Class::SurfaceType>(sphericalCoordinates, "SurfaceType")
       .value("EARTH_WGS84", Class::SurfaceType::EARTH_WGS84)
       .export_values();
}
}  // namespace python
}  // namespace math
}  // namespace ignition
