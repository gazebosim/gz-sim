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

#include "Frustum.hh"
#include <gz/math/Frustum.hh>

#include <pybind11/operators.h>
#include <pybind11/stl_bind.h>

namespace gz
{
namespace math
{
namespace python
{
void defineMathFrustum(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Frustum;
  std::string pyclass_name = typestr;
  py::class_<Class> (m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
   .def(py::init<>())
   .def(py::init<const double, const double, const gz::math::Angle&,
                 const double, const gz::math::Pose3d&>(),
        py::arg("_near") = 0,
        py::arg("_far") = 0,
        py::arg("_fov") = gz::math::Angle(0),
        py::arg("_aspectRatio") = 0,
        py::arg("_pose") = gz::math::Pose3d::Zero)
   .def(py::init<const Class&>())
   .def("near",
        &Class::Near,
        "Get the near distance. This is the distance from the "
        "frustum's vertex to the closest plane.")
   .def("set_near",
        &Class::SetNear,
        "Set the near distance. This is the distance from the "
        "frustum's vertex to the closest plane.")
   .def("far",
        &Class::Far,
        "Get the far distance. This is the distance from the "
        "frustum's vertex to the farthest plane.")
   .def("set_far",
        &Class::SetFar,
        "Set the far distance. This is the distance from the "
        "frustum's vertex to the farthest plane.")
   .def("fov",
        &Class::FOV,
        "Get the horizontal field of view. The field of view is the "
        "angle between the frustum's vertex and the edges of the near or far")
   .def("set_fov",
        &Class::SetFOV,
        "Set the horizontal field of view. The field of view is the "
        "angle between the frustum's vertex and the edges of the near or far")
   .def("aspect_ratio",
        &Class::AspectRatio,
        "Get the aspect ratio, which is the width divided by height "
        "of the near or far planes.")
   .def("set_aspect_ratio",
        &Class::SetAspectRatio,
        "Get the aspect ratio, which is the width divided by height "
        "of the near or far planes.")
   .def("pose",
        &Class::Pose,
        "Get the pose of the frustum")
   .def("set_pose",
        &Class::SetPose,
        "Set the pose of the frustum")
   .def("contains",
        py::overload_cast<const gz::math::AxisAlignedBox&>
            (&Class::Contains, py::const_),
        "Check if a box lies inside the pyramid frustum.")
   .def("contains",
        py::overload_cast<const gz::math::Vector3d&>
            (&Class::Contains, py::const_),
        "Check if a point lies inside the pyramid frustum.")
   .def("plane",
        &Class::Plane,
        "Get a plane of the frustum.");

   py::enum_<gz::math::Frustum::FrustumPlane>(m, "FrustumPlane")
       .value("FRUSTUM_PLANE_NEAR",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_NEAR)
       .value("FRUSTUM_PLANE_FAR",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_FAR)
       .value("FRUSTUM_PLANE_LEFT",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_LEFT)
       .value("FRUSTUM_PLANE_RIGHT",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_RIGHT)
       .value("FRUSTUM_PLANE_TOP",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_TOP)
       .value("FRUSTUM_PLANE_BOTTOM",
              gz::math::Frustum::FrustumPlane::FRUSTUM_PLANE_BOTTOM)
       .export_values();
}
}  // namespace python
}  // namespace math
}  // namespace gz
