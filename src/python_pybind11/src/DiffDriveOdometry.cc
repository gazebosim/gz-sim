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
#include "DiffDriveOdometry.hh"

namespace gz
{
namespace math
{
namespace python
{
void defineMathDiffDriveOdometry(
  py::module &m, const std::string &typestr)
{
  using Class = gz::math::DiffDriveOdometry;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<size_t>())
    .def(py::init<>())
    .def("init", &Class::Init, "Initialize the odometry")
    .def("initialized", &Class::Initialized,
         "Get whether Init has been called.")
    .def("update",
         &Class::Update,
         "Updates the odometry class with latest wheels and "
         "steerings position")
    .def("heading", &Class::Heading, "Get the heading.")
    .def("x", &Class::X, "Get the X position.")
    .def("y", &Class::Y, "Get the Y position.")
    .def("linear_velocity",
         &Class::LinearVelocity,
         "Get the linear velocity.")
    .def("angular_velocity",
         &Class::AngularVelocity,
         "Get the angular velocity.")
    .def("set_wheel_params",
         &Class::SetWheelParams,
         "Set the wheel parameters including the radius and separation.")
    .def("set_velocity_rolling_window_size",
         &Class::SetVelocityRollingWindowSize,
         "Set the velocity rolling window size.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
