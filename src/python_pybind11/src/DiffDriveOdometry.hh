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
#ifndef IGNITION_MATH_PYTHON__DIFFDRIVEODOMETRY_HH_
#define IGNITION_MATH_PYTHON__DIFFDRIVEODOMETRY_HH_

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/chrono.h>
#include <string>

#include <ignition/math/DiffDriveOdometry.hh>

namespace py = pybind11;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a py:: wrapper for an ignition::gazebo::DiffDriveOdometry
/**
 * \param[in] module a py:: module to add the definition to
 */
void defineMathDiffDriveOdometry(
  py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__DIFFDRIVEODOMETRY_HH_
