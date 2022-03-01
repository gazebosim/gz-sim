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

#ifndef IGNITION_MATH_PYTHON__MOVINGWINDOWFILTER_HH_
#define IGNITION_MATH_PYTHON__MOVINGWINDOWFILTER_HH_

#include <string>

#include <pybind11/pybind11.h>

#include <ignition/math/MovingWindowFilter.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Help define a pybind11 wrapper for an ignition::math::MovingWindowFilter
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathMovingWindowFilter(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::MovingWindowFilter<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def("update", &Class::Update, "Update value of filter")
    .def("set_window_size", &Class::SetWindowSize, "Set window size")
    .def("window_size", &Class::WindowSize, "Get the window size.")
    .def("window_filled",
         &Class::WindowFilled,
         "Get whether the window has been filled.")
    .def("value", &Class::Value, "Get filtered result");
}

/// Define a pybind11 wrapper for an ignition::math::MovingWindowFilter
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathMovingWindowFilter(py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__MovingWindowFilterD_HH_
