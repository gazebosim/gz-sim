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

#include <pybind11/chrono.h>
#include <pybind11/operators.h>
#include <string>

#include "StopWatch.hh"

#include <gz/math/Stopwatch.hh>

namespace gz
{
namespace math
{
namespace python
{
void defineMathStopwatch(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Stopwatch;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::self != py::self)
  .def(py::self == py::self)
  .def("start",
        &Class::Start, py::arg("_reset") = false,
        "Start the stopwatch.")
   .def("start_time",
        &Class::StartTime,
        "Get the time when the stopwatch was started.")
   .def("stop",
        &Class::Stop,
        "Stop the stopwatch")
   .def("stop_time",
        &Class::StopTime,
        "Get the time when the stopwatch was last stopped.")
   .def("running",
        &Class::Running,
        "Get whether the stopwatch is running.")
   .def("reset",
        &Class::Reset,
        "Reset the stopwatch. This resets the start time, stop time "
        "elapsed duration and elapsed stop duration.")
   .def("elapsed_run_time",
        &Class::ElapsedRunTime,
        "Get the amount of time that the stop watch has been "
        "running. This is the total amount of run time, spannning all start "
        "and stop calls. The Reset function or passing true to the Start "
        "function will reset this value.")
   .def("elapsed_stop_time",
        &Class::ElapsedStopTime,
        "Get the amount of time that the stop watch has been "
        "stopped. This is the total amount of stop time, spannning all start "
        "and stop calls. The Reset function or passing true to the Start "
        "function will reset this value.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
