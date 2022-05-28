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
#include "RollingMean.hh"
#include <gz/math/RollingMean.hh>

namespace gz
{
namespace math
{
namespace python
{
void defineMathRollingMean(py::module &m, const std::string &typestr)
{
  using Class = gz::math::RollingMean;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<size_t>(), py::arg("_windowSize") = 10)
  .def("mean", &Class::Mean, "Get the mean value.")
   .def("count", &Class::Count, "Get the number of data points.")
   .def("push", &Class::Push, "Insert a new value.")
   .def("clear", &Class::Clear, "Remove all the pushed values.")
   .def("set_window_size",
        &Class::SetWindowSize,
        "Set the new window size. This will also clear the data. "
        "Nothing happens if the _windowSize is zero.")
   .def("window_size", &Class::WindowSize, "Get the window size.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
