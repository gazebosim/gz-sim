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

#include <chrono>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/chrono.h>

#include <gz/math/GaussMarkovProcess.hh>

#include "GaussMarkovProcess.hh"

namespace py = pybind11;

namespace gz
{
namespace math
{
namespace python
{
void defineMathGaussMarkovProcess(
  py::module &m, const std::string &typestr)
{
  using Class = gz::math::GaussMarkovProcess;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double, double, double>())
    .def("set",
         &Class::Set,
         "Set the process parameters. This will also call Reset().")
    .def("start", &Class::Start,
         "Get the start value.")
    .def("value",
         &Class::Value,
         "Get the current process value.")
    .def("theta", &Class::Theta, "Get the theta value.")
    .def("mu", &Class::Mu, "Get the mu value.")
    .def("sigma", &Class::Sigma, "Get the sigma value.")
    .def("reset",
         &Class::Reset,
         "Reset the process. This will set the current process value")
    .def("update",
         py::overload_cast<
          const std::chrono::steady_clock::duration&>(
            &Class::Update),
         "Update the process and get the new value.")
    .def("update",
         py::overload_cast<double>(&Class::Update),
         "Update the process and get the new value.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
