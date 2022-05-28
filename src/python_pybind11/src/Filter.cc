/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "Filter.hh"

namespace gz
{
namespace math
{
namespace python
{
void defineMathFilter(py::module &m, const std::string &typestr)
{
  helpDefineMathFilter<int>(m, typestr + "i");
  helpDefineMathFilter<float>(m, typestr + "f");
  helpDefineMathFilter<double>(m, typestr + "d");
}

void defineMathOnePole(py::module &m, const std::string &typestr)
{
  helpDefineMathOnePole<int>(m, typestr + "i");
  helpDefineMathOnePole<float>(m, typestr + "f");
  helpDefineMathOnePole<double>(m, typestr + "d");
}

void defineMathOnePoleQuaternion(py::module &m, const std::string &typestr)
{
  using Class = gz::math::OnePoleQuaternion;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("value",
         &Class::Value,
         "Get the output of the filter.")
    .def("fc",
         &Class::Fc,
         "Set the cutoff frequency and sample rate.")
    .def("process",
         &Class::Process,
         "Update the filter's output.");
}

void defineMathOnePoleVector3(py::module &m, const std::string &typestr)
{
  using Class = gz::math::OnePoleVector3;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("value",
         &Class::
         Value,
         "Get the output of the filter.")
    .def("fc",
         &Class::Fc,
         "Set the cutoff frequency and sample rate.")
    .def("process",
         &Class::Process,
         "Update the filter's output.");
}

void defineMathBiQuad(py::module &m, const std::string &typestr)
{
  helpDefineMathBiQuad<int>(m, typestr + "i");
  helpDefineMathBiQuad<float>(m, typestr + "f");
  helpDefineMathBiQuad<double>(m, typestr + "d");
}

void defineMathBiQuadVector3(py::module &m, const std::string &typestr)
{
  using Class = gz::math::BiQuadVector3;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("fc",
          py::overload_cast<double, double>(&Class::Fc),
         "Set the cutoff frequency and sample rate.")
    .def("value",
         &Class::Value,
         "Get the output of the filter.")
    .def("fc",
         py::overload_cast<double, double, double>(&Class::Fc),
         "Set the cutoff frequency, sample rate and Q coefficient.")
    .def("process",
         &Class::Process,
         "Update the filter's output.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
