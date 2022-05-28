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

#include <gz/math/Vector3Stats.hh>

#include "Vector3Stats.hh"

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathVector3Stats(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Vector3Stats;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def("insert_data",
         &Class::InsertData,
         "Add a new sample to the statistical measures")
    .def("insert_statistic",
         &Class::InsertStatistic,
         "Add a new type of statistic.")
    .def("insert_statistics",
         &Class::InsertStatistics,
         "Set the size of the box.")
    .def("reset",
         &Class::Reset,
         "Forget all previous data.")
    .def("x",
         py::overload_cast<>(&Class::X),
         py::return_value_policy::reference_internal,
         "Get statistics for x component of signal.")
    .def("y",
         py::overload_cast<>(&Class::Y),
         py::return_value_policy::reference_internal,
         "Get statistics for y component of signal.")
    .def("z",
         py::overload_cast<>(&Class::Z),
         py::return_value_policy::reference_internal,
         "Get statistics for z component of signal.")
    .def("mag",
         py::overload_cast<>(&Class::Mag),
         py::return_value_policy::reference_internal,
         "Get statistics for mag component of signal.")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace math
}  // namespace gz
