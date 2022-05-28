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

#include "Rand.hh"
#include <gz/math/Rand.hh>

namespace gz
{
namespace math
{
namespace python
{
void defineMathRand(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Rand;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("seed",
        py::overload_cast<>(&gz::math::Rand::Seed),
        "Get the seed value.")
   .def("seed",
        py::overload_cast<unsigned int>(&gz::math::Rand::Seed),
        "Set the seed value.")
   .def("dbl_uniform",
        gz::math::Rand::DblUniform,
        "Get a double from a uniform distribution")
   .def("dbl_normal",
        gz::math::Rand::DblNormal,
        "Get a double from a normal distribution")
   .def("int_uniform",
        gz::math::Rand::IntUniform,
        "Get a integer from a uniform distribution")
   .def("int_normal",
        gz::math::Rand::IntNormal,
        "Get a integer from a normal distribution");
}
}  // namespace python
}  // namespace math
}  // namespace gz
