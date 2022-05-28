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

#include <sstream>
#include <string>

#include <gz/math/SemanticVersion.hh>
#include <pybind11/operators.h>

#include "SemanticVersion.hh"

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathSemanticVersion(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SemanticVersion;
  std::string pyclass_name = typestr;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<std::string&>())
  .def(py::init<const Class&>())
  .def(py::init<const unsigned int, const unsigned int, const unsigned int,
                std::string&, std::string>(),
                py::arg("_major") = 0, py::arg("_minor") = 0,
                py::arg("_patch") = 0, py::arg("_prerelease") = "",
                py::arg("_prerelease") = "")
  .def(py::self != py::self)
  .def(py::self == py::self)
  .def(py::self < py::self)
  .def(py::self <= py::self)
  .def(py::self > py::self)
  .def(py::self >= py::self)
  .def("parse",
       &Class::Parse,
       "Parse a version string and set the major, minor, patch "
       "numbers, and prerelease and build strings.")
   .def("version",
        &Class::Version,
        "Returns the version as a string")
   .def("major",
        &Class::Major,
        "Get the major number")
   .def("minor",
        &Class::Minor,
        "Get the minor number")
   .def("patch",
        &Class::Patch,
        "Get the patch number")
   .def("prerelease",
        &Class::Prerelease,
        "Get the prerelease string.")
   .def("build",
        &Class::Build,
        "Get the build metadata string. Build meta data is not used "
        "when determining precedence.")
   .def("__copy__", [](const Class &self) {
     return Class(self);
   })
   .def("__deepcopy__", [](const Class &self, py::dict) {
     return Class(self);
   }, "memo"_a)
   .def("__str__", toString)
   .def("__repr__", toString);
}
}  // namespace python
}  // namespace math
}  // namespace gz
