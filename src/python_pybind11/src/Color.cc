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

#include "Color.hh"
#include <gz/math/Color.hh>

#include <pybind11/operators.h>

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathColor(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Color;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<const Class&>())
  .def(py::init<const float, const float, const float, const float>(),
       py::arg("_r") = 0.0, py::arg("_g") = 0.0,
       py::arg("_b") = 0.0, py::arg("_a") = 1.0f)
  .def(py::self + py::self)
  .def(py::self += py::self)
  .def(py::self + float())
  .def(py::self - py::self)
  .def(py::self -= py::self)
  .def(py::self - float())
  .def(py::self / py::self)
  .def(py::self /= py::self)
  .def(py::self / float())
  .def(py::self * py::self)
  .def(py::self *= py::self)
  .def(py::self * float())
  .def(py::self != py::self)
  .def(py::self == py::self)
  .def_readonly_static("WHITE", &Class::White, "(1, 1, 1)")
  .def_readonly_static("BLACK", &Class::Black, "(0, 0, 0)")
  .def_readonly_static("RED", &Class::Red, "(1, 0, 0)")
  .def_readonly_static("GREEN", &Class::Green, "(0, 1, 0)")
  .def_readonly_static("BLUE", &Class::Blue, "(0, 0, 1)")
  .def_readonly_static("YELLOW", &Class::Yellow, "(1, 1, 0)")
  .def_readonly_static("MAGENTA", &Class::Magenta, "(1, 0, 1)")
  .def_readonly_static("CYAN", &Class::Cyan, "(0, 1, 1)")
  .def("reset",
        &Class::Reset,
        "Reset the color to default values to red=0, green=0, blue=0, alpha=1")
   .def("set",
        &Class::Set,
        py::arg("_r") = 1.0, py::arg("_g") = 1.0,
        py::arg("_b") = 1.0, py::arg("_a") = 1.0,
        "Set the contents of the vector")
   .def("hsv",
        &Class::HSV,
        "Set the color in HSV colorspace")
   .def("set_from_hsv",
        &Class::SetFromHSV,
        "Set a color based on HSV values")
   .def("yuv",
        &Class::YUV,
        "Get the color in YUV colorspace")
   .def("set_from_yuv",
        &Class::SetFromYUV,
        "Set from yuv")
   .def("as_rgba",
        &Class::AsRGBA,
        "Get as uint32 RGBA packed value")
   .def("as_bgra",
        &Class::AsBGRA,
        "Get as uint32 BGRA packed value")
   .def("as_argb",
        &Class::AsARGB,
        "Get as uint32 ARGB packed value")
   .def("as_abgr",
        &Class::AsABGR,
        "Get as uint32 ABGR packed value")
   .def("set_from_rgba",
        &Class::SetFromRGBA,
        "Set from uint32 RGBA packed value")
   .def("set_from_bgra",
        &Class::SetFromBGRA,
        "Set from uint32 BGRA packed value")
   .def("set_from_argb",
        &Class::SetFromARGB,
        "Set from uint32 ARGB packed value")
   .def("set_from_abgr",
        &Class::SetFromABGR,
        "Set from uint32 ABGR packed value")
   .def("r",
        py::overload_cast<>(&Class::R),
        "Get the red value")
   .def("g",
        py::overload_cast<>(&Class::G),
        "Get the green value")
   .def("b",
        py::overload_cast<>(&Class::B),
        "Get the blue value")
   .def("a",
        py::overload_cast<>(&Class::A),
        "Get the alpha value")
   .def("r",
        py::overload_cast<float>(&Class::R),
        "Get the red value")
   .def("g",
        py::overload_cast<float>(&Class::G),
        "Get the green value")
   .def("b",
        py::overload_cast<float>(&Class::B),
        "Get the blue value")
   .def("a",
        py::overload_cast<float>(&Class::A),
        "Get the alpha value")
   .def("__str__", toString)
   .def("__repr__", toString)
   .def("__copy__", [](const Class &self) {
     return Class(self);
   })
   .def("__deepcopy__", [](const Class &self, py::dict) {
     return Class(self);
   }, "memo"_a)
   .def("__getitem__",
        py::overload_cast<const unsigned int>(&Class::operator[]));
}
}  // namespace python
}  // namespace math
}  // namespace gz
