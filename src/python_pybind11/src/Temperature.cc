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

#include <gz/math/Temperature.hh>

#include "Temperature.hh"
#include <pybind11/operators.h>

using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathTemperature(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Temperature;
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
  .def(py::init<double>())
  .def(py::init<Class&>())
  .def("__call__", &Class::operator())
  .def(py::self + py::self)
  .def(py::self + double())
  .def(double() + py::self)
  .def(py::self += py::self)
  .def(py::self += double())
  .def(py::self - py::self)
  .def(py::self - double())
  .def(double() - py::self)
  .def(py::self -= py::self)
  .def(py::self -= double())
  .def(py::self * py::self)
  .def(py::self * double())
  .def(double() * py::self)
  .def(py::self *= py::self)
  .def(py::self *= double())
  .def(py::self / py::self)
  .def(py::self / double())
  .def(double() / py::self)
  .def(py::self /= py::self)
  .def(py::self /= double())
  .def(py::self != double())
  .def(py::self != py::self)
  .def(py::self == py::self)
  .def(py::self == double())
  .def(py::self < double())
  .def(py::self < py::self)
  .def(py::self <= double())
  .def(py::self <= py::self)
  .def(py::self > double())
  .def(py::self > py::self)
  .def(py::self >= double())
  .def(py::self >= py::self)
  .def("kelvin_to_celsius",
       &Class::KelvinToCelsius,
       "Convert Kelvin to Celsius")
   .def("kelvin_to_fahrenheit",
        &Class::KelvinToFahrenheit,
        "Convert Kelvin to Fahrenheit")
   .def("celsius_to_fahrenheit",
        &Class::CelsiusToFahrenheit,
        "Convert Celsius to Fahrenheit")
   .def("celsius_to_kelvin",
        &Class::CelsiusToKelvin,
        "Convert Celsius to Kelvin")
   .def("fahrenheit_to_celsius",
        &Class::FahrenheitToCelsius,
        "Convert Fahrenheit to Celsius")
   .def("fahrenheit_to_kelvin",
        &Class::FahrenheitToKelvin,
        "Convert Fahrenheit to Kelvin")
   .def("kelvin",
        &Class::Kelvin,
        "Get the temperature in Kelvin")
   .def("celsius",
        &Class::Celsius,
        "Get the temperature in Celsius")
   .def("fahrenheit",
        &Class::Fahrenheit,
        "Get the temperature in Fahrenheit")
   .def("set_kelvin",
        &Class::SetKelvin,
        "Set the temperature from a Kelvin value")
   .def("set_celsius",
        &Class::SetCelsius,
        "Set the temperature from a Celsius value")
   .def("set_fahrenheit",
        &Class::SetFahrenheit,
        "Set the temperature from a Fahrenheit value")
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
