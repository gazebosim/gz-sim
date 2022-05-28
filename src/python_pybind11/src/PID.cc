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
#include <string>

#include <gz/math/PID.hh>

#include "PID.hh"

namespace gz
{
namespace math
{
namespace python
{
void defineMathPID(py::module &m, const std::string &typestr)
{
  using Class = gz::math::PID;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<Class&>())
  .def(py::init<const double, const double, const double, const double,
                const double, const double, const double, const double>(),
                py::arg("_p") = 0.0, py::arg("_i") = 0.0, py::arg("_d") = 0.0,
                py::arg("_imax") = -1.0, py::arg("_imin") = 0.0,
                py::arg("_cmdMax") = -1.0, py::arg("_cmdMin") = 0.0,
                py::arg("_cmdOffset") = 0.0)
  .def("init",
       &Class::Init,
       py::arg("_p") = 0.0, py::arg("_i") = 0.0, py::arg("_d") = 0.0,
       py::arg("_imax") = -1.0, py::arg("_imin") = 0.0,
       py::arg("_cmdMax") = -1.0, py::arg("_cmdMin") = 0.0,
       py::arg("_cmdOffset") = 0.0,
       "Initialize PID-gains and integral term "
       "limits:[iMax:iMin]-[I1:I2].")
   .def("set_p_gain",
        &Class::SetPGain,
        "Set the proportional Gain.")
   .def("set_i_gain",
        &Class::SetIGain,
        "Set the integral Gain.")
   .def("set_d_gain",
        &Class::SetDGain,
        "Set the derivative Gain.")
   .def("set_i_max",
        &Class::SetIMax,
        "Set the integral upper limit.")
   .def("set_i_min",
        &Class::SetIMin,
        "Set the integral lower limit.")
   .def("set_cmd_max",
        &Class::SetCmdMax,
        "Set the maximum value for the command.")
   .def("set_cmd_min",
        &Class::SetCmdMin,
        "Set the minimum value for the command.")
   .def("set_cmd_offset",
        &Class::SetCmdOffset,
        "Set the offset value for the command, which is added to the result of "
        "the PID controller.")
   .def("p_gain",
        &Class::PGain,
        "Get the proportional Gain.")
   .def("i_gain",
        &Class::IGain,
        "Get the integral Gain.")
   .def("d_gain",
        &Class::DGain,
        "Get the derivative Gain.")
   .def("i_max",
        &Class::IMax,
        "Get the integral upper limit.")
   .def("i_min",
        &Class::IMin,
        "Get the integral lower limit.")
   .def("cmd_max",
        &Class::CmdMax,
        "Get the maximum value for the command.")
   .def("cmd_min",
        &Class::CmdMin,
        "Get the minimun value for the command.")
   .def("cmd_offset",
        &Class::CmdOffset,
        "Get the offset value for the command.")
   .def("update",
        &Class::Update,
        "Update the Pid loop with nonuniform time step size.")
   .def("set_cmd",
        &Class::SetCmd,
        "Set current target command for this PID controller.")
   .def("cmd",
        &Class::Cmd,
        "Return current command for this PID controller.")
   .def("errors",
        [](const Class &self) {
          double pe, ie, de;
          self.Errors(pe, ie, de);
          return std::make_tuple(pe, ie, de);
        },
        "Return PID error terms for the controller.")
   .def("reset",
        &Class::Reset,
        "Reset the errors and command.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
