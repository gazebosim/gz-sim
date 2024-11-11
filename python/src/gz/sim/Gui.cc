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
 */


#include <pybind11/pybind11.h>

#include <gz/sim/gui/Gui.hh>
#include <gz/common/Console.hh>

#include "Server.hh"

namespace gz
{
namespace sim
{
namespace python
{
void defineGuiClient(pybind11::module &_module)
{

  _module.def("run_gui", [](){
    auto pid = fork();
    if (pid == -1)
    {
      gzerr << "Failed to instantiate new process";
      return;
    }
    if (pid != 0)
    {
      return;
    }
    int zero = 0;
    gz::sim::gui::runGui(zero, nullptr, nullptr);
  },
  "Run the gui");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
