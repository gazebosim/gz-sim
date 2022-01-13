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
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "TestFixture.hh"

#include "ignition/gazebo/TestFixture.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
void
defineGazeboTestFixture(pybind11::object module)
{
  pybind11::class_<TestFixture> testFixture(module, "TestFixture");

  testFixture
  .def(pybind11::init<const std::string &>())
  .def(
    "server", &TestFixture::Server,
    "Get pointer to underlying server."
  )
  .def(
    "finalize", &TestFixture::Finalize,
    pybind11::return_value_policy::reference,
    "Finalize all the functions and add fixture to server."
  )
  .def(
    "on_pre_update", &TestFixture::OnPreUpdate,
    pybind11::return_value_policy::reference,
    "Wrapper around a system's pre-update callback"
  )
  .def(
    "on_update", &TestFixture::OnUpdate,
    pybind11::return_value_policy::reference_internal,
    "Wrapper around a system's update callback"
  )
  .def(
    "on_post_update", &TestFixture::OnPostUpdate,
    pybind11::return_value_policy::reference_internal,
    "Wrapper around a system's post-update callback"
  )
  .def(
    "on_configure", &TestFixture::OnConfigure,
    pybind11::return_value_policy::reference,
    "Wrapper around a system's pre-update callback"
  );
}
}
}
}
