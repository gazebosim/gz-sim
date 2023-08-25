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

#include "gz/sim/TestFixture.hh"

#include "wrap_functions.hh"

namespace gz
{
namespace sim
{
namespace python
{
void
defineSimTestFixture(pybind11::object module)
{
  pybind11::class_<TestFixture, std::shared_ptr<TestFixture>> testFixture(module, "TestFixture");

  // Since this class starts with "Test", pytest thinks it's a test and emits a
  // warning when it can't "collect" it.
  // Silence the warning by setting `__test__=False`.
  testFixture.attr("__test__") = false;

  testFixture
  .def(pybind11::init<const std::string &>())
  .def(
    "server", &TestFixture::Server,
    pybind11::return_value_policy::reference,
    "Get pointer to underlying server."
  )
  .def(
    "finalize", &TestFixture::Finalize,
    pybind11::return_value_policy::reference,
    "Finalize all the functions and add fixture to server."
  )
  .def(
    "on_pre_update", WrapCallbacks(
      [](TestFixture* self, std::function<void(
          const UpdateInfo &, EntityComponentManager &)> _cb)
      {
        self->OnPreUpdate(_cb);
      }
    ),
    pybind11::return_value_policy::reference,
    "Wrapper around a system's pre-update callback"
  )
  .def(
    "on_update", WrapCallbacks(
      [](TestFixture* self, std::function<void(
          const UpdateInfo &, EntityComponentManager &)> _cb)
      {
        self->OnUpdate(_cb);
      }
    ),
    pybind11::return_value_policy::reference,
    "Wrapper around a system's update callback"
  )
  .def(
    "on_post_update", WrapCallbacks(
      [](TestFixture* self, std::function<void(
          const UpdateInfo &, const EntityComponentManager &)> _cb)
      {
        self->OnPostUpdate(_cb);
      }
    ),
    pybind11::return_value_policy::reference,
    "Wrapper around a system's post-update callback"
  );
  // TODO(ahcorde): This method is not compiling for the following reason:
  // The EventManager class has an unordered_map which holds a unique_ptr
  // This make the class uncopyable, anyhow we should not copy the class
  // we just need the reference. Not really sure about what's going on here
  // .def(
  //   "on_configure", WrapCallbacks(
  //     [](TestFixture* self, std::function<void(
  //         const Entity &_entity,
  //         const std::shared_ptr<const sdf::Element> &_sdf,
  //         EntityComponentManager &_ecm,
  //         EventManager &_eventMgr)> _cb)
  //     {
  //       self->OnConfigure(_cb);
  //     }
  //   ),
  //   pybind11::return_value_policy::reference,
  //   "Wrapper around a system's configure callback"
  // );
}
}
}
}
