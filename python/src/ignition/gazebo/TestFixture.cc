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

#include "wrap_functions.hh"

// The wraps methods were copied from:
// https://github.com/RobotLocomotion/drake/blob/6ee5e9325821277a62bd5cd5456ccf02ca25dab7/bindings/pydrake/common/wrap_pybind.h
// It's under BSD 3-Clause License

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


// Determines if a type will go through pybind11's generic caster. This
// implies that the type has been declared using `pybind11::class_`, and can have
// a reference passed through. Otherwise, the type uses type-conversion:
// https://pybind11.readthedocs.io/en/stable/advanced/cast/index.html
template <typename T>
constexpr inline bool is_generic_pybind_v =
    std::is_base_of_v<pybind11::detail::type_caster_generic,
        pybind11::detail::make_caster<T>>;

template <typename T, typename = void>
struct wrap_ref_ptr : public wrap_arg_default<T> {};

template <typename T>
struct wrap_ref_ptr<T&,
  std::enable_if_t<is_generic_pybind_v<T> &&
  !std::is_same_v<T, const std::shared_ptr<const sdf::Element>> >> {
  // NOLINTNEXTLINE[runtime/references]: Intentional.
  static T* wrap(T& arg) { return &arg; }
  static T& unwrap(T* arg_wrapped) {
    return *arg_wrapped;
  }
};

template <typename T, typename = void>
struct wrap_callback : public wrap_arg_default<T> {};

template <typename Signature>
struct wrap_callback<const std::function<Signature>&>
    : public wrap_arg_function<wrap_ref_ptr, Signature> {};

template <typename Signature>
struct wrap_callback<std::function<Signature>>
    : public wrap_callback<const std::function<Signature>&> {};


/// Ensures that any `std::function<>` arguments are wrapped such that any `T&`
/// (which can infer for `T = const U`) is wrapped as `U*` (and conversely
/// unwrapped when returned).
/// Use this when you have a callback in C++ that has a lvalue reference (const
/// or mutable) to a C++ argument or return value.
/// Otherwise, `pybind11` may try and copy the object, will be bad if either
/// the type is a non-copyable or if you are trying to mutate the object; in
/// this case, the copy is mutated, but not the original you care about.
/// For more information, see: https://github.com/pybind/pybind11/issues/1241
template <typename Func>
auto WrapCallbacks(Func&& func) {
  return WrapFunction<wrap_callback, false>(std::forward<Func>(func));
}

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
  )
  .def(
    "on_configure", WrapCallbacks(
      [](TestFixture* self, std::function<void(
          const Entity &_entity,
          const std::shared_ptr<const sdf::Element> &_sdf,
          EntityComponentManager &_ecm,
          EventManager &_eventMgr)> _cb)
      {
        self->OnConfigure(_cb);
      }
    ),
    pybind11::return_value_policy::reference,
    "Wrapper around a system's pre-update callback"
  );
}
}
}
}
