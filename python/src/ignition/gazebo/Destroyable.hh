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

#ifndef IGNITION_GAZEBO_PYTHON__DESTROYABLE_HPP_
#define IGNITION_GAZEBO_PYTHON__DESTROYABLE_HPP_

#include <pybind11/pybind11.h>

namespace ignition
{
namespace gazebo
{
namespace python
{
/// This class blocks destruction when in use.
/// This class is required to handle the lifecycle of the
/// object.
class Destroyable
{
public:
  /// Default constructor
  Destroyable() = default;

  /// Copy constructor
  Destroyable(const Destroyable & other);

  /// Context manager __enter__ - block destruction
  void
  Enter();

  /// Context manager __exit__ - unblock destruction
  void
  Exit(pybind11::object pytype,
    pybind11::object pyvalue,
    pybind11::object pytraceback);

  /// Signal that the object should be destroyed as soon as it's not in use
  void
  DestroyWhenNotInUse();

  /// Override this to destroy an object
  virtual
  void
  Destroy();

  virtual
  ~Destroyable() = default;

private:
  size_t use_count = 0u;
  bool please_destroy_ = false;
};

/// Define a pybind11 wrapper for an rclpybind11::Destroyable
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineDestroyable(pybind11::object module);
}  // namespace python
}  // namespace ignition
}  // namespace gazebo
#endif  // IGNITION_GAZEBO_PYTHON__DESTROYABLE_HPP_
