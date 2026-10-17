/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "PhysicsMode.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
//////////////////////////////////////////////////
bool ParseMode(const std::string &_name, PhysicsMode &_mode)
{
  if (_name == "full")
    _mode = PhysicsMode::Full;
  else if (_name == "planar_kinematic")
    _mode = PhysicsMode::PlanarKinematic;
  else if (_name == "planar_dynamic")
    _mode = PhysicsMode::PlanarDynamic;
  else
    return false;
  return true;
}

//////////////////////////////////////////////////
std::string ModeName(PhysicsMode _mode)
{
  switch (_mode)
  {
    case PhysicsMode::Full:
      return "full";
    case PhysicsMode::PlanarKinematic:
      return "planar_kinematic";
    case PhysicsMode::PlanarDynamic:
      return "planar_dynamic";
    default:
      break;
  }
  return "";
}

//////////////////////////////////////////////////
std::vector<std::string> SupportedFeatures(PhysicsMode)
{
  // A kinematic body takes no forces, torques or contact parameters, and it
  // has no joints that an engine solves. Each planar mode gains features as
  // the system gains the code that honors them.
  return {};
}
}
}
}
