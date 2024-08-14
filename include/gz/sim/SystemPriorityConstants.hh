/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMPRIORITYCONSTANTS_HH_
#define GZ_SIM_SYSTEMPRIORITYCONSTANTS_HH_

#include <gz/sim/System.hh>

namespace gz::sim::systems
{
  /// \brief A suggested priprity value for a system that should execute before
  /// the Physics system.
  constexpr System::PriorityType kPrePhysicsPriority = -128;

  /// \brief Default priority value for the Physics system, with a negative
  /// value ensuring that it will run before systems with priority
  /// System::kDefaultPriority.
  constexpr System::PriorityType kPhysicsPriority = -64;

  /// \brief A suggested priority value for sensor systems that should execute
  /// after the Physics system but before the systems with priority
  /// System::kDefaultPriority.
  constexpr System::PriorityType kPostPhysicsSensorPriority = -32;
}

#endif
