/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_CLOCK_HH_
#define GZ_SIM_CLOCK_HH_

#include <gz/sim/simulation/Duration.hh>
#include <gz/sim/simulation/Time.hh>

#include <gz/utils/ImplPtr.hh>

namespace gz::sim::simulation
{

class Clock
{
  public: explicit Clock();

  public: Time Now() const;

  public: void Step(Duration _delta);

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(dataPtr)
};

}  // namespace gz::sim::simulation

#endif  // GZ_SIM_CLOCK_HH_
