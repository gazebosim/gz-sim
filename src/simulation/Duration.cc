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

#include <gz/sim/simulation/Duration.hh>

namespace gz::sim::simulation
{

Duration::Duration(int32_t _seconds, uint32_t _nanoseconds)
{
  this->duration = static_cast<int64_t>(_seconds) * 1000LL * 1000LL * 1000LL + _nanoseconds;
}

Duration::Duration(std::chrono::nanoseconds _nanoseconds)
{
  this->duration = _nanoseconds.count();
}

Duration::Duration(const Duration &_rhs) = default;

int64_t Duration::Nanoseconds() const {
  return this->duration;
}

double Duration::Seconds() const {
  return std::chrono::duration<double>(std::chrono::nanoseconds(this->duration)).count();
}


}  // namespace gz::sim::simulation;
