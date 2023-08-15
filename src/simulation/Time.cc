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

#include <gz/sim/simulation/Time.hh>

namespace gz::sim::simulation
{

Time::Time(int32_t _seconds, uint32_t _nanoseconds)
{
  this->timePoint = static_cast<int64_t>(_seconds) * 1000LL * 1000LL * 1000LL + _nanoseconds;
}

Time::Time(int64_t _nanoseconds)
{
  this->timePoint = _nanoseconds;
}

Time::~Time() = default;


int64_t Time::Nanoseconds() const {
  return this->timePoint;
}

double Time::Seconds() const {
  return std::chrono::duration<double>(std::chrono::nanoseconds(this->timePoint)).count();
}

}  // namespace gz::sim::simulation;
