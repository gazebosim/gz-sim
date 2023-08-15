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

#ifndef GZ_SIM_SIMULATION_TIME_HH_
#define GZ_SIM_SIMULATION_TIME_HH_

#include <cstdint>

#include <chrono>

namespace gz::sim::simulation
{

class Time
{
  public: Time(int32_t _seconds, uint32_t _nanoseconds);

  public: explicit Time(int64_t _nanoseconds = 0);

  public: virtual ~Time();

  public: double Seconds() const;

  public: int64_t Nanoseconds() const;

  private: int64_t timePoint;
};

}  // namespace gz::sim::simulation;

#endif  // GZ_SIM_SIMULATION_TIME_HH_
