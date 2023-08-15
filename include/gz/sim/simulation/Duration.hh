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

#ifndef GZ_SIM_DURATION_HH_
#define GZ_SIM_DURATION_HH_

#include <cstdint>

#include <chrono>

namespace gz::sim::simulation
{

class Duration
{
  public: Duration(int32_t _seconds, uint32_t _nanoseconds);

  /// Construct duration from the specified std::chrono::nanoseconds.
  public: explicit Duration(std::chrono::nanoseconds _nanoseconds);

  // This constructor matches any std::chrono value other than nanoseconds
  // intentionally not using explicit to create a conversion constructor
  template<class Rep, class Period>
  // cppcheck-suppress noExplicitConstructor
  Duration(const std::chrono::duration<Rep, Period> &_duration)  // NOLINT(runtime/explicit)
  : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(_duration))
  {}

  public: Duration(const Duration &_rhs);

  public: virtual ~Duration() = default;

  public: int64_t Nanoseconds() const;

  public: double Seconds() const;

  private: Duration() = default;

  private: int64_t duration;
};

}  // namespace gz::sim::simulation;
#endif // GZ_SIM_DURATION_HH_
