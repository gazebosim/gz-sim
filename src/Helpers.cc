/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "ignition/math/Helpers.hh"

/////////////////////////////////////////////
uint64_t ignition::math::Pair(const uint32_t _a, const uint32_t _b)
{
  // Store in 64bit local variable so that we don't overflow.
  uint64_t a = _a;
  uint64_t b = _b;

  // Szudzik's function
  return _a >= _b ?  a * a + a + b : a + b * b;
}

/////////////////////////////////////////////
std::tuple<uint32_t, uint32_t> ignition::math::Unpair(const uint64_t _key)
{
  // Must explicityly cast so that the _key is not auto cast to a double
  uint64_t sqrt = std::floor(std::sqrt(static_cast<long double>(_key)));
  uint64_t sq = sqrt * sqrt;

  return ((_key - sq) >= sqrt) ?
    std::make_tuple(sqrt, _key - sq - sqrt) :
    std::make_tuple(_key - sq, sqrt);
}
