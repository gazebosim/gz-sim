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
uint32_t ignition::math::Pair(const uint16_t _a, const uint16_t _b)
{
  // Store in 32bit local variable so that we don't overflow.
  uint32_t a = _a;
  uint32_t b = _b;

  // Szudzik's function
  return _a >= _b ?  a * a + a + b : a + b * b;
}

/////////////////////////////////////////////
std::tuple<uint16_t, uint16_t> ignition::math::Unpair(const uint32_t _key)
{
  // Must explicitly cast so that the _key is not auto cast to a double
  uint32_t sqrt = static_cast<uint32_t>(
      std::floor(std::sqrt(static_cast<long double>(_key))));
  uint32_t sq = sqrt * sqrt;

  return ((_key - sq) >= sqrt) ?
    std::make_tuple(static_cast<uint16_t>(sqrt),
                    static_cast<uint16_t>(_key - sq - sqrt)) :
    std::make_tuple(static_cast<uint16_t>(_key - sq),
                    static_cast<uint16_t>(sqrt));
}
