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
ignition::math::PairOutput ignition::math::Pair(
    const ignition::math::PairInput _a, const ignition::math::PairInput _b)
{
  // Store in 64bit local variable so that we don't overflow.
  uint64_t a = _a;
  uint64_t b = _b;

  // Szudzik's function
  return _a >= _b ?
          static_cast<PairOutput>(a * a + a + b) :
          static_cast<PairOutput>(a + b * b);
}

/////////////////////////////////////////////
std::tuple<ignition::math::PairInput, ignition::math::PairInput>
ignition::math::Unpair(const ignition::math::PairOutput _key)
{
  // Must explicitly cast so that the _key is not auto cast to a double
  uint64_t sqrt = static_cast<uint64_t>(
      std::floor(std::sqrt(static_cast<long double>(_key))));
  uint64_t sq = sqrt * sqrt;

  return ((_key - sq) >= sqrt) ?
    std::make_tuple(static_cast<PairInput>(sqrt),
                    static_cast<PairInput>(_key - sq - sqrt)) :
    std::make_tuple(static_cast<PairInput>(_key - sq),
                    static_cast<PairInput>(sqrt));
}
