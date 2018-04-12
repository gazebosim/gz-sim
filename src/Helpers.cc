/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    /////////////////////////////////////////////
    PairOutput Pair(const PairInput _a, const PairInput _b)
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
    std::tuple<PairInput, PairInput> Unpair(const PairOutput _key)
    {
      // Accurate 64-bit integer sqrt
      //  From https://stackoverflow.com/a/18501209
      //
      // \todo: Remove this ifdef and use the "0x1p-20" version when c++17
      // is used. All platforms should then support the "p" literal.
      //
      // Note that the of Hexadecimal floats is failing in some Linux g++
      // compilers from 6.x/7.x series. The native support for C++ is defined
      // in C++17 See https://bugzilla.redhat.com/show_bug.cgi?id=1321986
#if (__cplusplus < 201703L)
      uint64_t sqrt = static_cast<uint64_t>(std::sqrt(_key) -9.53674e-07);
#else
      uint64_t sqrt = static_cast<uint64_t>(std::sqrt(_key) - 0x1p-20);
#endif
      if (2 * sqrt < _key - sqrt * sqrt)
        sqrt++;

      uint64_t sq = sqrt * sqrt;

      return ((_key - sq) >= sqrt) ?
        std::make_tuple(static_cast<PairInput>(sqrt),
                        static_cast<PairInput>(_key - sq - sqrt)) :
        std::make_tuple(static_cast<PairInput>(_key - sq),
                        static_cast<PairInput>(sqrt));
    }
    }
  }
}
