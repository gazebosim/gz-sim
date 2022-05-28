/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef GZ_MATH_DETAIL_WELLORDERED_VECTOR_HH_
#define GZ_MATH_DETAIL_WELLORDERED_VECTOR_HH_
#include <gz/math/Vector3.hh>

namespace gz
{
  namespace math
  {
    /// \brief Comparator for well-ordering vectors.
    template<typename T>
    struct WellOrderedVectors
    {
      /// \brief The normal Vector3::operator< is not actually properly ordered.
      /// It does not form an ordinal set. This leads to various complications.
      /// To solve this we introduce this function which orders vector3's by
      /// their X value first, then their Y values and lastly their Z-values.
      /// \param[in] _a - first vector
      /// \param[in] _b - second vector
      /// \return true if _a comes before _b.
      bool operator() (const Vector3<T>& _a, const Vector3<T>& _b) const
      {
        if (_a[0] < _b[0])
        {
          return true;
        }
        else if (equal<T>(_a[0], _b[0], 1e-3))
        {
          if (_a[1] < _b[1])
          {
            return true;
          }
          else if (equal<T>(_a[1], _b[1], 1e-3))
          {
            return _a[2] < _b[2];
          }
          else
          {
            return false;
          }
        }
        return false;
      }
    };
  }
}

#endif
