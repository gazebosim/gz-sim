/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_VECTOR3_STATS_PRIVATE_HH_
#define IGNITION_MATH_VECTOR3_STATS_PRIVATE_HH_

#include <ignition/math/SignalStats.hh>

namespace ignition
{
  namespace math
  {
    /// \brief Private data class for the Vector3Stats class.
    class Vector3StatsPrivate
    {
      /// \brief Statistics for x component of signal.
      public: SignalStats x;

      /// \brief Statistics for y component of signal.
      public: SignalStats y;

      /// \brief Statistics for z component of signal.
      public: SignalStats z;

      /// \brief Statistics for magnitude of signal.
      public: SignalStats mag;
    };
  }
}
#endif

