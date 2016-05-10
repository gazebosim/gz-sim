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
#ifndef IGNITION_MATH_BOX_PRIVATE_HH_
#define IGNITION_MATH_BOX_PRIVATE_HH_

#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    /// \internal
    /// \brief Private data for Box class
    class BoxPrivate
    {
      /// \brief Enumeration of extents
      public: enum Extent {EXTENT_NULL, EXTENT_FINITE};

      /// \brief Minimum corner of the box
      public: Vector3d min = Vector3d::Zero;

      /// \brief Maximum corner of the box
      public: Vector3d max = Vector3d::Zero;

      /// \brief When set to EXTENT_NULL (the default value)
      /// the min and max are not valid positions
      public: Extent extent = EXTENT_NULL;
    };
  }
}
#endif
