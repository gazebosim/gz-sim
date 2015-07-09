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
#ifndef _IGNITION_PYRAMID_FRUSTUM_PRIVATE_HH_
#define _IGNITION_PYRAMID_FRUSTUM_PRIVATE_HH_

#include <array>

#include <ignition/math/Angle.hh>
#include <ignition/math/Plane.hh>

namespace ignition
{
  namespace math
  {
    /// \internal
    /// \brief Private data for the PyramidFrustum class
    class PyramidFrustumPrivate
    {
      /// \brief Constructor
      public: PyramidFrustumPrivate(const double _nearClipDist,
                                    const double _farClipDist,
                                    const math::Angle &_fov)
              : nearClip(_nearClipDist), farClip(_farClipDist), fov(_fov),
                aspectRatio(1.0)
              {
              }

      /// \brief Near clip distance
      public: double nearClip;

      /// \brief Far clip distance
      public: double farClip;

      /// \brief Field of view
      public: math::Angle fov;

      /// \brief Aspect ratio of the near and far clip planes. This is the
      // width divided by the height.
      public: double aspectRatio;

      /// \brief Each plane of the frustum.
      /// \sa PyramidFrustum::PyramidFrustumPlane
      public: std::array<Planed, 6> planes;
    };
  }
}
#endif
