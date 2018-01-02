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
#ifndef IGNITION_MATH_FRUSTUMPRIVATE_HH_
#define IGNITION_MATH_FRUSTUMPRIVATE_HH_

#include <array>
#include <utility>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Plane.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    /// \internal
    /// \brief Private data for the Frustum class
    class FrustumPrivate
    {
      /// \brief Constructor
      /// \param[in] _near Near distance. This is the distance from
      /// the frustum's vertex to the closest plane
      /// \param[in] _far Far distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \param[in] _fov Field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \param[in] _aspectRatio The aspect ratio, which is the width divided
      /// by height of the near or far planes.
      /// \param[in] _pose Pose of the frustum, which is the vertex (top of
      /// the pyramid).
      public: FrustumPrivate(const double _near,
                             const double _far,
                             const math::Angle &_fov,
                             const double _aspectRatio,
                             const Pose3d &_pose)
              : near(_near), far(_far), fov(_fov),
                aspectRatio(_aspectRatio), pose(_pose)
              {
              }

      /// \brief Near distance
      public: double near;

      /// \brief Far distance
      public: double far;

      /// \brief Field of view
      public: math::Angle fov;

      /// \brief Aspect ratio of the near and far planes. This is the
      // width divided by the height.
      public: double aspectRatio;

      /// \brief Pose of the frustum
      public: math::Pose3d pose;

      /// \brief Each plane of the frustum.
      /// \sa Frustum::FrustumPlane
      public: std::array<Planed, 6> planes;

      /// \brief Each corner of the frustum.
      public: std::array<Vector3d, 8> points;

      /// \brief each edge of the frustum.
      public: std::array<std::pair<Vector3d, Vector3d>, 12> edges;
    };
    }
  }
}
#endif
