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

%module frustum
%{
#include <gz/math/Frustum.hh>
#include <gz/math/Angle.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/config.hh>
%}

namespace gz
{
  namespace math
  {
    class Frustum
    {
      /// \brief Planes that define the boundaries of the frustum.
      public: enum FrustumPlane
      {
        /// \brief Near plane
        FRUSTUM_PLANE_NEAR   = 0,

        /// \brief Far plane
        FRUSTUM_PLANE_FAR    = 1,

        /// \brief Left plane
        FRUSTUM_PLANE_LEFT   = 2,

        /// \brief Right plane
        FRUSTUM_PLANE_RIGHT  = 3,

        /// \brief Top plane
        FRUSTUM_PLANE_TOP    = 4,

        /// \brief Bottom plane
        FRUSTUM_PLANE_BOTTOM = 5
      };

      public: Frustum();

      public: Frustum(double _near,
                  double _far,
                  const gz::math::Angle &_fov,
                  double _aspectRatio,
                  const gz::math::Pose3<double> &_pose = gz::math::Pose3<double>::Zero);

      public: Frustum(const Frustum &_p);

      public: double Near() const;

      public: void SetNear(double _near);

      public: double Far() const;

      public: void SetFar(double _far);

      public: gz::math::Angle FOV() const;

      public: void SetFOV(const gz::math::Angle &_fov);

      public: double AspectRatio() const;

      public: void SetAspectRatio(double _aspectRatio);

      public: gz::math::Plane<double> Plane(const FrustumPlane _plane) const;

      public: bool Contains(const gz::math::AxisAlignedBox &_b) const;

      public: bool Contains(const gz::math::Vector3<double> &_p) const;

      public: gz::math::Pose3<double> Pose() const;

      public: void SetPose(const gz::math::Pose3<double> &_pose);
    };
  }
}
