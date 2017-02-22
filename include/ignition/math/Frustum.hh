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
#ifndef IGNITION_MATH_FRUSTUM_HH_
#define IGNITION_MATH_FRUSTUM_HH_

#include <ignition/math/Plane.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FrustumPrivate;

    /// \brief Mathematical representation of a frustum and related functions.
    /// This is also known as a view frustum.
    class IGNITION_VISIBLE Frustum
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

      /// \brief Default constructor. With the following default values:
      ///
      /// * near: 0.0
      /// * far: 1.0
      /// * fov: 0.78539 radians (45 degrees)
      /// * aspect ratio: 1.0
      /// * pose: Pose3d::Zero
      public: Frustum();

      /// \brief Constructor
      /// \param[in] _near Near plane distance. This is the distance from
      /// the frustum's vertex to the closest plane
      /// \param[in] _far Far plane distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \param[in] _fov Field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \param[in] _aspectRatio The aspect ratio, which is the width divided
      /// by height of the near or far planes.
      /// \param[in] _pose Pose of the frustum, which is the vertex (top of
      /// the pyramid).
      public: Frustum(const double _near,
                      const double _far,
                      const math::Angle &_fov,
                      const double _aspectRatio,
                      const math::Pose3d &_pose = math::Pose3d::Zero);

      /// \brief Copy Constructor
      /// \param[in] _p Frustum to copy.
      public: Frustum(const Frustum &_p);

      /// \brief Destructor
      public: virtual ~Frustum();

      /// \brief Get the near distance. This is the distance from the
      /// frustum's vertex to the closest plane.
      /// \return Near distance.
      /// \sa SetNear
      public: double Near() const;

      /// \brief Set the near distance. This is the distance from the
      /// frustum's vertex to the closest plane.
      /// \param[in] _near Near distance.
      /// \sa Near
      public: void SetNear(const double _near);

      /// \brief Get the far distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \return Far distance.
      /// \sa SetFar
      public: double Far() const;

      /// \brief Set the far distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \param[in] _far Far distance.
      /// \sa Far
      public: void SetFar(const double _far);

      /// \brief Get the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \return The field of view.
      /// \sa SetFOV
      public: math::Angle FOV() const;

      /// \brief Set the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \param[in] _fov The field of view.
      /// \sa FOV
      public: void SetFOV(const math::Angle &_fov);

      /// \brief Get the aspect ratio, which is the width divided by height
      /// of the near or far planes.
      /// \return The frustum's aspect ratio.
      /// \sa SetAspectRatio
      public: double AspectRatio() const;

      /// \brief Set the aspect ratio, which is the width divided by height
      /// of the near or far planes.
      /// \param[in] _aspectRatio The frustum's aspect ratio.
      /// \sa AspectRatio
      public: void SetAspectRatio(const double _aspectRatio);

      /// \brief Get a plane of the frustum.
      /// \param[in] _plane The plane to return.
      /// \return Plane of the frustum.
      public: Planed Plane(const FrustumPlane _plane) const;

      /// \brief Check if a box lies inside the pyramid frustum.
      /// \param[in] _b Box to check.
      /// \return True if the box is inside the pyramid frustum.
      public: bool Contains(const Box &_b) const;

      /// \brief Check if a point lies inside the pyramid frustum.
      /// \param[in] _p Point to check.
      /// \return True if the point is inside the pyramid frustum.
      public: bool Contains(const Vector3d &_p) const;

      /// \brief Get the pose of the frustum
      /// \return Pose of the frustum
      /// \sa SetPose
      public: Pose3d Pose() const;

      /// \brief Set the pose of the frustum
      /// \param[in] _pose Pose of the frustum, top vertex.
      /// \sa Pose
      public: void SetPose(const Pose3d &_pose);

      /// \brief Assignment operator. Set this frustum to the parameter.
      /// \param[in]  _b Frustum to copy
      /// \return The new frustum.
      public: Frustum &operator=(const Frustum &_f);

      /// \brief Compute the planes of the frustum. This is called whenever
      /// a property of the frustum is changed.
      private: void ComputePlanes();

      /// \internal
      /// \brief Private data pointer
      private: FrustumPrivate *dataPtr;
    };
  }
}
#endif
