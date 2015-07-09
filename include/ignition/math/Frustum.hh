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
#ifndef _IGNITION_FRUSTUM_HH_
#define _IGNITION_FRUSTUM_HH_

#include <ignition/math/Plane.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FrustumPrivate;

    /// \class Frustum Frustum.hh ignition/math/Frustum.hh
    /// \brief Mathematical representation of a frustum and related functions.
    /// This is also known as a square frustum or view frustum.
    class IGNITION_VISIBLE Frustum
    {
      /// \brief Planes of that define the boundaries of the frustum
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

      /// \brief Default constructor
      /// \param[in] _nearClip Near clip distance. This is the distance from
      /// the frustum's vertex to the closest clip plane
      /// \param[in] _farClip Far clip distance. This is the distance from the
      /// frustum's vertex to the farthest clip plane.
      /// \param[in] _fov Field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// clip plane. This value represents the horizontal angle.
      /// \param[in] _aspectRatio The aspect ratio, which is the width divided
      /// by height of the near or far clip planes.
      public: Frustum(const double _nearClip,
                      const double _farClip,
                      const math::Angle &_fov,
                      const double _aspectRatio,
                      const math::Pose3d &_pose = math::Pose3d::Zero);

      /// \brief Copy Constructor
      /// \param[in] _p Frustum to copy.
      public: Frustum(const Frustum &_p);

      /// \brief Destructor
      public: virtual ~Frustum();

      /// \brief Get the near clip distance. This is the distance from the
      /// frustum's vertex to the closest clip plane.
      /// \return Near clip distance.
      /// \sa SetNearClip
      public: double NearClip() const;

      /// \brief Set the near clip distance. This is the distance from the
      /// frustum's vertex to the closest clip plane.
      /// \param[in] _near Near clip distance.
      /// \sa NearClip
      public: void SetNearClip(const double _near);

      /// \brief Get the far clip distance. This is the distance from the
      /// frustum's vertex to the farthest clip plane.
      /// \return Far clip distance.
      /// \sa SetFarClip
      public: double FarClip() const;

      /// \brief Set the far clip distance. This is the distance from the
      /// frustum's vertex to the farthest clip plane.
      /// \param[in] _far Far clip distance.
      /// \sa FarClip
      public: void SetFarClip(const double _far);

      /// \brief Get the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// clip plane. This value represents the horizontal angle.
      /// \return The field of view.
      /// \sa SetFOV
      public: math::Angle FOV() const;

      /// \brief Set the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// clip plane. This value represents the horizontal angle.
      /// \param[in] _fov The field of view.
      /// \sa FOV
      public: void SetFOV(const math::Angle &_fov);

      /// \brief Get the aspect ratio, which is the width divided by height
      /// of the near or far clip planes.
      /// \return The frustum's aspect ratio.
      /// \sa SetAspectRatio
      public: double AspectRatio() const;

      /// \brief Set the aspect ratio, which is the width divided by height
      /// of the near or far clip planes.
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
      /// \param[in] _pose Pose of the frustum
      /// \sa Pose
      public: void SetPose(const Pose3d &_pose);

      /// \brief Compute the planes of the frustum. This is called whenever
      /// a property of the frustum is changed.
      private: void ComputePlanes();

      /// \brief Private data pointer
      private: FrustumPrivate *dataPtr;
    };
  }
}
#endif
