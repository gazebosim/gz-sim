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
#ifndef _IGNITION_PYRAMID_FRUSTUM_HH_
#define _IGNITION_PYRAMID_FRUSTUM_HH_

#include <ignition/math/Plane.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class PyramidFrustumPrivate;

    /// \class Box Box.hh ignition/math/Box.hh
    /// \brief Mathematical representation of a box and related functions.
    class IGNITION_VISIBLE PyramidFrustum
    {
      public: enum PyramidFrustumPlane
      {
        PYRAMID_FRUSTUM_PLANE_NEAR   = 0,
        PYRAMID_FRUSTUM_PLANE_FAR    = 1,
        PYRAMID_FRUSTUM_PLANE_LEFT   = 2,
        PYRAMID_FRUSTUM_PLANE_RIGHT  = 3,
        PYRAMID_FRUSTUM_PLANE_TOP    = 4,
        PYRAMID_FRUSTUM_PLANE_BOTTOM = 5
      };

      /// \brief Default constructor
      public: PyramidFrustum(const double _nearClip,
                             const double _farClip,
                             const math::Angle &_fov,
                             const math::Pose3d &_pose);

      /// \brief Copy Constructor
      /// \param[in] _p PyramidFrustum to copy
      public: PyramidFrustum(const PyramidFrustum &_p);

      /// \brief Destructor
      public: virtual ~PyramidFrustum();

      /// \brief Get the near clip distance.
      /// \return Near clip distance.
      public: double NearClip() const;

      /// \brief Get the far clip distance.
      /// \return Far clip distance.
      public: double FarClip() const;

      /// \brief Get the field of view.
      /// \return The field of view.
      public: math::Angle FOV() const;

      /// \brief Get a plane of the frustum.
      /// \param[in] _plane The plane to return.
      /// \return Plane of the frustum.
      public: Planed Plane(const PyramidFrustumPlane _plane) const;

      /// \brief Check if a box lies inside the pyramid frustum.
      /// \param[in] _b Box to check.
      /// \return True if the box is inside the pyramid frustum.
      public: bool Contains(const Box &_b) const;

      /// \brief Check if a point lies inside the pyramid frustum.
      /// \param[in] _p Point to check.
      /// \return True if the point is inside the pyramid frustum.
      public: bool Contains(const Vector3d &_p) const;

      /// \brief Set the pose of the frustum
      /// \param[in] _pose Pose of the frustum
      public: void SetPose(const Pose3d &_pose);

      /// \brief Output operator
      /// \param[in] _out Output stream
      /// \param[in] _b Box to output to the stream
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::PyramidFrustum &_p)
      {
        /*
          _out << _p.dataPtr->planes[0] << " "
               << _p.dataPtr->planes[1] << " "
               << _p.dataPtr->planes[2] << " "
               << _p.dataPtr->planes[3] << " "
               << _p.dataPtr->planes[4] << " "
               << _p.dataPtr->planes[5];
               */

        return _out;
      }

      /// \brief Private data pointer
      private: PyramidFrustumPrivate *dataPtr;
    };
  }
}
#endif
