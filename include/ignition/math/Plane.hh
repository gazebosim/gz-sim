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
#ifndef IGNITION_MATH_PLANE_HH_
#define IGNITION_MATH_PLANE_HH_

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Plane Plane.hh ignition/math/Plane.hh
    /// \brief A plane and related functions.
    template<typename T>
    class Plane
    {
      /// \brief Enum used to indicate a side of the plane, no side, or both
      /// sides for entities on the plane.
      /// \sa Side
      public: enum PlaneSide
      {
        /// \brief Negative side of the plane. This is the side that is
        /// opposite the normal.
        NEGATIVE_SIDE = 0,

        /// \brief Positive side of the plane. This is the side that has the
        /// normal vector.
        POSITIVE_SIDE = 1,

        /// \brief On the plane.
        NO_SIDE = 2,

        /// \brief On both sides of the plane.
        BOTH_SIDE = 3
      };

      /// \brief Constructor
      public: Plane()
      : d(0.0)
      {
      }

      /// \brief Constructor from a normal and a distance
      /// \param[in] _normal The plane normal
      /// \param[in] _offset Offset along the normal
      public: Plane(const Vector3<T> &_normal, T _offset = 0.0)
      : normal(_normal), d(_offset)
      {
      }

      /// \brief Constructor
      /// \param[in] _normal The plane normal
      /// \param[in] _size Size of the plane
      /// \param[in] _offset Offset along the normal
      public: Plane(const Vector3<T> &_normal, const Vector2<T> &_size,
                    T _offset)
      {
        this->Set(_normal, _size, _offset);
      }

      /// \brief Copy constructor
      /// \param[in] _plane Plane to copy
      public: Plane(const Plane &_plane)
      : normal(_plane.normal), size(_plane.size), d(_plane.d)
      {}

      /// \brief Destructor
      public: virtual ~Plane() {}

      /// \brief Set the plane
      /// \param[in] _normal The plane normal
      /// \param[in] _offset Offset along the normal
      public: void Set(const Vector3<T> &_normal, T _offset)
      {
        this->normal = _normal;
        this->d = _offset;
      }

      /// \brief Set the plane
      /// \param[in] _normal The plane normal
      /// \param[in] _size Size of the plane
      /// \param[in] _offset Offset along the normal
      public: void Set(const Vector3<T> &_normal, const Vector2<T> &_size,
                       T _offset)
      {
        this->normal = _normal;
        this->size = _size;
        this->d = _offset;
      }

      /// \brief The distance to the plane from the given point. The
      /// distance can be negative, which indicates the point is on the
      /// negative side of the plane.
      /// \param[in] _point 3D point to calculate distance from.
      /// \return Distance from the point to the plane.
      /// \sa Side
      public: T Distance(const Vector3<T> &_point) const
      {
        return this->normal.Dot(_point) - this->d;
      }

      /// \brief The side of the plane a point is on.
      /// \param[in] _point The 3D point to check.
      /// \return Plane::NEGATIVE_SIDE if the distance from the point to the
      /// plane is negative, Plane::POSITIVE_SIDE if the distance from the
      /// point to the plane is positive, or Plane::NO_SIDE if the
      /// point is on the plane.
      public: PlaneSide Side(const Vector3<T> &_point) const
      {
        T dist = this->Distance(_point);

        if (dist < 0.0)
          return NEGATIVE_SIDE;

        if (dist > 0.0)
          return POSITIVE_SIDE;

        return NO_SIDE;
      }

      /// \brief The side of the plane a box is on.
      /// \param[in] _box The 3D box to check.
      /// \return Plane::NEGATIVE_SIDE if the distance from the box to the
      /// plane is negative, Plane::POSITIVE_SIDE if the distance from the
      /// box to the plane is positive, or Plane::BOTH_SIDE if the
      /// box is on the plane.
      public: PlaneSide Side(const math::AxisAlignedBox &_box) const
      {
        double dist = this->Distance(_box.Center());
        double maxAbsDist = this->normal.AbsDot(_box.Size()/2.0);

        if (dist < -maxAbsDist)
          return NEGATIVE_SIDE;

        if (dist > maxAbsDist)
          return POSITIVE_SIDE;

        return BOTH_SIDE;
      }

      /// \brief Get distance to the plane give an origin and direction
      /// \param[in] _origin the origin
      /// \param[in] _dir a direction
      /// \return the shortest distance
      public: T Distance(const Vector3<T> &_origin,
                         const Vector3<T> &_dir) const
      {
        T denom = this->normal.Dot(_dir);

        if (std::abs(denom) < 1e-3)
        {
          // parallel
          return 0;
        }
        else
        {
          T nom = _origin.Dot(this->normal) - this->d;
          T t = -(nom/denom);
          return t;
        }
      }

      /// \brief Get the plane size
      public: inline const Vector2<T> &Size() const
      {
        return this->size;
      }

      /// \brief Get the plane size
      public: inline Vector2<T> &Size()
      {
        return this->size;
      }

      /// \brief Get the plane offset
      public: inline const Vector3<T> &Normal() const
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline Vector3<T> &Normal()
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline T Offset() const
      {
        return this->d;
      }

      /// \brief Equal operator
      /// \param _p another plane
      /// \return itself
      public: Plane<T> &operator=(const Plane<T> &_p)
      {
        this->normal = _p.normal;
        this->size = _p.size;
        this->d = _p.d;

        return *this;
      }

      /// \brief Plane normal
      private: Vector3<T> normal;

      /// \brief Plane size
      private: Vector2<T> size;

      /// \brief Plane offset
      private: T d;
    };

    typedef Plane<int> Planei;
    typedef Plane<double> Planed;
    typedef Plane<float> Planef;
    }
  }
}

#endif
