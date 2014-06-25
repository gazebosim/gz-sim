/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _IGNITION_PLANE_HH_
#define _IGNITION_PLANE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

namespace ignition
{
  namespace math
  {
    /// \class Plane Plane.hh ignition/math/Plane.hh
    /// \brief A plane and related functions.
    template<typename T>
    class Plane
    {
      /// \brief Constructor
      public: Plane()
      {
        this->d = 0.0;
      }

      /// \brief Constructor from a normal and a distanec
      /// \param[in] _normal The plane normal
      /// \param[in] _offset Offset along the normal
      public: Plane(const Vector3<T> &_normal, T _offset = 0.0)
      {
        this->normal = _normal;
        this->d = _offset;
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

      /// \brief Destructor
      public: virtual ~Plane() {}

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
      public: inline const Vector2<T> &GetSize() const
      {
        return this->size;
      }

      /// \brief Get the plane size
      public: inline Vector2<T> &GetSize()
      {
        return this->size;
      }

      /// \brief Get the plane offset
      public: inline const Vector3<T> &GetNormal() const
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline Vector3<T> &GetNormal()
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline T GetOffset() const
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

#endif
