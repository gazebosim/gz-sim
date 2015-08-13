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

#ifndef _IGNITION_TRIANGLE3_HH_
#define _IGNITION_TRIANGLE3_HH_
#include <set>

#include <ignition/math/Line3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/IndexException.hh>

namespace ignition
{
  namespace math
  {
    /// \class Triangle3 Triangle3.hh ignition/math/Triangle3.hh
    /// \brief A 3-dimensional triangle and related functions.
    template<typename T>
    class Triangle3
    {
      /// \brief Default constructor
      public: Triangle3() = default;

      /// \brief Constructor
      /// \param[in] _pt1 First point that defines the triangle.
      /// \param[in] _pt2 Second point that defines the triangle.
      /// \param[in] _pt3 Third point that defines the triangle.
      public: Triangle3(const Vector3<T> &_pt1,
                        const Vector3<T> &_pt2,
                        const Vector3<T> &_pt3)
      {
        this->Set(_pt1, _pt2, _pt3);
      }

      /// \brief Set one vertex of the triangle.
      /// \param[in] _index Index of the point to set.
      /// \param[in] _pt Value of the point to set.
      /// \throws IndexException if _index is > 2.
      public: void Set(const unsigned int _index, const Vector3<T> &_pt)
      {
        if (_index > 2)
          throw IndexException();
        else
          this->pts[_index] = _pt;
      }

      /// \brief Set all vertices of the triangle.
      /// \param[in] _pt1 First point that defines the triangle.
      /// \param[in] _pt2 Second point that defines the triangle.
      /// \param[in] _pt3 Third point that defines the triangle.
      public: void Set(const Vector3<T> &_pt1,
                       const Vector3<T> &_pt2,
                       const Vector3<T> &_pt3)
      {
        this->pts[0] = _pt1;
        this->pts[1] = _pt2;
        this->pts[2] = _pt3;
      }

      /// \brief Get whether this triangle is valid, based on triangle
      /// inequality: the sum of the lengths of any two sides must be greater
      /// than the length of the remaining side.
      /// \return True if the triangle inequality holds
      public: bool Valid() const
      {
        T a = this->Side(0).Length();
        T b = this->Side(1).Length();
        T c = this->Side(2).Length();
        return (a+b) > c && (b+c) > a && (c+a) > b;
      }

      /// \brief Get a line segment for one side of the triangle.
      /// \param[in] _index Index of the side to retrieve, where
      /// 0 == Line2(pt1, pt2),
      /// 1 == Line2(pt2, pt3),
      /// 2 == Line2(pt3, pt1)
      /// \return Line segment of the requested side.
      /// \throws IndexException if _index is > 2.
      public: Line3<T> Side(const unsigned int _index) const
      {
        if (_index > 2)
          throw IndexException();
        else if (_index == 0)
          return Line3<T>(this->pts[0], this->pts[1]);
        else if (_index == 1)
          return Line3<T>(this->pts[1], this->pts[2]);
        else
          return Line3<T>(this->pts[2], this->pts[0]);
      }

      /// \brief Check if this triangle completely contains the given line
      /// segment.
      /// \param[in] _line Line to check.
      /// \return True if the line's start and end points are both inside
      /// this triangle.
      public: bool Contains(const Line3<T> &_line) const
      {
        return this->Contains(_line[0]) && this->Contains(_line[1]);
      }

      /// \brief Get whether this triangle contains the given point.
      /// \param[in] _pt Point to check.
      /// \return True if the point is inside or on the triangle.
      public: bool Contains(const Vector3<T> &_pt) const
      {
        // Prepare our barycentric variables
        Vector3<T> u = this->pts[1] - this->pts[0];
        Vector3<T> v = this->pts[2] - this->pts[0];
        Vector3<T> w = _pt - this->pts[0];

        Vector3<T> vCrossW = v.Cross(w);
        Vector3<T> vCrossU = v.Cross(u);

        // Test sign of r
        if (vCrossW.Dot(vCrossU) < 0)
          return false;

        Vector3<T> uCrossW = u.Cross(w);
        Vector3<T> uCrossV = u.Cross(v);

        // Test sign of t
        if (uCrossW.Dot(uCrossV) < 0)
          return false;

        // At this point, we know that r and t and both > 0.
        // Therefore, as long as their sum is <= 1, each must be less <= 1
        float denom = uCrossV.Length();
        float r = vCrossW.Length() / denom;
        float t = uCrossW.Length() / denom;

        return (r+t <= 1);
      }

      /// \brief Get the triangle's normal vector.
      /// \return The normal vector for the triangle.
      public: Vector3d Normal() const
      {
         return Vector3d::Normal(this->pts[0], this->pts[1], this->pts[2]);
      }

      /// \brief Get whether the given line intersects this triangle.
      /// \param[in] _line Line to check.
      /// \param[out] _ipt1 Return value of the first intersection point,
      /// only valid if the return value of the function is true.
      /// \return True if the given line intersects this triangle.
      public: bool Intersects(const Line3<T> &_line, Vector3<T> &_ipt1)
      {
        // Triangle normal
        Vector3d norm = this->Normal();

        // Ray direction to intersect with triangle
        Vector3d dir = (_line[1] - _line[0]).Normalize();

        double denom = norm.Dot(dir);

        // Handle the case when the line is not co-planar with the triangle
        if (!math::equal(denom, 0.0))
        {
          // Distance from line start to triangle intersection
          double intersection =
            -norm.Dot(_line[0] - this->pts[0]) / denom;

          // Make sure the ray intersects the triangle
          if (intersection < 1.0 || intersection > _line.Length())
            return false;

          // Return point of intersection
          _ipt1 = _line[0] + (dir * intersection);

          return true;
        }
        // Line co-planar with triangle
        else
        {
          // If the line is completely inside the triangle
          if (this->Contains(_line))
          {
            _ipt1 = _line[0];
            return true;
          }
          // If the line intersects the first side
          else if (_line.Intersect(this->Side(0), _ipt1))
          {
            return true;
          }
          // If the line intersects the second side
          else if (_line.Intersect(this->Side(1), _ipt1))
          {
            return true;
          }
          // If the line intersects the third side
          else if (_line.Intersect(this->Side(2), _ipt1))
          {
            return true;
          }
        }

        return false;
      }

      /// \brief Get the length of the triangle's perimeter.
      /// \return Sum of the triangle's line segments.
      public: T Perimeter() const
      {
        return this->Side(0).Length() + this->Side(1).Length() +
               this->Side(2).Length();
      }

      /// \brief Get the area of this triangle.
      /// \return Triangle's area.
      public: double Area() const
      {
        double s = this->Perimeter() / 2.0;
        T a = this->Side(0).Length();
        T b = this->Side(1).Length();
        T c = this->Side(2).Length();

        // Heron's formula
        // http://en.wikipedia.org/wiki/Heron%27s_formula
        return sqrt(s * (s-a) * (s-b) * (s-c));
      }

      /// \brief Get one of points that define the triangle.
      /// \param[in] _index: 0, 1, or 2.
      /// \throws IndexException if _index is > 2.
      public: Vector3<T> operator[](size_t _index) const
      {
        if (_index > 2)
          throw IndexException();
        return this->pts[_index];
      }

      /// The points of the triangle
      private: Vector3<T> pts[3];
    };

    /// Integer specialization of the Triangle class.
    typedef Triangle3<int> Triangle3i;

    /// Double specialization of the Triangle class.
    typedef Triangle3<double> Triangle3d;

    /// Float specialization of the Triangle class.
    typedef Triangle3<float> Triangle3f;
  }
}
#endif
