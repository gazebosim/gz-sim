/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_TRIANGLE3_HH_
#define IGNITION_MATH_TRIANGLE3_HH_

#include <ignition/math/Line3.hh>
#include <ignition/math/Plane.hh>
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

      /// \brief Constructor.
      ///
      /// Keep in mind that the triangle normal
      /// is determined by the order of these vertices. Search
      /// the internet for "triangle winding" for more information.
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
      ///
      /// Keep in mind that the triangle normal
      /// is determined by the order of these vertices. Search
      /// the internet for "triangle winding" for more information.
      ///
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
      ///
      /// Keep in mind that the triangle normal
      /// is determined by the order of these vertices. Search
      /// the internet for "triangle winding" for more information.
      ///
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
      /// 0 == Line3(pt1, pt2),
      /// 1 == Line3(pt2, pt3),
      /// 2 == Line3(pt3, pt1)
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
        // Make sure the point is on the same plane as the triangle
        if (Planed(this->Normal()).Side(_pt) == Planed::NO_SIDE)
        {
          Vector3d v0 = this->pts[2] - this->pts[0];
          Vector3d v1 = this->pts[1] - this->pts[0];
          Vector3d v2 = _pt - this->pts[0];

          double dot00 = v0.Dot(v0);
          double dot01 = v0.Dot(v1);
          double dot02 = v0.Dot(v2);
          double dot11 = v1.Dot(v1);
          double dot12 = v1.Dot(v2);

          // Compute barycentric coordinates
          double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
          double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
          double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

          // Check if point is in triangle
          return (u >= 0) && (v >= 0) && (u + v <= 1);
        }
        else
        {
          return false;
        }
      }

      /// \brief Get the triangle's normal vector.
      /// \return The normal vector for the triangle.
      public: Vector3d Normal() const
      {
         return Vector3d::Normal(this->pts[0], this->pts[1], this->pts[2]);
      }

      /// \brief Get whether the given line intersects an edge of this triangle.
      ///
      /// The returned intersection point is one of:
      ///
      /// * If the line is coplanar with the triangle:
      ///   * The point on the closest edge of the triangle that the line
      ///     intersects.
      ///   OR
      ///   * The first point on the line, if the line is completely contained
      /// * If the line is not coplanar, the point on the triangle that the
      ///   line intersects.
      ///
      /// \param[in] _line Line to check.
      /// \param[out] _ipt1 Return value of the first intersection point,
      /// only valid if the return value of the function is true.
      /// \return True if the given line intersects this triangle.
      public: bool Intersects(const Line3<T> &_line, Vector3<T> &_ipt1) const
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

    /// \brief Integer specialization of the Triangle class.
    typedef Triangle3<int> Triangle3i;

    /// \brief Double specialization of the Triangle class.
    typedef Triangle3<double> Triangle3d;

    /// \brief Float specialization of the Triangle class.
    typedef Triangle3<float> Triangle3f;
  }
}
#endif
