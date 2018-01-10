/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_TRIANGLE_HH_
#define IGNITION_MATH_TRIANGLE_HH_

#include <set>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Triangle Triangle.hh ignition/math/Triangle.hh
    /// \brief Triangle class and related functions.
    template<typename T>
    class Triangle
    {
      /// \brief Default constructor
      public: Triangle() = default;

      /// \brief Constructor
      /// \param[in] _pt1 First point that defines the triangle.
      /// \param[in] _pt2 Second point that defines the triangle.
      /// \param[in] _pt3 Third point that defines the triangle.
      public: Triangle(const math::Vector2<T> &_pt1,
                       const math::Vector2<T> &_pt2,
                       const math::Vector2<T> &_pt3)
      {
        this->Set(_pt1, _pt2, _pt3);
      }

      /// \brief Set one vertex of the triangle.
      /// \param[in] _index Index of the point to set, where
      /// 0 == first vertex, 1 == second vertex, and 2 == third vertex.
      /// The index is clamped to the range [0, 2].
      /// \param[in] _pt Value of the point to set.
      public: void Set(const unsigned int _index, const math::Vector2<T> &_pt)
      {
        this->pts[clamp(_index, 0u, 2u)] = _pt;
      }

      /// \brief Set all vertices of the triangle.
      /// \param[in] _pt1 First point that defines the triangle.
      /// \param[in] _pt2 Second point that defines the triangle.
      /// \param[in] _pt3 Third point that defines the triangle.
      public: void Set(const math::Vector2<T> &_pt1,
                       const math::Vector2<T> &_pt2,
                       const math::Vector2<T> &_pt3)
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
      /// \param[in] _index Index of the side to retreive, where
      /// 0 == Line2(pt1, pt2),
      /// 1 == Line2(pt2, pt3),
      /// 2 == Line2(pt3, pt1)
      /// The index is clamped to the range [0, 2]
      /// \return Line segment of the requested side.
      public: Line2<T> Side(const unsigned int _index) const
      {
        if (_index == 0)
          return Line2<T>(this->pts[0], this->pts[1]);
        else if (_index == 1)
          return Line2<T>(this->pts[1], this->pts[2]);
        else
          return Line2<T>(this->pts[2], this->pts[0]);
      }

      /// \brief Check if this triangle completely contains the given line
      /// segment.
      /// \param[in] _line Line to check.
      /// \return True if the line's start and end points are both inside
      /// this triangle.
      public: bool Contains(const Line2<T> &_line) const
      {
        return this->Contains(_line[0]) && this->Contains(_line[1]);
      }

      /// \brief Get whether this triangle contains the given point.
      /// \param[in] _pt Point to check.
      /// \return True if the point is inside or on the triangle.
      public: bool Contains(const math::Vector2<T> &_pt) const
      {
        // Compute vectors
        math::Vector2<T> v0 = this->pts[2] -this->pts[0];
        math::Vector2<T> v1 = this->pts[1] -this->pts[0];
        math::Vector2<T> v2 = _pt - this->pts[0];

        // Compute dot products
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

      /// \brief Get whether the given line intersects this triangle.
      /// \param[in] _line Line to check.
      /// \param[out] _ipt1 Return value of the first intersection point,
      /// only valid if the return value of the function is true.
      /// \param[out] _ipt2 Return value of the second intersection point,
      /// only valid if the return value of the function is true.
      /// \return True if the given line intersects this triangle.
      public: bool Intersects(const Line2<T> &_line,
                              math::Vector2<T> &_ipt1,
                              math::Vector2<T> &_ipt2) const
      {
        if (this->Contains(_line))
        {
          _ipt1 = _line[0];
          _ipt2 = _line[1];
          return true;
        }

        Line2<T> line1(this->pts[0], this->pts[1]);
        Line2<T> line2(this->pts[1], this->pts[2]);
        Line2<T> line3(this->pts[2], this->pts[0]);

        math::Vector2<T> pt;
        std::set<math::Vector2<T> > points;

        if (line1.Intersect(_line, pt))
          points.insert(pt);

        if (line2.Intersect(_line, pt))
          points.insert(pt);

        if (line3.Intersect(_line, pt))
          points.insert(pt);

        if (points.empty())
        {
          return false;
        }
        else if (points.size() == 1)
        {
          auto iter = points.begin();

          _ipt1 = *iter;
          if (this->Contains(_line[0]))
            _ipt2 = _line[0];
          else
          {
            _ipt2 = _line[1];
          }
        }
        else
        {
          auto iter = points.begin();
          _ipt1 = *(iter++);
          _ipt2 = *iter;
        }

        return true;
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
      /// \param[in] _index The index, where 0 == first vertex,
      /// 1 == second vertex, and 2 == third vertex.
      /// The index is clamped to the range [0, 2]
      /// \return The point specified by _index.
      public: math::Vector2<T> operator[](const size_t _index) const
      {
        return this->pts[clamp(_index, IGN_ZERO_SIZE_T, IGN_TWO_SIZE_T)];
      }

      /// The points of the triangle
      private: math::Vector2<T> pts[3];
    };

    /// Integer specialization of the Triangle class.
    typedef Triangle<int> Trianglei;

    /// Double specialization of the Triangle class.
    typedef Triangle<double> Triangled;

    /// Float specialization of the Triangle class.
    typedef Triangle<float> Trianglef;
    }
  }
}
#endif
