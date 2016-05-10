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
#ifndef IGNITION_MATH_LINE3_HH_
#define IGNITION_MATH_LINE3_HH_

#include <algorithm>
#include <ignition/math/Vector3.hh>
#include <ignition/math/IndexException.hh>

namespace ignition
{
  namespace math
  {
    /// \class Line3 Line3.hh ignition/math/Line3.hh
    /// \brief A three dimensional line segment. The line is defined by a
    /// start and end point.
    template<typename T>
    class Line3
    {
      /// \brief Line Constructor
      public: Line3() = default;

      /// \brief Copy constructor
      /// \param[in] _line a line object
      public: Line3(const Line3<T> &_line)
      {
        this->pts[0] = _line[0];
        this->pts[1] = _line[1];
      }

      /// \brief Constructor.
      /// \param[in] _ptA Start point of the line segment
      /// \param[in] _ptB End point of the line segment
      public: Line3(const math::Vector3<T> &_ptA, const math::Vector3<T> &_ptB)
      {
        this->Set(_ptA, _ptB);
      }

      /// \brief 2D Constructor where Z coordinates are 0
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      public: Line3(const double _x1, const double _y1,
        const double _x2, const double _y2)
      {
        this->Set(_x1, _y1, _x2, _y2);
      }

      /// \brief Constructor.
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _z1 Z coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      /// \param[in] _z2 Z coordinate of the end point.
      public: Line3(const double _x1, const double _y1,
        const double _z1, const double _x2,
        const double _y2, const double _z2)
      {
        this->Set(_x1, _y1, _z1, _x2, _y2, _z2);
      }

      /// \brief Set the start and end point of the line segment
      /// \param[in] _ptA Start point of the line segment
      /// \param[in] _ptB End point of the line segment
      public: void Set(const math::Vector3<T> &_ptA,
                       const math::Vector3<T> &_ptB)
      {
        this->pts[0] = _ptA;
        this->pts[1] = _ptB;
      }

      /// \brief Set the start point of the line segment
      /// \param[in] _ptA Start point of the line segment
      public: void SetA(const math::Vector3<T> &_ptA)
      {
        this->pts[0] = _ptA;
      }

      /// \brief Set the end point of the line segment
      /// \param[in] _ptB End point of the line segment
      public: void SetB(const math::Vector3<T> &_ptB)
      {
        this->pts[1] = _ptB;
      }

      /// \brief Set the start and end point of the line segment, assuming that
      /// both points have the same height.
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      /// \param[in] _z Z coordinate of both points,
      /// by default _z is set to 0.
      public: void Set(const double _x1, const double _y1,
        const double _x2, const double _y2,
        const double _z = 0)
      {
        this->pts[0].Set(_x1, _y1, _z);
        this->pts[1].Set(_x2, _y2, _z);
      }

      /// \brief Set the start and end point of the line segment
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _z1 Z coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      /// \param[in] _z2 Z coordinate of the end point.
      public: void Set(const double _x1, const double _y1,
        const double _z1, const double _x2,
        const double _y2, const double _z2)
      {
        this->pts[0].Set(_x1, _y1, _z1);
        this->pts[1].Set(_x2, _y2, _z2);
      }

      /// \brief Get the direction of the line
      /// \return The direction vector
      public: math::Vector3<T> Direction() const
      {
        return (this->pts[1] - this->pts[0]).Normalize();
      }

      /// \brief Get the length of the line
      /// \return The length of the line.
      public: T Length() const
      {
        return this->pts[0].Distance(this->pts[1]);
      }

      /// \brief Get the shortest line between this line and the
      /// provided line.
      ///
      /// In the case when the two lines are parallel, we choose the first
      /// point of this line and the closest point in the provided line.
      /// \param[in] _line Line to compare against this.
      /// \param[out] _result The shortest line between _line and this.
      /// \return True if a solution was found. False if a solution is not
      /// possible.
      public: bool Distance(const Line3<T> &_line, Line3<T> &_result,
                            const double _epsilon = 1e-6) const
      {
        Vector3<T> p13 = this->pts[0] - _line[0];
        Vector3<T> p43 = _line[1] - _line[0];

        if (std::abs(p43.X()) < _epsilon && std::abs(p43.Y()) < _epsilon &&
            std::abs(p43.Z()) < _epsilon)
        {
          return false;
        }

        Vector3<T> p21 = this->pts[1] - this->pts[0];

        if (std::abs(p21.X()) < _epsilon && std::abs(p21.Y()) < _epsilon &&
            std::abs(p21.Z()) < _epsilon)
        {
          return false;
        }

        double d1343 = p13.Dot(p43);
        double d4321 = p43.Dot(p21);
        double d1321 = p13.Dot(p21);
        double d4343 = p43.Dot(p43);
        double d2121 = p21.Dot(p21);

        double denom = d2121 * d4343 - d4321 * d4321;

        // In this case, we choose the first point in this line,
        // and the closest point in the provided line.
        if (std::abs(denom) < _epsilon)
        {
          double d1 = this->pts[0].Distance(_line[0]);
          double d2 = this->pts[0].Distance(_line[1]);

          double d3 = this->pts[1].Distance(_line[0]);
          double d4 = this->pts[1].Distance(_line[1]);

          if (d1 <= d2 && d1 <= d3 && d1 <= d4)
          {
            _result.SetA(this->pts[0]);
            _result.SetB(_line[0]);
          }
          else if (d2 <= d3 && d2 <= d4)
          {
            _result.SetA(this->pts[0]);
            _result.SetB(_line[1]);
          }
          else if (d3 <= d4)
          {
            _result.SetA(this->pts[1]);
            _result.SetB(_line[0]);
          }
          else
          {
            _result.SetA(this->pts[1]);
            _result.SetB(_line[1]);
          }

          return true;
        }

        double numer = d1343 * d4321 - d1321 * d4343;

        double mua = clamp(numer / denom, 0.0, 1.0);
        double mub = clamp((d1343 + d4321 * mua) / d4343, 0.0, 1.0);

        _result.Set(this->pts[0] + (p21 * mua), _line[0] + (p43 * mub));

        return true;
      }

      /// \brief Check if this line intersects the given line segment.
      /// \param[in] _line The line to check for intersection.
      /// \param[in] _epsilon The error bounds within which the intersection
      /// check will return true.
      /// \return True if an intersection was found.
      public: bool Intersect(const Line3<T> &_line,
                             double _epsilon = 1e-6) const
      {
        static math::Vector3<T> ignore;
        return this->Intersect(_line, ignore, _epsilon);
      }

      /// \brief Test if this line and the given line are coplanar.
      /// \param[in] _line Line to check against.
      /// \param[in] _epsilon The error bounds within which the
      /// check will return true.
      /// \return True if the two lines are coplanar.
      public: bool Coplanar(const Line3<T> &_line,
                            const double _epsilon = 1e-6) const
      {
        return std::abs((_line[0] - this->pts[0]).Dot(
              (this->pts[1] - this->pts[0]).Cross(_line[1] - _line[0])))
          <= _epsilon;
      }

      /// \brief Test if this line and the given line are parallel.
      /// \param[in] _line Line to check against.
      /// \param[in] _epsilon The error bounds within which the
      /// check will return true.
      /// \return True if the two lines are parallel.
      public: bool Parallel(const Line3<T> &_line,
                            const double _epsilon = 1e-6) const
      {
        return (this->pts[1] - this->pts[0]).Cross(
            _line[1] - _line[0]).Length() <= _epsilon;
      }

      /// \brief Check if this line intersects the given line segment. The
      /// point of intersection is returned in the _pt parameter.
      /// \param[in] _line The line to check for intersection.
      /// \param[out] _pt The point of intersection. This value is only
      /// valid if the return value is true.
      /// \param[in] _epsilon The error bounds within which the intersection
      /// check will return true.
      /// \return True if an intersection was found.
      public: bool Intersect(const Line3<T> &_line, math::Vector3<T> &_pt,
                             double _epsilon = 1e-6) const
      {
        // Handle special case when lines are parallel
        if (this->Parallel(_line, _epsilon))
        {
          // Check if _line's starting point is on the line.
          if (this->Within(_line[0], _epsilon))
          {
            _pt = _line[0];
            return true;
          }
          // Check if _line's ending point is on the line.
          else if (this->Within(_line[1], _epsilon))
          {
            _pt = _line[1];
            return true;
          }
          // Otherwise return false.
          else
            return false;
        }

        // Get the line that is the shortest distance between this and _line
        math::Line3<T> distLine;
        this->Distance(_line, distLine, _epsilon);

        // If the length of the line is less than epsilon, then they
        // intersect.
        if (distLine.Length() < _epsilon)
        {
          _pt = distLine[0];
          return true;
        }

        return false;
      }

      /// \brief Check if the given point is between the start and end
      /// points of the line segment.
      /// \param[in] _pt Point to check.
      /// \param[in] _epsilon The error bounds within which the within
      /// check will return true.
      /// \return True if the point is on the segement.
      public: bool Within(const math::Vector3<T> &_pt,
                          double _epsilon = 1e-6) const
      {
        return _pt.X() <= std::max(this->pts[0].X(),
                                   this->pts[1].X()) + _epsilon &&
               _pt.X() >= std::min(this->pts[0].X(),
                                   this->pts[1].X()) - _epsilon &&
               _pt.Y() <= std::max(this->pts[0].Y(),
                                   this->pts[1].Y()) + _epsilon &&
               _pt.Y() >= std::min(this->pts[0].Y(),
                                   this->pts[1].Y()) - _epsilon &&
               _pt.Z() <= std::max(this->pts[0].Z(),
                                   this->pts[1].Z()) + _epsilon &&
               _pt.Z() >= std::min(this->pts[0].Z(),
                                   this->pts[1].Z()) - _epsilon;
      }

      /// \brief Equality operator.
      /// \param[in] _line Line to compare for equality.
      /// \return True if the given line is equal to this line
      public: bool operator==(const Line3<T> &_line) const
      {
        return this->pts[0] == _line[0] && this->pts[1] == _line[1];
      }

      /// \brief Inequality operator.
      /// \param[in] _line Line to compare for inequality.
      /// \return True if the given line is not to this line
      public: bool operator!=(const Line3<T> &_line) const
      {
        return !(*this == _line);
      }

      /// \brief Get the start or end point.
      /// \param[in] _index 0 = start point, 1 = end point.
      /// \throws IndexException if _index is > 1.
      public: math::Vector3<T> operator[](const size_t _index) const
      {
        if (_index > 1)
          throw IndexException();
        return this->pts[_index];
      }

      /// \brief Stream extraction operator
      /// \param[in] _out output stream
      /// \param[in] _line Line3 to output
      /// \return The stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const Line3<T> &_line)
      {
        _out << _line[0] << " " << _line[1];
        return _out;
      }

      /// \brief Assignment operator
      /// \param[in] _line a new value
      /// \return this
      public: Line3 &operator=(const Line3<T> &_line)
      {
        this->pts[0] = _line[0];
        this->pts[1] = _line[1];

        return *this;
      }

      /// \brief Vector for storing the start and end points of the line
      private: math::Vector3<T> pts[2];
    };

    typedef Line3<int> Line3i;
    typedef Line3<double> Line3d;
    typedef Line3<float> Line3f;
  }
}
#endif
