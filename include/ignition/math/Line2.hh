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
#ifndef IGNITION_MATH_LINE2_HH_
#define IGNITION_MATH_LINE2_HH_

#include <algorithm>
#include <ignition/math/Vector2.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Line2 Line2.hh ignition/math/Line2.hh
    /// \brief A two dimensional line segment. The line is defined by a
    /// start and end point.
    template<typename T>
    class Line2
    {
      /// \brief Constructor.
      /// \param[in] _ptA Start point of the line segment
      /// \param[in] _ptB End point of the line segment
      public: Line2(const math::Vector2<T> &_ptA, const math::Vector2<T> &_ptB)
      {
        this->Set(_ptA, _ptB);
      }

      /// \brief Constructor.
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      public: Line2(double _x1, double _y1, double _x2, double _y2)
      {
        this->Set(_x1, _y1, _x2, _y2);
      }

      /// \brief Set the start and end point of the line segment
      /// \param[in] _ptA Start point of the line segment
      /// \param[in] _ptB End point of the line segment
      public: void Set(const math::Vector2<T> &_ptA,
                       const math::Vector2<T> &_ptB)
      {
        this->pts[0] = _ptA;
        this->pts[1] = _ptB;
      }

      /// \brief Set the start and end point of the line segment
      /// \param[in] _x1 X coordinate of the start point.
      /// \param[in] _y1 Y coordinate of the start point.
      /// \param[in] _x2 X coordinate of the end point.
      /// \param[in] _y2 Y coordinate of the end point.
      public: void Set(double _x1, double _y1, double _x2, double _y2)
      {
        this->pts[0].Set(_x1, _y1);
        this->pts[1].Set(_x2, _y2);
      }

      /// \brief Return the cross product of this line and the given line.
      /// Give 'a' as this line and 'b' as given line, the equation is:
      /// (a.start.x - a.end.x) * (b.start.y - b.end.y) -
      /// (a.start.y - a.end.y) * (b.start.x - b.end.x)
      /// \param[in] _line Line for the cross product computation.
      /// \return Return the cross product of this line and the given line.
      public: double CrossProduct(const Line2<T> &_line) const
      {
        return (this->pts[0].X() - this->pts[1].X()) *
               (_line[0].Y() -_line[1].Y()) -
               (this->pts[0].Y() - this->pts[1].Y()) *
               (_line[0].X() - _line[1].X());
      }

      /// \brief Return the cross product of this line and the given point.
      /// Given 'a' and 'b' as the start and end points, the equation is:
      //  (_pt.y - a.y) * (b.x - a.x) - (_pt.x - a.x) * (b.y - a.y)
      /// \param[in] _pt Point for the cross product computation.
      /// \return Return the cross product of this line and the given point.
      public: double CrossProduct(const Vector2<T> &_pt) const
      {
        return (_pt.Y() - this->pts[0].Y()) *
               (this->pts[1].X() - this->pts[0].X()) -
               (_pt.X() - this->pts[0].X()) *
               (this->pts[1].Y() - this->pts[0].Y());
      }

      /// \brief Check if the given point is collinear with this line.
      /// \param[in] _pt The point to check.
      /// \param[in] _epsilon The error bounds within which the collinear
      /// check will return true.
      /// \return Return true if the point is collinear with this line, false
      /// otherwise.
      public: bool Collinear(const math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const
      {
        return math::equal(this->CrossProduct(_pt),
            static_cast<T>(0), _epsilon);
      }

      /// \brief Check if the given line is parallel with this line.
      /// \param[in] _line The line to check.
      /// \param[in] _epsilon The error bounds within which the parallel
      /// check will return true.
      /// \return Return true if the line is parallel with this line, false
      /// otherwise. Return true if either line is a point (line with zero
      /// length).
      public: bool Parallel(const math::Line2<T> &_line,
                            double _epsilon = 1e-6) const
      {
        return math::equal(this->CrossProduct(_line),
            static_cast<T>(0), _epsilon);
      }

      /// \brief Check if the given line is collinear with this line. This
      /// is the AND of Parallel and Intersect.
      /// \param[in] _line The line to check.
      /// \param[in] _epsilon The error bounds within which the collinear
      /// check will return true.
      /// \return Return true if the line is collinear with this line, false
      /// otherwise.
      public: bool Collinear(const math::Line2<T> &_line,
                             double _epsilon = 1e-6) const
      {
        return this->Parallel(_line, _epsilon) &&
               this->Intersect(_line, _epsilon);
      }

      /// \brief Return whether the given point is on this line segment.
      /// \param[in] _pt Point to check.
      /// \param[in] _epsilon The error bounds within which the OnSegment
      /// check will return true.
      /// \return True if the point is on the segement.
      public: bool OnSegment(const math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const
      {
        return this->Collinear(_pt, _epsilon) && this->Within(_pt, _epsilon);
      }

      /// \brief Check if the given point is between the start and end
      /// points of the line segment. This does not imply that the point is
      /// on the segment.
      /// \param[in] _pt Point to check.
      /// \param[in] _epsilon The error bounds within which the within
      /// check will return true.
      /// \return True if the point is on the segement.
      public: bool Within(const math::Vector2<T> &_pt,
                          double _epsilon = 1e-6) const
      {
        return _pt.X() <= std::max(this->pts[0].X(),
                                   this->pts[1].X()) + _epsilon &&
               _pt.X() >= std::min(this->pts[0].X(),
                                   this->pts[1].X()) - _epsilon &&
               _pt.Y() <= std::max(this->pts[0].Y(),
                                   this->pts[1].Y()) + _epsilon &&
               _pt.Y() >= std::min(this->pts[0].Y(),
                                   this->pts[1].Y()) - _epsilon;
      }

      /// \brief Check if this line intersects the given line segment.
      /// \param[in] _line The line to check for intersection.
      /// \param[in] _epsilon The error bounds within which the intersection
      /// check will return true.
      /// \return True if an intersection was found.
      public: bool Intersect(const Line2<T> &_line,
                             double _epsilon = 1e-6) const
      {
        static math::Vector2<T> ignore;
        return this->Intersect(_line, ignore, _epsilon);
      }

      /// \brief Check if this line intersects the given line segment. The
      /// point of intersection is returned in the _result parameter.
      /// \param[in] _line The line to check for intersection.
      /// \param[out] _pt The point of intersection. This value is only
      /// valid if the return value is true.
      /// \param[in] _epsilon The error bounds within which the intersection
      /// check will return true.
      /// \return True if an intersection was found.
      public: bool Intersect(const Line2<T> &_line, math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const
      {
        double d = this->CrossProduct(_line);

        // d is zero if the two line are collinear. Must check special
        // cases.
        if (math::equal(d, 0.0, _epsilon))
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
          // Other wise return false.
          else
            return false;
        }

        _pt.X((_line[0].X() - _line[1].X()) *
              (this->pts[0].X() * this->pts[1].Y() -
               this->pts[0].Y() * this->pts[1].X()) -
              (this->pts[0].X() - this->pts[1].X()) *
              (_line[0].X() * _line[1].Y() - _line[0].Y() * _line[1].X()));

        _pt.Y((_line[0].Y() - _line[1].Y()) *
              (this->pts[0].X() * this->pts[1].Y() -
               this->pts[0].Y() * this->pts[1].X()) -
              (this->pts[0].Y() - this->pts[1].Y()) *
              (_line[0].X() * _line[1].Y() - _line[0].Y() * _line[1].X()));

        _pt /= d;

        if (_pt.X() < std::min(this->pts[0].X(), this->pts[1].X()) ||
            _pt.X() > std::max(this->pts[0].X(), this->pts[1].X()) ||
            _pt.X() < std::min(_line[0].X(), _line[1].X()) ||
            _pt.X() > std::max(_line[0].X(), _line[1].X()))
        {
          return false;
        }

        if (_pt.Y() < std::min(this->pts[0].Y(), this->pts[1].Y()) ||
            _pt.Y() > std::max(this->pts[0].Y(), this->pts[1].Y()) ||
            _pt.Y() < std::min(_line[0].Y(), _line[1].Y()) ||
            _pt.Y() > std::max(_line[0].Y(), _line[1].Y()))
        {
          return false;
        }

        return true;
      }

      /// \brief Get the length of the line
      /// \return The length of the line.
      public: T Length() const
      {
        return sqrt((this->pts[0].X() - this->pts[1].X()) *
                    (this->pts[0].X() - this->pts[1].X()) +
                    (this->pts[0].Y() - this->pts[1].Y()) *
                    (this->pts[0].Y() - this->pts[1].Y()));
      }

      /// \brief Get the slope of the line
      /// \return The slope of the line, NAN_D if the line is vertical.
      public: double Slope() const
      {
        if (math::equal(this->pts[1].X(), this->pts[0].X()))
          return NAN_D;

        return (this->pts[1].Y() - this->pts[0].Y()) /
               static_cast<double>(this->pts[1].X() - this->pts[0].X());
      }

      /// \brief Equality operator.
      /// \param[in] _line Line to compare for equality.
      /// \return True if the given line is equal to this line
      public: bool operator==(const Line2<T> &_line) const
      {
        return this->pts[0] == _line[0] && this->pts[1] == _line[1];
      }

      /// \brief Inequality operator.
      /// \param[in] _line Line to compare for inequality.
      /// \return True if the given line is not to this line
      public: bool operator!=(const Line2<T> &_line) const
      {
        return !(*this == _line);
      }

      /// \brief Get the start or end point.
      /// \param[in] _index 0 = start point, 1 = end point. The _index
      /// is clamped to the range [0, 1]
      public: math::Vector2<T> operator[](size_t _index) const
      {
        return this->pts[clamp(_index, IGN_ZERO_SIZE_T, IGN_ONE_SIZE_T)];
      }

      /// \brief Stream extraction operator
      /// \param[in] _out output stream
      /// \param[in] _pt Line2 to output
      /// \return The stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const Line2<T> &_line)
      {
        _out << _line[0] << " " << _line[1];
        return _out;
      }

      private: math::Vector2<T> pts[2];
    };


    typedef Line2<int> Line2i;
    typedef Line2<double> Line2d;
    typedef Line2<float> Line2f;
    }
  }
}
#endif
