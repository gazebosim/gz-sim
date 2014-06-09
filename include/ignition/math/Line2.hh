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
#ifndef _IGNITION_LINE2_HH_
#define _IGNITION_LINE2_HH_

#include <algorithm>
#include <ignition/math/Vector2.hh>
#include <ignition/math/IndexException.hh>

namespace ignition
{
  namespace math
  {
    /// \brief A two dimensional line segment. The line is defined by a
    /// start and end point.
    template<typename T>
    class IGNITION_VISIBLE Line2
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

      /// \brief Return whether the given point is on this line segment.
      /// \param[in] _pt Point to check.
      /// \return True if the point is on the segement.
      public: bool OnSegment(const math::Vector2<T> &_pt) const
      {
        return _pt.x() <= std::max(this->pts[0].x(), this->pts[1].x()) &&
               _pt.x() >= std::min(this->pts[0].x(), this->pts[1].x()) &&
               _pt.y() <= std::max(this->pts[0].y(), this->pts[1].y()) &&
               _pt.y() >= std::min(this->pts[0].y(), this->pts[1].y());
      }

      /// \brief Check if this line intersects the given line segment. The
      /// point of intersection is returned in the _result parameter.
      /// \param[in] _line The line to check for intersection.
      /// \param[out] _pt The point of intersection. This value is only
      /// valid if the return value is true.
      /// \return True if an intersection was found.
      public: bool Intersect(const Line2<T> &_line, math::Vector2<T> &_pt) const
      {
        double d = (this->pts[0].x() - this->pts[1].x()) *
                   (_line[0].y() -_line[1].y()) -
                   (this->pts[0].y() - this->pts[1].y()) *
                   (_line[0].x() - _line[1].x());

        // d is zero if the two line are co-linear. Must check special
        // cases.
        if (math::equal(d, 0.0))
        {
          // Check if _line's starting point is on the line.
          if (this->OnSegment(_line[0]))
          {
            _pt = _line[0];
            return true;
          }
          // Check if _line's ending point is on the line.
          else if (this->OnSegment(_line[1]))
          {
            _pt = _line[1];
            return true;
          }
          // Other wise return false.
          else
            return false;
        }

        _pt.x((_line[0].x() - _line[1].x()) *
              (this->pts[0].x() * this->pts[1].y() -
               this->pts[0].y() * this->pts[1].x()) -
              (this->pts[0].x() - this->pts[1].x()) *
              (_line[0].x() * _line[1].y() - _line[0].y() * _line[1].x()));

        _pt.y((_line[0].y() - _line[1].y()) *
              (this->pts[0].x() * this->pts[1].y() -
               this->pts[0].y() * this->pts[1].x()) -
              (this->pts[0].y() - this->pts[1].y()) *
              (_line[0].x() * _line[1].y() - _line[0].y() * _line[1].x()));

        _pt /= d;

        if (_pt.x() < std::min(this->pts[0].x(), this->pts[1].x()) ||
            _pt.x() > std::max(this->pts[0].x(), this->pts[1].x()) ||
            _pt.x() < std::min(_line[0].x(), _line[1].x()) ||
            _pt.x() > std::max(_line[0].x(), _line[1].x()))
        {
          return false;
        }

        if (_pt.y() < std::min(this->pts[0].y(), this->pts[1].y()) ||
            _pt.y() > std::max(this->pts[0].y(), this->pts[1].y()) ||
            _pt.y() < std::min(_line[0].y(), _line[1].y()) ||
            _pt.y() > std::max(_line[0].y(), _line[1].y()))
        {
          return false;
        }

        return true;
      }

      /// \brief Get the length of the line
      /// \return The length of the line.
      public: T Length() const
      {
        return sqrt((this->pts[0].x() - this->pts[1].x()) *
                    (this->pts[0].x() - this->pts[1].x()) +
                    (this->pts[0].y() - this->pts[1].y()) *
                    (this->pts[0].y() - this->pts[1].y()));
      }

      /// \brief Get the slope of the line
      /// \return The slope of the line, NAN_D if the line is vertical.
      public: double Slope() const
      {
        if (math::equal(this->pts[1].x(), this->pts[0].x()))
          return NAN_D;

        return (this->pts[1].y() - this->pts[0].y()) /
               static_cast<double>(this->pts[1].x() - this->pts[0].x());
      }

      /// \brief Get the start or end point.
      /// \param[in] _index 0 = start point, 1 = end point.
      /// \throws IndexException if _index is > 1.
      public: math::Vector2<T> operator[](size_t _index) const
      {
        if (_index > 1)
          throw IndexException();
        return this->pts[_index];
      }

      private: math::Vector2<T> pts[2];
    };

    typedef Line2<int> Line2i;
    typedef Line2<double> Line2d;
    typedef Line2<float> Line2f;
  }
}
#endif
