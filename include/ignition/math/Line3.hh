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
      /// \param[in] _index 0 = start point, 1 = end point. The _index
      /// parameter is clamped to the range (0, 1).
      public: math::Vector3<T> operator[](const size_t _index) const
      {
        return this->pts[_index <= 1 ? _index : 1];
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
