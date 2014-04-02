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
 * WITHOUdouble WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _IGNITION_VECTOR2D_HH_
#define _IGNITION_VECTOR2D_HH_

#include <math.h>
#include <iostream>
#include <fstream>

namespace ignition
{
  namespace math
  {
    /// \class Vector2d Vector2D.hh ignition/math.hh
    /// \brief Generic double x, y vector
    class Vector2d
    {
      /// \brief Constructor
      public: Vector2d();

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: Vector2d(const double &_x, const double &_y);

      /// \brief Copy constructor
      /// \param[in] _v the value
      public: Vector2d(const Vector2d &_v);

      /// \brief Destructor
      public: virtual ~Vector2d();

      /// \brief Calc distance to the given point
      /// \param[in] _pt The point to measure to
      /// \return the distance
      public: double Distance(const Vector2d &_pt) const;

      /// \brief  Normalize the vector length
      public: void Normalize();

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: void Set(double _x, double _y);

      /// \brief Return the cross product of this vector and _v
      /// \param[in] _v the vector
      /// \return the cross product
      public: Vector2d Cross(const Vector2d &_v) const;

      /// \brief Assignment operator
      /// \param[in] _v a value for x and y element
      /// \return this
      public: Vector2d &operator =(const Vector2d &_v);

      /// \brief Assignment operator
      /// \param[in] _v the value for x and y element
      /// \return this
      public: const Vector2d &operator =(double _v);

      /// \brief Addition operator
      /// \param[in] _v vector to add
      /// \return sum vector
      public: Vector2d operator+(const Vector2d &_v) const;

      /// \brief Addition assignment operator
      /// \param[in] _v the vector to add
      // \return this
      public: const Vector2d &operator+=(const Vector2d &_v);

      /// \brief Subtraction operator
      /// \param[in] _v the vector to substract
      /// \return the subtracted vector
      public: Vector2d operator-(const Vector2d &_v) const;

      /// \brief Subtraction assignment operator
      /// \param[in] _v the vector to substract
      /// \return this
      public: const Vector2d &operator-=(const Vector2d &_v);

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \result a result
      public: const Vector2d operator/(const Vector2d &_v) const;

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \return this
      public: const Vector2d &operator/=(const Vector2d &_v);

      /// \brief Division operator
      /// \param[in] _v the value
      /// \return a vector
      public: const Vector2d operator/(double _v) const;

      /// \brief Division operator
      /// \param[in] _v the divisor
      /// \return a vector
      public: const Vector2d &operator/=(double _v);

      /// \brief Multiplication operators
      /// \param[in] _v the vector
      /// \return the result
      public: const Vector2d operator*(const Vector2d &_v) const;

      /// \brief Multiplication assignment operator
      /// \remarks this is an element wise multiplication
      /// \param[in] _v the vector
      /// \return this
      public: const Vector2d &operator*=(const Vector2d &_v);

      /// \brief Multiplication operators
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2d operator*(double _v) const;

      /// \brief Multiplication assignment operator
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2d &operator*=(double _v);

      /// \brief Equal to operator
      /// \param[in] _v the vector to compare to
      /// \return true if the elements of the 2 vectors are equal within
      /// a tolerence (1e-6)
      public: bool operator ==(const Vector2d &_v) const;

      /// \brief Not equal to operator
      /// \return true if elements are of diffent values (tolerence 1e-6)
      public: bool operator!=(const Vector2d &_v) const;

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if finite, false otherwise
      public: bool IsFinite() const;

      /// \brief Array subscript operator
      /// \param[in] _index the index
      /// \return the value, or 0 if _index is out of bounds
      public: double operator[](unsigned int _index) const;

      /// \brief x data
      public: double x;

      /// \brief y data
      public: double y;

      /// \brief Stream extraction operator
      /// \param[in] _out output stream
      /// \param[in] _pt Vector2d to output
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::Vector2d &_pt)
      {
        _out << _pt.x << " " << _pt.y;
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector3 to read values into
      /// \return The stream
      public: friend std::istream &operator>>(std::istream &_in,
                  ignition::math::Vector2d &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.x >> _pt.y;
        return _in;
      }
    };
  }
}
#endif
