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
#ifndef _IGNITION_VECTOR4_HH_
#define _IGNITION_VECTOR4_HH_

#include <iostream>
#include <fstream>
#include "ignition/math/Matrix4.hh"

namespace ignition
{
  namespace math
  {
    /// \class Vector4 Vector4.hh ignitino/math.hh
    /// \brief double Generic x, y, z, w vector
    class Vector4
    {
      /// \brief Constructor
      public: Vector4();

      /// \brief Constructor with component values
      /// \param[in] _x value along x axis
      /// \param[in] _y value along y axis
      /// \param[in] _z value along z axis
      /// \param[in] _w value along w axis
      public: Vector4(const double &_x, const double &_y, const double &_z,
                      const double &_w);

      /// \brief Copy constructor
      /// \param[in] _v vector
      public: Vector4(const Vector4 &_v);


      /// \brief Destructor
      public: virtual ~Vector4();

      /// \brief Calc distance to the given point
      /// \param[in] _pt the point
      /// \return the distance
      public: double Distance(const Vector4 &_pt) const;

      /// \brief Returns the length (magnitude) of the vector
      public: double GetLength() const;

      /// \brief Return the square of the length (magnitude) of the vector
      /// \return the length
      public: double GetSquaredLength() const;

      /// \brief Normalize the vector length
      public: void Normalize();

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x axis
      /// \param[in] _y value along y axis
      /// \param[in] _z value along z axis
      /// \param[in] _w value along w axis
      public: void Set(double _x = 0, double _y = 0 , double _z = 0,
                       double _w = 0);

      /// \brief Assignment operator
      /// \param[in] _v the vector
      /// \return a reference to this vector
      public: Vector4 &operator =(const Vector4 &_v);

      /// \brief Assignment operator
      /// \param[in] _value
      public: Vector4 &operator =(double _value);

      /// \brief Addition operator
      /// \param[in] _v the vector to add
      /// \result a sum vector
      public: Vector4 operator+(const Vector4 &_v) const;

      /// \brief Addition operator
      /// \param[in] _v the vector to add
      /// \return this vector
      public: const Vector4 &operator+=(const Vector4 &_v);

      /// \brief Subtraction operator
      /// \param[in] _v the vector to substract
      /// \return a vector
      public: Vector4 operator-(const Vector4 &_v) const;

      /// \brief Subtraction assigment operators
      /// \param[in] _v the vector to substract
      /// \return this vector
      public: const Vector4 &operator-=(const Vector4 &_v);

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _v the vector to perform element wise division with
      /// \return a result vector
      public: const Vector4 operator/(const Vector4 &_v) const;

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _v the vector to perform element wise division with
      /// \return this
      public: const Vector4 &operator/=(const Vector4 &_v);

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _pt another vector
      /// \return a result vector
      public: const Vector4 operator/(double _v) const;

      /// \brief Division operator
      /// \param[in] _v scaling factor
      /// \return a vector
      public: const Vector4 &operator/=(double _v);

      /// \brief Multiplication operator.
      /// \remarks Performs element wise multiplication,
      /// which has limited use.
      /// \param[in] _pt another vector
      /// \return result vector
      public: const Vector4 operator*(const Vector4 &_pt) const;

      /// \brief Matrix multiplication operator.
      /// \param[in] _m matrix
      /// \return the vector multiplied by _m
      public: const Vector4 operator*(const Matrix4 &_m) const;

      /// \brief Multiplication assignment operator
      /// \remarks Performs element wise multiplication,
      /// which has limited use.
      /// \param[in] _pt a vector
      /// \return this
      public: const Vector4 &operator*=(const Vector4 &_pt);

      /// \brief Multiplication operators
      /// \param[in] _v scaling factor
      /// \return a  scaled vector
      public: const Vector4 operator*(double _v) const;

      /// \brief Multiplication assignment operator
      /// \param[in] _v scaling factor
      /// \return this
      public: const Vector4 &operator*=(double _v);

      /// \brief Equal to operator
      /// \param[in] _pt the other vector
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
      public: bool operator ==(const Vector4 &_pt) const;

      /// \brief Not equal to operator
      /// \param[in] _pt the other vector
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
      public: bool operator!=(const Vector4 &_pt) const;

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if finite, false otherwise
      public: bool IsFinite() const;

      /// \brief Array subscript operator
      /// \param[in] _index
      public: double operator[](unsigned int _index) const;

      /// X value
      public: double x;

      /// Y value
      public: double y;

      /// Z value
      public: double z;

      /// W value
      public: double w;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pt Vector4 to output
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::Vector4 &_pt)
      {
        _out << _pt.x << " " << _pt.y << " " << _pt.z << " " << _pt.w;
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector4 to read values into
      /// \return the stream
      public: friend std::istream &operator >> (std::istream &_in,
                  ignition::math::Vector4 &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.x >> _pt.y >> _pt.z >> _pt.w;
        return _in;
      }
    };
  }
}
#endif
