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
#ifndef IGNITION_MATH_VECTOR3_HH_
#define IGNITION_MATH_VECTOR3_HH_

#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>

#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Vector3 Vector3.hh ignition/math/Vector3.hh
    /// \brief The Vector3 class represents the generic vector containing 3
    /// elements.  Since it's commonly used to keep coordinate system
    /// related information, its elements are labeled by x, y, z.
    template<typename T>
    class Vector3
    {
      /// \brief math::Vector3(0, 0, 0)
      public: static const Vector3 Zero;

      /// \brief math::Vector3(1, 1, 1)
      public: static const Vector3 One;

      /// \brief math::Vector3(1, 0, 0)
      public: static const Vector3 UnitX;

      /// \brief math::Vector3(0, 1, 0)
      public: static const Vector3 UnitY;

      /// \brief math::Vector3(0, 0, 1)
      public: static const Vector3 UnitZ;

      /// \brief Constructor
      public: Vector3()
      {
        this->data[0] = 0;
        this->data[1] = 0;
        this->data[2] = 0;
      }

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value along z
      public: Vector3(const T &_x, const T &_y, const T &_z)
      {
        this->data[0] = _x;
        this->data[1] = _y;
        this->data[2] = _z;
      }

      /// \brief Copy constructor
      /// \param[in] _v a vector
      public: Vector3(const Vector3<T> &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];
        this->data[2] = _v[2];
      }

      /// \brief Destructor
      public: virtual ~Vector3() {}

      /// \brief Return the sum of the values
      /// \return the sum
      public: T Sum() const
      {
        return this->data[0] + this->data[1] + this->data[2];
      }

      /// \brief Calc distance to the given point
      /// \param[in] _pt the point
      /// \return the distance
      public: T Distance(const Vector3<T> &_pt) const
      {
        return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
                    (this->data[1]-_pt[1])*(this->data[1]-_pt[1]) +
                    (this->data[2]-_pt[2])*(this->data[2]-_pt[2]));
      }

      /// \brief Calc distance to the given point
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value along z
      /// \return the distance
      public: T Distance(T _x, T _y, T _z) const
      {
        return this->Distance(Vector3(_x, _y, _z));
      }

      /// \brief Returns the length (magnitude) of the vector
      /// \return the length
      public: T Length() const
      {
        return sqrt(this->SquaredLength());
      }

      /// \brief Return the square of the length (magnitude) of the vector
      /// \return the squared length
      public: T SquaredLength() const
      {
        return std::pow(this->data[0], 2)
             + std::pow(this->data[1], 2)
             + std::pow(this->data[2], 2);
      }

      /// \brief Normalize the vector length
      /// \return unit length vector
      public: Vector3 Normalize()
      {
        T d = this->Length();

        if (!equal<T>(d, static_cast<T>(0.0)))
        {
          this->data[0] /= d;
          this->data[1] /= d;
          this->data[2] /= d;
        }

        return *this;
      }

      /// \brief Return a normalized vector
      /// \return unit length vector
      public: Vector3 Normalized() const
      {
        Vector3<T> result = *this;
        result.Normalize();
        return result;
      }

      /// \brief Round to near whole number, return the result.
      /// \return the result
      public: Vector3 Round()
      {
        this->data[0] = nearbyint(this->data[0]);
        this->data[1] = nearbyint(this->data[1]);
        this->data[2] = nearbyint(this->data[2]);
        return *this;
      }

      /// \brief Get a rounded version of this vector
      /// \return a rounded vector
      public: Vector3 Rounded() const
      {
        Vector3<T> result = *this;
        result.Round();
        return result;
      }

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value aling z
      public: inline void Set(T _x = 0, T _y = 0, T _z = 0)
      {
        this->data[0] = _x;
        this->data[1] = _y;
        this->data[2] = _z;
      }

      /// \brief Return the cross product of this vector with another vector.
      /// \param[in] _v a vector
      /// \return the cross product
      public: Vector3 Cross(const Vector3<T> &_v) const
      {
        return Vector3(this->data[1] * _v[2] - this->data[2] * _v[1],
                       this->data[2] * _v[0] - this->data[0] * _v[2],
                       this->data[0] * _v[1] - this->data[1] * _v[0]);
      }

      /// \brief Return the dot product of this vector and another vector
      /// \param[in] _v the vector
      /// \return the dot product
      public: T Dot(const Vector3<T> &_v) const
      {
        return this->data[0] * _v[0] +
               this->data[1] * _v[1] +
               this->data[2] * _v[2];
      }

      /// \brief Return the absolute dot product of this vector and
      /// another vector. This is similar to the Dot function, except the
      /// absolute value of each component of the vector is used.
      ///
      /// result = abs(x1 * x2) + abs(y1 * y2) + abs(z1 *z2)
      ///
      /// \param[in] _v the vector
      /// \return The absolute dot product
      public: T AbsDot(const Vector3<T> &_v) const
      {
        return std::abs(this->data[0] * _v[0]) +
               std::abs(this->data[1] * _v[1]) +
               std::abs(this->data[2] * _v[2]);
      }

      /// \brief Get the absolute value of the vector
      /// \return a vector with positive elements
      public: Vector3 Abs() const
      {
        return Vector3(std::abs(this->data[0]),
                       std::abs(this->data[1]),
                       std::abs(this->data[2]));
      }

      /// \brief Return a vector that is perpendicular to this one.
      /// \return an orthogonal vector
      public: Vector3 Perpendicular() const
      {
        static const T sqrZero = 1e-06 * 1e-06;

        Vector3<T> perp = this->Cross(Vector3(1, 0, 0));

        // Check the length of the vector
        if (perp.SquaredLength() < sqrZero)
        {
          perp = this->Cross(Vector3(0, 1, 0));
        }

        return perp;
      }

      /// \brief Get a normal vector to a triangle
      /// \param[in] _v1 first vertex of the triangle
      /// \param[in] _v2 second vertex
      /// \param[in] _v3 third vertex
      /// \return the normal
      public: static Vector3 Normal(const Vector3<T> &_v1,
                  const Vector3<T> &_v2, const Vector3<T> &_v3)
      {
        Vector3<T> a = _v2 - _v1;
        Vector3<T> b = _v3 - _v1;
        Vector3<T> n = a.Cross(b);
        return n.Normalize();
      }

      /// \brief Get distance to a line
      /// \param[in] _pt1 first point on the line
      /// \param[in] _pt2 second point on the line
      /// \return the minimum distance from this point to the line
      public: T DistToLine(const Vector3<T> &_pt1, const Vector3 &_pt2)
      {
        T d = ((*this) - _pt1).Cross((*this) - _pt2).Length();
        d = d / (_pt2 - _pt1).Length();
        return d;
      }

      /// \brief Set this vector's components to the maximum of itself and the
      ///        passed in vector
      /// \param[in] _v the maximum clamping vector
      public: void Max(const Vector3<T> &_v)
      {
        if (_v[0] > this->data[0])
          this->data[0] = _v[0];
        if (_v[1] > this->data[1])
          this->data[1] = _v[1];
        if (_v[2] > this->data[2])
          this->data[2] = _v[2];
      }

      /// \brief Set this vector's components to the minimum of itself and the
      ///        passed in vector
      /// \param[in] _v the minimum clamping vector
      public: void Min(const Vector3<T> &_v)
      {
        if (_v[0] < this->data[0])
          this->data[0] = _v[0];
        if (_v[1] < this->data[1])
          this->data[1] = _v[1];
        if (_v[2] < this->data[2])
          this->data[2] = _v[2];
      }

      /// \brief Get the maximum value in the vector
      /// \return the maximum element
      public: T Max() const
      {
        return std::max(std::max(this->data[0], this->data[1]), this->data[2]);
      }

      /// \brief Get the minimum value in the vector
      /// \return the minimum element
      public: T Min() const
      {
        return std::min(std::min(this->data[0], this->data[1]), this->data[2]);
      }

      /// \brief Assignment operator
      /// \param[in] _v a new value
      /// \return this
      public: Vector3 &operator=(const Vector3<T> &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];
        this->data[2] = _v[2];

        return *this;
      }

      /// \brief Assignment operator
      /// \param[in] _value assigned to all elements
      /// \return this
      public: Vector3 &operator=(T _v)
      {
        this->data[0] = _v;
        this->data[1] = _v;
        this->data[2] = _v;

        return *this;
      }

      /// \brief Addition operator
      /// \param[in] _v vector to add
      /// \return the sum vector
      public: Vector3 operator+(const Vector3<T> &_v) const
      {
        return Vector3(this->data[0] + _v[0],
                       this->data[1] + _v[1],
                       this->data[2] + _v[2]);
      }

      /// \brief Addition assignment operator
      /// \param[in] _v vector to add
      /// \return the sum vector
      public: const Vector3 &operator+=(const Vector3<T> &_v)
      {
        this->data[0] += _v[0];
        this->data[1] += _v[1];
        this->data[2] += _v[2];

        return *this;
      }

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \return sum vector
      public: inline Vector3<T> operator+(const T _s) const
      {
        return Vector3<T>(this->data[0] + _s,
                          this->data[1] + _s,
                          this->data[2] + _s);
      }

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \param[in] _v input vector
      /// \return sum vector
      public: friend inline Vector3<T> operator+(const T _s,
                                                 const Vector3<T> &_v)
      {
        return {_v.X() + _s, _v.Y() + _s, _v.Z() + _s};
      }

      /// \brief Addition assignment operator
      /// \param[in] _s scalar addend
      /// \return this
      public: const Vector3<T> &operator+=(const T _s)
      {
        this->data[0] += _s;
        this->data[1] += _s;
        this->data[2] += _s;

        return *this;
      }

      /// \brief Negation operator
      /// \return negative of this vector
      public: inline Vector3 operator-() const
      {
        return Vector3(-this->data[0], -this->data[1], -this->data[2]);
      }

      /// \brief Subtraction operators
      /// \param[in] _pt a vector to substract
      /// \return a vector after the substraction
      public: inline Vector3<T> operator-(const Vector3<T> &_pt) const
      {
        return Vector3(this->data[0] - _pt[0],
                       this->data[1] - _pt[1],
                       this->data[2] - _pt[2]);
      }

      /// \brief Subtraction assignment operators
      /// \param[in] _pt subtrahend
      /// \return a vector after the substraction
      public: const Vector3<T> &operator-=(const Vector3<T> &_pt)
      {
        this->data[0] -= _pt[0];
        this->data[1] -= _pt[1];
        this->data[2] -= _pt[2];

        return *this;
      }

      /// \brief Subtraction operators
      /// \param[in] _s the scalar subtrahend
      /// \return difference vector
      public: inline Vector3<T> operator-(const T _s) const
      {
        return Vector3<T>(this->data[0] - _s,
                          this->data[1] - _s,
                          this->data[2] - _s);
      }

      /// \brief Subtraction operators
      /// \param[in] _s the scalar minuend
      /// \param[in] _v vector subtrahend
      /// \return difference vector
      public: friend inline Vector3<T> operator-(const T _s,
                                                 const Vector3<T> &_v)
      {
        return {_s - _v.X(), _s - _v.Y(), _s - _v.Z()};
      }

      /// \brief Subtraction assignment operator
      /// \param[in] _s scalar subtrahend
      /// \return this
      public: const Vector3<T> &operator-=(const T _s)
      {
        this->data[0] -= _s;
        this->data[1] -= _s;
        this->data[2] -= _s;

        return *this;
      }

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _pt the vector divisor
      /// \return a vector
      public: const Vector3<T> operator/(const Vector3<T> &_pt) const
      {
        return Vector3(this->data[0] / _pt[0],
                       this->data[1] / _pt[1],
                       this->data[2] / _pt[2]);
      }

      /// \brief Division assignment operator
      /// \remarks this is an element wise division
      /// \param[in] _pt the vector divisor
      /// \return a vector
      public: const Vector3<T> &operator/=(const Vector3<T> &_pt)
      {
        this->data[0] /= _pt[0];
        this->data[1] /= _pt[1];
        this->data[2] /= _pt[2];

        return *this;
      }

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v the divisor
      /// \return a vector
      public: const Vector3<T> operator/(T _v) const
      {
        return Vector3(this->data[0] / _v,
                       this->data[1] / _v,
                       this->data[2] / _v);
      }

      /// \brief Division assignment operator
      /// \remarks this is an element wise division
      /// \param[in] _v the divisor
      /// \return this
      public: const Vector3<T> &operator/=(T _v)
      {
        this->data[0] /= _v;
        this->data[1] /= _v;
        this->data[2] /= _v;

        return *this;
      }

      /// \brief Multiplication operator
      /// \remarks this is an element wise multiplication, not a cross product
      /// \param[in] _p multiplier operator
      /// \return a vector
      public: Vector3<T> operator*(const Vector3<T> &_p) const
      {
        return Vector3(this->data[0] * _p[0],
                       this->data[1] * _p[1],
                       this->data[2] * _p[2]);
      }

      /// \brief Multiplication assignment operators
      /// \remarks this is an element wise multiplication, not a cross product
      /// \param[in] _v a vector
      /// \return this
      public: const Vector3<T> &operator*=(const Vector3<T> &_v)
      {
        this->data[0] *= _v[0];
        this->data[1] *= _v[1];
        this->data[2] *= _v[2];

        return *this;
      }

      /// \brief Multiplication operators
      /// \param[in] _s the scaling factor
      /// \return a scaled vector
      public: inline Vector3<T> operator*(T _s) const
      {
        return Vector3<T>(this->data[0] * _s,
                          this->data[1] * _s,
                          this->data[2] * _s);
      }

      /// \brief Multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _v input vector
      /// \return a scaled vector
      public: friend inline Vector3<T> operator*(T _s, const Vector3<T> &_v)
      {
        return {_v.X() * _s, _v.Y() * _s, _v.Z() * _s};
      }

      /// \brief Multiplication operator
      /// \param[in] _v scaling factor
      /// \return this
      public: const Vector3<T> &operator*=(T _v)
      {
        this->data[0] *= _v;
        this->data[1] *= _v;
        this->data[2] *= _v;

        return *this;
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _v the vector to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the vectors are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const Vector3 &_v, const T &_tol) const
      {
        return equal<T>(this->data[0], _v[0], _tol)
            && equal<T>(this->data[1], _v[1], _tol)
            && equal<T>(this->data[2], _v[2], _tol);
      }

      /// \brief Equal to operator
      /// \param[in] _v The vector to compare against
      /// \return true if each component is equal within a
      /// default tolerence (1e-3), false otherwise
      public: bool operator==(const Vector3<T> &_v) const
      {
        return this->Equal(_v, static_cast<T>(1e-3));
      }

      /// \brief Not equal to operator
      /// \param[in] _v The vector to compare against
      /// \return false if each component is equal within a
      /// default tolerence (1e-3), true otherwise
      public: bool operator!=(const Vector3<T> &_v) const
      {
        return !(*this == _v);
      }

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if is finite or false otherwise
      public: bool IsFinite() const
      {
        // std::isfinite works with floating point values,
        // need to explicit cast to avoid ambiguity in vc++.
        return std::isfinite(static_cast<double>(this->data[0])) &&
               std::isfinite(static_cast<double>(this->data[1])) &&
               std::isfinite(static_cast<double>(this->data[2]));
      }

      /// \brief Corrects any nan values
      public: inline void Correct()
      {
        // std::isfinite works with floating point values,
        // need to explicit cast to avoid ambiguity in vc++.
        if (!std::isfinite(static_cast<double>(this->data[0])))
          this->data[0] = 0;
        if (!std::isfinite(static_cast<double>(this->data[1])))
          this->data[1] = 0;
        if (!std::isfinite(static_cast<double>(this->data[2])))
          this->data[2] = 0;
      }

      /// \brief Array subscript operator
      /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z.
      /// The index is clamped to the range [0,2].
      /// \return The value.
      public: T &operator[](const std::size_t _index)
      {
        return this->data[clamp(_index, IGN_ZERO_SIZE_T, IGN_TWO_SIZE_T)];
      }

      /// \brief Const-qualified array subscript operator
      /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z.
      /// The index is clamped to the range [0,2].
      /// \return The value.
      public: T operator[](const std::size_t _index) const
      {
        return this->data[clamp(_index, IGN_ZERO_SIZE_T, IGN_TWO_SIZE_T)];
      }

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision the decimal places
      public: void Round(int _precision)
      {
        this->data[0] = precision(this->data[0], _precision);
        this->data[1] = precision(this->data[1], _precision);
        this->data[2] = precision(this->data[2], _precision);
      }

      /// \brief Equality test
      /// \remarks This is equivalent to the == operator
      /// \param[in] _v the other vector
      /// \return true if the 2 vectors have the same values, false otherwise
      public: bool Equal(const Vector3<T> &_v) const
      {
        return equal<T>(this->data[0], _v[0]) &&
               equal<T>(this->data[1], _v[1]) &&
               equal<T>(this->data[2], _v[2]);
      }

      /// \brief Get the x value.
      /// \return The x component of the vector
      public: inline T X() const
      {
        return this->data[0];
      }

      /// \brief Get the y value.
      /// \return The y component of the vector
      public: inline T Y() const
      {
        return this->data[1];
      }

      /// \brief Get the z value.
      /// \return The z component of the vector
      public: inline T Z() const
      {
        return this->data[2];
      }

      /// \brief Get a mutable reference to the x value.
      /// \return The x component of the vector
      public: inline T &X()
      {
        return this->data[0];
      }

      /// \brief Get a mutable reference to the y value.
      /// \return The y component of the vector
      public: inline T &Y()
      {
        return this->data[1];
      }

      /// \brief Get a mutable reference to the z value.
      /// \return The z component of the vector
      public: inline T &Z()
      {
        return this->data[2];
      }

      /// \brief Set the x value.
      /// \param[in] _v Value for the x component.
      public: inline void X(const T &_v)
      {
        this->data[0] = _v;
      }

      /// \brief Set the y value.
      /// \param[in] _v Value for the y component.
      public: inline void Y(const T &_v)
      {
        this->data[1] = _v;
      }

      /// \brief Set the z value.
      /// \param[in] _v Value for the z component.
      public: inline void Z(const T &_v)
      {
        this->data[2] = _v;
      }

      /// \brief Less than operator.
      /// \param[in] _pt Vector to compare.
      /// \return True if this vector's X(), Y(), or Z() value is less
      /// than the given vector's corresponding values.
      public: bool operator<(const Vector3<T> &_pt) const
      {
        return this->data[0] < _pt[0] || this->data[1] < _pt[1] ||
               this->data[2] < _pt[2];
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _pt Vector3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Vector3<T> &_pt)
      {
        _out << precision(_pt[0], 6) << " " << precision(_pt[1], 6) << " "
          << precision(_pt[2], 6);
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param _in input stream
      /// \param _pt vector3 to read values into
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, ignition::math::Vector3<T> &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        T x, y, z;
        _in >> x >> y >> z;
        _pt.Set(x, y, z);
        return _in;
      }

      /// \brief The x, y, and z values
      private: T data[3];
    };

    template<typename T> const Vector3<T> Vector3<T>::Zero(0, 0, 0);
    template<typename T> const Vector3<T> Vector3<T>::One(1, 1, 1);
    template<typename T> const Vector3<T> Vector3<T>::UnitX(1, 0, 0);
    template<typename T> const Vector3<T> Vector3<T>::UnitY(0, 1, 0);
    template<typename T> const Vector3<T> Vector3<T>::UnitZ(0, 0, 1);

    typedef Vector3<int> Vector3i;
    typedef Vector3<double> Vector3d;
    typedef Vector3<float> Vector3f;
    }
  }
}
#endif
