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
#ifndef _IGNITION_VECTOR2_HH_
#define _IGNITION_VECTOR2_HH_

#include <ignition/math/IndexException.hh>

namespace ignition
{
  namespace math
  {
    /// \class Vector2 Vector2.hh ignition/math/Vector2.hh
    /// \brief Two dimensional (x, y) vector.
    template<typename T>
    class Vector2
    {
      /// \brief Default Constructor
      public: Vector2()
      {
        this->data[0] = 0;
        this->data[1] = 0;
      }

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: Vector2(const T &_x, const T &_y)
      {
        this->data[0] = _x;
        this->data[1] = _y;
      }

      /// \brief Copy constructor
      /// \param[in] _v the value
      public: Vector2(const Vector2<T> &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];
      }

      /// \brief Destructor
      public: virtual ~Vector2() {}

      /// \brief Calc distance to the given point
      /// \param[in] _pt The point to measure to
      /// \return the distance
      public: double Distance(const Vector2 &_pt) const
      {
        return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
                    (this->data[1]-_pt[1])*(this->data[1]-_pt[1]));
      }

      /// \brief  Normalize the vector length
      public: void Normalize()
      {
        double d = sqrt(this->data[0] * this->data[0] +
                        this->data[1] * this->data[1]);

        this->data[0] /= d;
        this->data[1] /= d;
      }

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: void Set(T _x, T _y)
      {
        this->data[0] = _x;
        this->data[1] = _y;
      }

      /// \brief Get the dot product of this vector and _v
      /// \param[in] _v the vector
      /// \return The dot product
      public: T Dot(const Vector2<T> &_v) const
      {
        return (this->data[0] * _v[0]) + (this->data[1] * _v[1]);
      }

      /// \brief Assignment operator
      /// \param[in] _v a value for x and y element
      /// \return this
      public: Vector2 &operator=(const Vector2 &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];

        return *this;
      }

      /// \brief Assignment operator
      /// \param[in] _v the value for x and y element
      /// \return this
      public: const Vector2 &operator=(T _v)
      {
        this->data[0] = _v;
        this->data[1] = _v;

        return *this;
      }

      /// \brief Addition operator
      /// \param[in] _v vector to add
      /// \return sum vector
      public: Vector2 operator+(const Vector2 &_v) const
      {
        return Vector2(this->data[0] + _v[0], this->data[1] + _v[1]);
      }

      /// \brief Addition assignment operator
      /// \param[in] _v the vector to add
      // \return this
      public: const Vector2 &operator+=(const Vector2 &_v)
      {
        this->data[0] += _v[0];
        this->data[1] += _v[1];

        return *this;
      }

      /// \brief Subtraction operator
      /// \param[in] _v the vector to substract
      /// \return the subtracted vector
      public: Vector2 operator-(const Vector2 &_v) const
      {
        return Vector2(this->data[0] - _v[0], this->data[1] - _v[1]);
      }

      /// \brief Subtraction assignment operator
      /// \param[in] _v the vector to substract
      /// \return this
      public: const Vector2 &operator-=(const Vector2 &_v)
      {
        this->data[0] -= _v[0];
        this->data[1] -= _v[1];

        return *this;
      }

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \result a result
      public: const Vector2 operator/(const Vector2 &_v) const
      {
        return Vector2(this->data[0] / _v[0], this->data[1] / _v[1]);
      }

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \param[in] _v a vector
      /// \return this
      public: const Vector2 &operator/=(const Vector2 &_v)
      {
        this->data[0] /= _v[0];
        this->data[1] /= _v[1];

        return *this;
      }

      /// \brief Division operator
      /// \param[in] _v the value
      /// \return a vector
      public: const Vector2 operator/(T _v) const
      {
        return Vector2(this->data[0] / _v, this->data[1] / _v);
      }

      /// \brief Division operator
      /// \param[in] _v the divisor
      /// \return a vector
      public: const Vector2 &operator/=(T _v)
      {
        this->data[0] /= _v;
        this->data[1] /= _v;

        return *this;
      }

      /// \brief Multiplication operators
      /// \param[in] _v the vector
      /// \return the result
      public: const Vector2 operator*(const Vector2 &_v) const
      {
        return Vector2(this->data[0] * _v[0], this->data[1] * _v[1]);
      }

      /// \brief Multiplication assignment operator
      /// \remarks this is an element wise multiplication
      /// \param[in] _v the vector
      /// \return this
      public: const Vector2 &operator*=(const Vector2 &_v)
      {
        this->data[0] *= _v[0];
        this->data[1] *= _v[1];

        return *this;
      }

      /// \brief Multiplication operators
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2 operator*(T _v) const
      {
        return Vector2(this->data[0] * _v, this->data[1] * _v);
      }

      /// \brief Multiplication assignment operator
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: const Vector2 &operator*=(T _v)
      {
        this->data[0] *= _v;
        this->data[1] *= _v;

        return *this;
      }

      /// \brief Equal to operator
      /// \param[in] _v the vector to compare to
      /// \return true if the elements of the 2 vectors are equal within
      /// a tolerence (1e-6)
      public: bool operator==(const Vector2 &_v) const
      {
        return equal(this->data[0], _v[0]) && equal(this->data[1], _v[1]);
      }

      /// \brief Not equal to operator
      /// \return true if elements are of diffent values (tolerence 1e-6)
      public: bool operator!=(const Vector2 &_v) const
      {
        return !(*this == _v);
      }

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if finite, false otherwise
      public: bool IsFinite() const
      {
        // std::isfinite works with floating point values,
        // need to explicit cast to avoid ambiguity in vc++.
        return std::isfinite(static_cast<double>(this->data[0])) &&
               std::isfinite(static_cast<double>(this->data[1]));
      }

      /// \brief Array subscript operator
      /// \param[in] _index the index
      /// \return the value. Throws an IndexException if _index is out of
      /// bounds.
      /// \throws IndexException if _index is >= 2.
      public: inline T operator[](size_t _index) const
      {
        if (_index > 1)
          throw IndexException();
        return this->data[_index];
      }

      /// \brief Return the x value.
      /// \return Value of the X component.
      /// \throws N/A.
      public: inline T X() const
      {
        return this->data[0];
      }

      /// \brief Return the y value.
      /// \return Value of the Y component.
      /// \throws N/A.
      public: inline T Y() const
      {
        return this->data[1];
      }

      /// \brief Return a mutable x value.
      /// \return Value of the X component.
      /// \throws N/A.
      public: inline T &X()
      {
        return this->data[0];
      }

      /// \brief Return a mutable y value.
      /// \return Value of the Y component.
      /// \throws N/A.
      public: inline T &Y()
      {
        return this->data[1];
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

      /// \brief Stream extraction operator
      /// \param[in] _out output stream
      /// \param[in] _pt Vector2 to output
      /// \return The stream
      /// \throws N/A.
      public: friend std::ostream
      &operator<<(std::ostream &_out, const Vector2<T> &_pt)
      {
        _out << _pt[0] << " " << _pt[1];
        return _out;
      }

      /// \brief Less than operator.
      /// \param[in] _pt Vector to compare.
      /// \return True if this vector2 first or second value is less than
      /// the given vector's first or second value.
      public: bool operator<(const Vector2<T> &_pt) const
      {
        return this->data[0] < _pt[0] || this->data[1] < _pt[1];
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector2 to read values into
      /// \return The stream
      /// \throws N/A.
      public: friend std::istream
      &operator>>(std::istream &_in, Vector2<T> &_pt)
      {
        T x, y;
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> x >> y;
        _pt.Set(x, y);
        return _in;
      }

      /// \brief The x and y values.
      private: T data[2];
    };

    typedef Vector2<int> Vector2i;
    typedef Vector2<double> Vector2d;
    typedef Vector2<float> Vector2f;
  }
}
#endif
