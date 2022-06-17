/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GZ_MATH_VECTOR2_HH_
#define GZ_MATH_VECTOR2_HH_

#include <algorithm>
#include <cmath>
#include <istream>
#include <limits>
#include <ostream>

#include <gz/math/Helpers.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Vector2 Vector2.hh gz/math/Vector2.hh
    /// \brief Two dimensional (x, y) vector.
    template<typename T>
    class Vector2
    {
      /// \brief math::Vector2(0, 0)
      public: static const Vector2<T> &Zero;

      /// \brief math::Vector2(1, 1)
      public: static const Vector2<T> &One;

      /// \brief math::Vector2(NaN, NaN, NaN)
      public: static const Vector2 &NaN;

      /// \brief Default Constructor
      public: constexpr Vector2()
      : data{0, 0}
      {
      }

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      public: constexpr Vector2(const T &_x, const T &_y)
      : data{_x, _y}
      {
      }

      /// \brief Copy constructor
      /// \param[in] _v the value
      public: Vector2(const Vector2<T> &_v) = default;

      /// \brief Destructor
      public: ~Vector2() = default;

      /// \brief Return the sum of the values
      /// \return the sum
      public: T Sum() const
      {
        return this->data[0] + this->data[1];
      }

      /// \brief Calc distance to the given point
      /// \param[in] _pt The point to measure to
      /// \return the distance
      public: double Distance(const Vector2 &_pt) const
      {
        return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
                    (this->data[1]-_pt[1])*(this->data[1]-_pt[1]));
      }

      /// \brief Returns the length (magnitude) of the vector
      /// \return The length
      public: T Length() const
      {
        return static_cast<T>(sqrt(this->SquaredLength()));
      }

      /// \brief Returns the square of the length (magnitude) of the vector
      /// \return The squared length
      public: T SquaredLength() const
      {
        return
          this->data[0] * this->data[0] +
          this->data[1] * this->data[1];
      }

      /// \brief Normalize the vector length
      public: void Normalize()
      {
        T d = this->Length();

        if (!equal<T>(d, static_cast<T>(0.0)))
        {
          this->data[0] /= d;
          this->data[1] /= d;
        }
      }

      /// \brief Returns a normalized vector
      /// \return unit length vector
      public: Vector2 Normalized() const
      {
        Vector2<T> result = *this;
        result.Normalize();
        return result;
      }

      /// \brief Round to near whole number, return the result.
      /// \return the result
      public: Vector2 Round()
      {
        this->data[0] = static_cast<T>(std::nearbyint(this->data[0]));
        this->data[1] = static_cast<T>(std::nearbyint(this->data[1]));
        return *this;
      }

      /// \brief Get a rounded version of this vector
      /// \return a rounded vector
      public: Vector2 Rounded() const
      {
        Vector2<T> result = *this;
        result.Round();
        return result;
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

      /// \brief Get the absolute value of the vector
      /// \return a vector with positive elements
      public: Vector2 Abs() const
      {
        return Vector2(std::abs(this->data[0]),
                       std::abs(this->data[1]));
      }

      /// \brief Return the absolute dot product of this vector and
      /// another vector. This is similar to the Dot function, except the
      /// absolute value of each component of the vector is used.
      ///
      /// result = abs(x1 * x2) + abs(y1 * y2)
      ///
      /// \param[in] _v The vector
      /// \return The absolute dot product
      public: T AbsDot(const Vector2<T> &_v) const
      {
        return std::abs(this->data[0] * _v[0]) +
               std::abs(this->data[1] * _v[1]);
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
      }

      /// \brief Set this vector's components to the maximum of itself and the
      ///        passed in vector
      /// \param[in] _v the maximum clamping vector
      public: void Max(const Vector2<T> &_v)
      {
        this->data[0] = std::max(_v[0], this->data[0]);
        this->data[1] = std::max(_v[1], this->data[1]);
      }

      /// \brief Set this vector's components to the minimum of itself and the
      ///        passed in vector
      /// \param[in] _v the minimum clamping vector
      public: void Min(const Vector2<T> &_v)
      {
        this->data[0] = std::min(_v[0], this->data[0]);
        this->data[1] = std::min(_v[1], this->data[1]);
      }

      /// \brief Get the maximum value in the vector
      /// \return the maximum element
      public: T Max() const
      {
        return std::max(this->data[0], this->data[1]);
      }

      /// \brief Get the minimum value in the vector
      /// \return the minimum element
      public: T Min() const
      {
        return std::min(this->data[0], this->data[1]);
      }

      /// \brief Assignment operator
      /// \param[in] _v a value for x and y element
      /// \return this
      public: Vector2 &operator=(const Vector2 &_v) = default;

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

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \return sum vector
      public: inline Vector2<T> operator+(const T _s) const
      {
        return Vector2<T>(this->data[0] + _s,
                          this->data[1] + _s);
      }

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \param[in] _v input vector
      /// \return sum vector
      public: friend inline Vector2<T> operator+(const T _s,
                                                 const Vector2<T> &_v)
      {
        return _v + _s;
      }

      /// \brief Addition assignment operator
      /// \param[in] _s scalar addend
      /// \return this
      public: const Vector2<T> &operator+=(const T _s)
      {
        this->data[0] += _s;
        this->data[1] += _s;

        return *this;
      }

      /// \brief Negation operator
      /// \return negative of this vector
      public: inline Vector2 operator-() const
      {
        return Vector2(-this->data[0], -this->data[1]);
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

      /// \brief Subtraction operators
      /// \param[in] _s the scalar subtrahend
      /// \return difference vector
      public: inline Vector2<T> operator-(const T _s) const
      {
        return Vector2<T>(this->data[0] - _s,
                          this->data[1] - _s);
      }

      /// \brief Subtraction operators
      /// \param[in] _s the scalar minuend
      /// \param[in] _v vector subtrahend
      /// \return difference vector
      public: friend inline Vector2<T> operator-(const T _s,
                                                 const Vector2<T> &_v)
      {
        return {_s - _v.X(), _s - _v.Y()};
      }

      /// \brief Subtraction assignment operator
      /// \param[in] _s scalar subtrahend
      /// \return this
      public: const Vector2<T> &operator-=(T _s)
      {
        this->data[0] -= _s;
        this->data[1] -= _s;

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

      /// \brief Scalar left multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _v the vector to scale
      /// \return a scaled vector
      public: friend inline const Vector2 operator*(const T _s,
                                                    const Vector2 &_v)
      {
        return Vector2(_v * _s);
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

      /// \brief Equality test with tolerance.
      /// \param[in] _v the vector to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the vectors are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const Vector2 &_v, const T &_tol) const
      {
        return equal<T>(this->data[0], _v[0], _tol)
            && equal<T>(this->data[1], _v[1], _tol);
      }

      /// \brief Equal to operator
      /// \param[in] _v the vector to compare to
      /// \return true if the elements of the 2 vectors are equal within
      /// a tolerence (1e-6)
      public: bool operator==(const Vector2 &_v) const
      {
        return this->Equal(_v, static_cast<T>(1e-6));
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
      /// \param[in] _index The index, where 0 == x and 1 == y.
      /// The index is clamped to the range [0,1].
      public: T &operator[](const std::size_t _index)
      {
        return this->data[clamp(_index, GZ_ZERO_SIZE_T, GZ_ONE_SIZE_T)];
      }

      /// \brief Const-qualified array subscript operator
      /// \param[in] _index The index, where 0 == x and 1 == y.
      /// The index is clamped to the range [0,1].
      public: T operator[](const std::size_t _index) const
      {
        return this->data[clamp(_index, GZ_ZERO_SIZE_T, GZ_ONE_SIZE_T)];
      }

      /// \brief Return the x value.
      /// \return Value of the X component.
      public: inline T X() const
      {
        return this->data[0];
      }

      /// \brief Return the y value.
      /// \return Value of the Y component.
      public: inline T Y() const
      {
        return this->data[1];
      }

      /// \brief Return a mutable x value.
      /// \return Value of the X component.
      public: inline T &X()
      {
        return this->data[0];
      }

      /// \brief Return a mutable y value.
      /// \return Value of the Y component.
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
      /// \param[out] _out output stream
      /// \param[in] _pt Vector2 to output
      /// \return The stream
      public: friend std::ostream
      &operator<<(std::ostream &_out, const Vector2<T> &_pt)
      {
        for (auto i : {0, 1})
        {
          if (i > 0)
            _out << " ";

          appendToStream(_out, _pt[i]);
        }
        return _out;
      }

      /// \brief Less than operator.
      /// \param[in] _pt Vector to compare.
      /// \return True if this vector's first or second value is less than
      /// the given vector's first or second value.
      public: bool operator<(const Vector2<T> &_pt) const
      {
        return this->data[0] < _pt[0] || this->data[1] < _pt[1];
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector2 to read values into
      /// \return The stream
      public: friend std::istream
      &operator>>(std::istream &_in, Vector2<T> &_pt)
      {
        T x, y;
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> x >> y;
        if (!_in.fail())
        {
          _pt.Set(x, y);
        }
        return _in;
      }

      /// \brief The x and y values.
      private: T data[2];
    };

    namespace detail {

      template<typename T>
      constexpr Vector2<T> gVector2Zero(0, 0);

      template<typename T>
      constexpr Vector2<T> gVector2One(1, 1);

      template<typename T>
      constexpr Vector2<T> gVector2NaN(
          std::numeric_limits<T>::quiet_NaN(),
          std::numeric_limits<T>::quiet_NaN());

    }  // namespace detail

    template<typename T>
    const Vector2<T> &Vector2<T>::Zero = detail::gVector2Zero<T>;

    template<typename T>
    const Vector2<T> &Vector2<T>::One = detail::gVector2One<T>;

    template<typename T>
    const Vector2<T> &Vector2<T>::NaN = detail::gVector2NaN<T>;

    typedef Vector2<int> Vector2i;
    typedef Vector2<double> Vector2d;
    typedef Vector2<float> Vector2f;
    }
  }
}
#endif
