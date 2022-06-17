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
#ifndef GZ_MATH_VECTOR4_HH_
#define GZ_MATH_VECTOR4_HH_

#include <algorithm>
#include <cmath>
#include <limits>

#include <gz/math/Matrix4.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Vector4 Vector4.hh gz/math/Vector4.hh
    /// \brief T Generic x, y, z, w vector
    template<typename T>
    class Vector4
    {
      /// \brief math::Vector4(0, 0, 0, 0)
      public: static const Vector4<T> &Zero;

      /// \brief math::Vector4(1, 1, 1, 1)
      public: static const Vector4<T> &One;

      /// \brief math::Vector4(NaN, NaN, NaN, NaN)
      public: static const Vector4 &NaN;

      /// \brief Constructor
      public: constexpr Vector4()
      : data{0, 0, 0, 0}
      {
      }

      /// \brief Constructor with component values
      /// \param[in] _x value along x axis
      /// \param[in] _y value along y axis
      /// \param[in] _z value along z axis
      /// \param[in] _w value along w axis
      public: constexpr Vector4(const T &_x, const T &_y, const T &_z,
                                const T &_w)
      : data{_x, _y, _z, _w}
      {
      }

      /// \brief Copy constructor
      /// \param[in] _v vector
      public: Vector4(const Vector4<T> &_v) = default;

      /// \brief Destructor
      public: ~Vector4() = default;

      /// \brief Calc distance to the given point
      /// \param[in] _pt the point
      /// \return the distance
      public: T Distance(const Vector4<T> &_pt) const
      {
        return static_cast<T>(sqrt(
                    (this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
                    (this->data[1]-_pt[1])*(this->data[1]-_pt[1]) +
                    (this->data[2]-_pt[2])*(this->data[2]-_pt[2]) +
                    (this->data[3]-_pt[3])*(this->data[3]-_pt[3])));
      }

      /// \brief Calc distance to the given point
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value along z
      /// \param[in] _w value along w
      /// \return the distance
      public: T Distance(T _x, T _y, T _z, T _w) const
      {
        return this->Distance(Vector4(_x, _y, _z, _w));
      }

      /// \brief Returns the length (magnitude) of the vector
      /// \return The length
      public: T Length() const
      {
        return static_cast<T>(sqrt(this->SquaredLength()));
      }

      /// \brief Return the square of the length (magnitude) of the vector
      /// \return the length
      public: T SquaredLength() const
      {
        return
          this->data[0] * this->data[0] +
          this->data[1] * this->data[1] +
          this->data[2] * this->data[2] +
          this->data[3] * this->data[3];
      }

      /// \brief Round to near whole number.
      public: void Round()
      {
        this->data[0] = static_cast<T>(std::nearbyint(this->data[0]));
        this->data[1] = static_cast<T>(std::nearbyint(this->data[1]));
        this->data[2] = static_cast<T>(std::nearbyint(this->data[2]));
        this->data[3] = static_cast<T>(std::nearbyint(this->data[3]));
      }

      /// \brief Get a rounded version of this vector
      /// \return a rounded vector
      public: Vector4 Rounded() const
      {
        Vector4<T> result = *this;
        result.Round();
        return result;
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
        if (!std::isfinite(static_cast<double>(this->data[3])))
          this->data[3] = 0;
      }

      /// \brief Normalize the vector length
      public: void Normalize()
      {
        T d = this->Length();

        if (!equal<T>(d, static_cast<T>(0.0)))
        {
          this->data[0] /= d;
          this->data[1] /= d;
          this->data[2] /= d;
          this->data[3] /= d;
        }
      }

      /// \brief Return a normalized vector
      /// \return unit length vector
      public: Vector4 Normalized() const
      {
        Vector4<T> result = *this;
        result.Normalize();
        return result;
      }

      /// \brief Return the dot product of this vector and another vector
      /// \param[in] _v the vector
      /// \return the dot product
      public: T Dot(const Vector4<T> &_v) const
      {
        return this->data[0] * _v[0] +
               this->data[1] * _v[1] +
               this->data[2] * _v[2] +
               this->data[3] * _v[3];
      }

      /// \brief Return the absolute dot product of this vector and
      /// another vector. This is similar to the Dot function, except the
      /// absolute value of each component of the vector is used.
      ///
      /// result = abs(x1 * x2) + abs(y1 * y2) + abs(z1 * z2) + abs(w1 * w2)
      ///
      /// \param[in] _v the vector
      /// \return The absolute dot product
      public: T AbsDot(const Vector4<T> &_v) const
      {
        return std::abs(this->data[0] * _v[0]) +
               std::abs(this->data[1] * _v[1]) +
               std::abs(this->data[2] * _v[2]) +
               std::abs(this->data[3] * _v[3]);
      }

      /// \brief Get the absolute value of the vector
      /// \return a vector with positive elements
      public: Vector4 Abs() const
      {
        return Vector4(std::abs(this->data[0]),
                       std::abs(this->data[1]),
                       std::abs(this->data[2]),
                       std::abs(this->data[3]));
      }

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x axis
      /// \param[in] _y value along y axis
      /// \param[in] _z value along z axis
      /// \param[in] _w value along w axis
      public: void Set(T _x = 0, T _y = 0, T _z = 0, T _w = 0)
      {
        this->data[0] = _x;
        this->data[1] = _y;
        this->data[2] = _z;
        this->data[3] = _w;
      }

      /// \brief Set this vector's components to the maximum of itself and the
      ///        passed in vector
      /// \param[in] _v the maximum clamping vector
      public: void Max(const Vector4<T> &_v)
      {
        this->data[0] = std::max(_v[0], this->data[0]);
        this->data[1] = std::max(_v[1], this->data[1]);
        this->data[2] = std::max(_v[2], this->data[2]);
        this->data[3] = std::max(_v[3], this->data[3]);
      }

      /// \brief Set this vector's components to the minimum of itself and the
      ///        passed in vector
      /// \param[in] _v the minimum clamping vector
      public: void Min(const Vector4<T> &_v)
      {
        this->data[0] = std::min(_v[0], this->data[0]);
        this->data[1] = std::min(_v[1], this->data[1]);
        this->data[2] = std::min(_v[2], this->data[2]);
        this->data[3] = std::min(_v[3], this->data[3]);
      }

      /// \brief Get the maximum value in the vector
      /// \return the maximum element
      public: T Max() const
      {
        return *std::max_element(this->data, this->data+4);
      }

      /// \brief Get the minimum value in the vector
      /// \return the minimum element
      public: T Min() const
      {
        return *std::min_element(this->data, this->data+4);
      }

      /// \brief Return the sum of the values
      /// \return the sum
      public: T Sum() const
      {
        return this->data[0] + this->data[1] + this->data[2] + this->data[3];
      }

      /// \brief Assignment operator
      /// \param[in] _v the vector
      /// \return a reference to this vector
      public: Vector4<T> &operator=(const Vector4<T> &_v) = default;

      /// \brief Assignment operator
      /// \param[in] _value
      public: Vector4<T> &operator=(T _value)
      {
        this->data[0] = _value;
        this->data[1] = _value;
        this->data[2] = _value;
        this->data[3] = _value;

        return *this;
      }

      /// \brief Addition operator
      /// \param[in] _v the vector to add
      /// \result a sum vector
      public: Vector4<T> operator+(const Vector4<T> &_v) const
      {
        return Vector4<T>(this->data[0] + _v[0],
                          this->data[1] + _v[1],
                          this->data[2] + _v[2],
                          this->data[3] + _v[3]);
      }

      /// \brief Addition operator
      /// \param[in] _v the vector to add
      /// \return this vector
      public: const Vector4<T> &operator+=(const Vector4<T> &_v)
      {
        this->data[0] += _v[0];
        this->data[1] += _v[1];
        this->data[2] += _v[2];
        this->data[3] += _v[3];

        return *this;
      }

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \return sum vector
      public: inline Vector4<T> operator+(const T _s) const
      {
        return Vector4<T>(this->data[0] + _s,
                          this->data[1] + _s,
                          this->data[2] + _s,
                          this->data[3] + _s);
      }

      /// \brief Addition operators
      /// \param[in] _s the scalar addend
      /// \param[in] _v input vector
      /// \return sum vector
      public: friend inline Vector4<T> operator+(const T _s,
                                                 const Vector4<T> &_v)
      {
        return _v + _s;
      }

      /// \brief Addition assignment operator
      /// \param[in] _s scalar addend
      /// \return this
      public: const Vector4<T> &operator+=(const T _s)
      {
        this->data[0] += _s;
        this->data[1] += _s;
        this->data[2] += _s;
        this->data[3] += _s;

        return *this;
      }

      /// \brief Negation operator
      /// \return negative of this vector
      public: inline Vector4 operator-() const
      {
        return Vector4(-this->data[0], -this->data[1],
                       -this->data[2], -this->data[3]);
      }

      /// \brief Subtraction operator
      /// \param[in] _v the vector to substract
      /// \return a vector
      public: Vector4<T> operator-(const Vector4<T> &_v) const
      {
        return Vector4<T>(this->data[0] - _v[0],
                          this->data[1] - _v[1],
                          this->data[2] - _v[2],
                          this->data[3] - _v[3]);
      }

      /// \brief Subtraction assigment operators
      /// \param[in] _v the vector to substract
      /// \return this vector
      public: const Vector4<T> &operator-=(const Vector4<T> &_v)
      {
        this->data[0] -= _v[0];
        this->data[1] -= _v[1];
        this->data[2] -= _v[2];
        this->data[3] -= _v[3];

        return *this;
      }

      /// \brief Subtraction operators
      /// \param[in] _s the scalar subtrahend
      /// \return difference vector
      public: inline Vector4<T> operator-(const T _s) const
      {
        return Vector4<T>(this->data[0] - _s,
                          this->data[1] - _s,
                          this->data[2] - _s,
                          this->data[3] - _s);
      }

      /// \brief Subtraction operators
      /// \param[in] _s the scalar minuend
      /// \param[in] _v vector subtrahend
      /// \return difference vector
      public: friend inline Vector4<T> operator-(const T _s,
                                                 const Vector4<T> &_v)
      {
        return {_s - _v.X(), _s - _v.Y(), _s - _v.Z(), _s - _v.W()};
      }

      /// \brief Subtraction assignment operator
      /// \param[in] _s scalar subtrahend
      /// \return this
      public: const Vector4<T> &operator-=(const T _s)
      {
        this->data[0] -= _s;
        this->data[1] -= _s;
        this->data[2] -= _s;
        this->data[3] -= _s;

        return *this;
      }

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _v the vector to perform element wise division with
      /// \return a result vector
      public: const Vector4<T> operator/(const Vector4<T> &_v) const
      {
        return Vector4<T>(this->data[0] / _v[0],
                          this->data[1] / _v[1],
                          this->data[2] / _v[2],
                          this->data[3] / _v[3]);
      }

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _v the vector to perform element wise division with
      /// \return this
      public: const Vector4<T> &operator/=(const Vector4<T> &_v)
      {
        this->data[0] /= _v[0];
        this->data[1] /= _v[1];
        this->data[2] /= _v[2];
        this->data[3] /= _v[3];

        return *this;
      }

      /// \brief Division assignment operator
      /// \remarks Performs element wise division,
      /// which has limited use.
      /// \param[in] _v another vector
      /// \return a result vector
      public: const Vector4<T> operator/(T _v) const
      {
        return Vector4<T>(this->data[0] / _v, this->data[1] / _v,
            this->data[2] / _v, this->data[3] / _v);
      }

      /// \brief Division operator
      /// \param[in] _v scaling factor
      /// \return a vector
      public: const Vector4<T> &operator/=(T _v)
      {
        this->data[0] /= _v;
        this->data[1] /= _v;
        this->data[2] /= _v;
        this->data[3] /= _v;

        return *this;
      }

      /// \brief Multiplication operator.
      /// \remarks Performs element wise multiplication,
      /// which has limited use.
      /// \param[in] _pt another vector
      /// \return result vector
      public: const Vector4<T> operator*(const Vector4<T> &_pt) const
      {
        return Vector4<T>(this->data[0] * _pt[0],
                          this->data[1] * _pt[1],
                          this->data[2] * _pt[2],
                          this->data[3] * _pt[3]);
      }

      /// \brief Matrix multiplication operator.
      /// \param[in] _m matrix
      /// \return the vector multiplied by _m
      public: const Vector4<T> operator*(const Matrix4<T> &_m) const
      {
        return Vector4<T>(
            this->data[0]*_m(0, 0) + this->data[1]*_m(1, 0) +
            this->data[2]*_m(2, 0) + this->data[3]*_m(3, 0),
            this->data[0]*_m(0, 1) + this->data[1]*_m(1, 1) +
            this->data[2]*_m(2, 1) + this->data[3]*_m(3, 1),
            this->data[0]*_m(0, 2) + this->data[1]*_m(1, 2) +
            this->data[2]*_m(2, 2) + this->data[3]*_m(3, 2),
            this->data[0]*_m(0, 3) + this->data[1]*_m(1, 3) +
            this->data[2]*_m(2, 3) + this->data[3]*_m(3, 3));
      }

      /// \brief Multiplication assignment operator
      /// \remarks Performs element wise multiplication,
      /// which has limited use.
      /// \param[in] _pt a vector
      /// \return this
      public: const Vector4<T> &operator*=(const Vector4<T> &_pt)
      {
        this->data[0] *= _pt[0];
        this->data[1] *= _pt[1];
        this->data[2] *= _pt[2];
        this->data[3] *= _pt[3];

        return *this;
      }

      /// \brief Multiplication operators
      /// \param[in] _v scaling factor
      /// \return a  scaled vector
      public: const Vector4<T> operator*(T _v) const
      {
        return Vector4<T>(this->data[0] * _v, this->data[1] * _v,
            this->data[2] * _v, this->data[3] * _v);
      }

      /// \brief Scalar left multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _v the vector to scale
      /// \return a scaled vector
      public: friend inline const Vector4 operator*(const T _s,
                                                    const Vector4 &_v)
      {
        return Vector4(_v * _s);
      }

      /// \brief Multiplication assignment operator
      /// \param[in] _v scaling factor
      /// \return this
      public: const Vector4<T> &operator*=(T _v)
      {
        this->data[0] *= _v;
        this->data[1] *= _v;
        this->data[2] *= _v;
        this->data[3] *= _v;

        return *this;
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _v the vector to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the vectors are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const Vector4 &_v, const T &_tol) const
      {
        return equal<T>(this->data[0], _v[0], _tol)
            && equal<T>(this->data[1], _v[1], _tol)
            && equal<T>(this->data[2], _v[2], _tol)
            && equal<T>(this->data[3], _v[3], _tol);
      }

      /// \brief Equal to operator
      /// \param[in] _v the other vector
      /// \return true if each component is equal within a
      /// default tolerence (1e-6), false otherwise
      public: bool operator==(const Vector4<T> &_v) const
      {
        return this->Equal(_v, static_cast<T>(1e-6));
      }

      /// \brief Not equal to operator
      /// \param[in] _pt the other vector
      /// \return false if each component is equal within a
      /// default tolerence (1e-6), true otherwise
      public: bool operator!=(const Vector4<T> &_pt) const
      {
        return !(*this == _pt);
      }

      /// \brief See if a point is finite (e.g., not nan)
      /// \return true if finite, false otherwise
      public: bool IsFinite() const
      {
        // std::isfinite works with floating point values,
        // need to explicit cast to avoid ambiguity in vc++.
        return std::isfinite(static_cast<double>(this->data[0])) &&
               std::isfinite(static_cast<double>(this->data[1])) &&
               std::isfinite(static_cast<double>(this->data[2])) &&
               std::isfinite(static_cast<double>(this->data[3]));
      }

      /// \brief Array subscript operator
      /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z, 3 == w.
      /// The index is clamped to the range (0,3).
      /// \return The value.
      public: T &operator[](const std::size_t _index)
      {
        return this->data[clamp(_index, GZ_ZERO_SIZE_T, GZ_THREE_SIZE_T)];
      }

      /// \brief Const-qualified array subscript operator
      /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z, 3 == w.
      /// The index is clamped to the range (0,3).
      /// \return The value.
      public: T operator[](const std::size_t _index) const
      {
        return this->data[clamp(_index, GZ_ZERO_SIZE_T, GZ_THREE_SIZE_T)];
      }

      /// \brief Return a mutable x value.
      /// \return The x component of the vector
      public: T &X()
      {
        return this->data[0];
      }

      /// \brief Return a mutable y value.
      /// \return The y component of the vector
      public: T &Y()
      {
        return this->data[1];
      }

      /// \brief Return a mutable z value.
      /// \return The z component of the vector
      public: T &Z()
      {
        return this->data[2];
      }

      /// \brief Return a mutable w value.
      /// \return The w component of the vector
      public: T &W()
      {
        return this->data[3];
      }

      /// \brief Get the x value.
      /// \return The x component of the vector
      public: T X() const
      {
        return this->data[0];
      }

      /// \brief Get the y value.
      /// \return The y component of the vector
      public: T Y() const
      {
        return this->data[1];
      }

      /// \brief Get the z value.
      /// \return The z component of the vector
      public: T Z() const
      {
        return this->data[2];
      }

      /// \brief Get the w value.
      /// \return The w component of the vector
      public: T W() const
      {
        return this->data[3];
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

      /// \brief Set the w value.
      /// \param[in] _v Value for the w component.
      public: inline void W(const T &_v)
      {
        this->data[3] = _v;
      }

      /// \brief Less than operator.
      /// \param[in] _pt Vector to compare.
      /// \return True if this vector's X(), Y(), Z() or W() value is less
      /// than the given vector's corresponding values.
      public: bool operator<(const Vector4<T> &_pt) const
      {
        return this->data[0] < _pt[0] || this->data[1] < _pt[1] ||
               this->data[2] < _pt[2] || this->data[3] < _pt[3];
      }

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pt Vector4 to output
      /// \return The stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const gz::math::Vector4<T> &_pt)
      {
        for (auto i : {0, 1, 2, 3})
        {
          if (i > 0)
            _out << " ";

          appendToStream(_out, _pt[i]);
        }
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector4 to read values into
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, gz::math::Vector4<T> &_pt)
      {
        T x, y, z, w;

        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> x >> y >> z >> w;
        if (!_in.fail())
        {
          _pt.Set(x, y, z, w);
        }
        return _in;
      }

      /// \brief Data values, 0==x, 1==y, 2==z, 3==w
      private: T data[4];
    };

    namespace detail {

      template<typename T>
      constexpr Vector4<T> gVector4Zero(0, 0, 0, 0);

      template<typename T>
      constexpr Vector4<T> gVector4One(1, 1, 1, 1);

      template<typename T>
      constexpr Vector4<T> gVector4NaN(
          std::numeric_limits<T>::quiet_NaN(),
          std::numeric_limits<T>::quiet_NaN(),
          std::numeric_limits<T>::quiet_NaN(),
          std::numeric_limits<T>::quiet_NaN());

    }  // namespace detail

    template<typename T>
    const Vector4<T> &Vector4<T>::Zero = detail::gVector4Zero<T>;

    template<typename T>
    const Vector4<T> &Vector4<T>::One = detail::gVector4One<T>;

    template<typename T>
    const Vector4<T> &Vector4<T>::NaN = detail::gVector4NaN<T>;

    typedef Vector4<int> Vector4i;
    typedef Vector4<double> Vector4d;
    typedef Vector4<float> Vector4f;
    }
  }
}
#endif
