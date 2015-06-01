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

#include <ignition/math/Matrix4.hh>

namespace ignition
{
  namespace math
  {
    /// \class Vector4 Vector4.hh ignition/math/Vector4.hh
    /// \brief T Generic x, y, z, w vector
    template<typename T>
    class Vector4
    {
      /// \brief math::Vector4(0, 0, 0, 0)
      public: static const Vector4<T> Zero;

      /// \brief math::Vector4(1, 1, 1, 1)
      public: static const Vector4<T> One;

      /// \brief Constructor
      public: Vector4()
      {
        this->data[0] = this->data[1] = this->data[2] = this->data[3] = 0;
      }

      /// \brief Constructor with component values
      /// \param[in] _x value along x axis
      /// \param[in] _y value along y axis
      /// \param[in] _z value along z axis
      /// \param[in] _w value along w axis
      public: Vector4(const T &_x, const T &_y, const T &_z, const T &_w)
      {
        this->data[0] = _x;
        this->data[1] = _y;
        this->data[2] = _z;
        this->data[3] = _w;
      }

      /// \brief Copy constructor
      /// \param[in] _v vector
      public: Vector4(const Vector4<T> &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];
        this->data[2] = _v[2];
        this->data[3] = _v[3];
      }

      /// \brief Destructor
      public: virtual ~Vector4() {}

      /// \brief Calc distance to the given point
      /// \param[in] _pt the point
      /// \return the distance
      public: T Distance(const Vector4<T> &_pt) const
      {
        return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
                    (this->data[1]-_pt[1])*(this->data[1]-_pt[1]) +
                    (this->data[2]-_pt[2])*(this->data[2]-_pt[2]) +
                    (this->data[3]-_pt[3])*(this->data[3]-_pt[3]));
      }

      /// \brief Returns the length (magnitude) of the vector
      public: T Length() const
      {
        return sqrt(
            this->data[0] * this->data[0] +
            this->data[1] * this->data[1] +
            this->data[2] * this->data[2] +
            this->data[3] * this->data[3]);
      }

      /// \brief Return the square of the length (magnitude) of the vector
      /// \return the length
      public: T SquaredLength() const
      {
        return this->data[0] * this->data[0] + this->data[1] * this->data[1] +
          this->data[2] * this->data[2] + this->data[3] * this->data[3];
      }

      /// \brief Normalize the vector length
      public: void Normalize()
      {
        T d = this->Length();

        this->data[0] /= d;
        this->data[1] /= d;
        this->data[2] /= d;
        this->data[3] /= d;
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

      /// \brief Assignment operator
      /// \param[in] _v the vector
      /// \return a reference to this vector
      public: Vector4<T> &operator=(const Vector4<T> &_v)
      {
        this->data[0] = _v[0];
        this->data[1] = _v[1];
        this->data[2] = _v[2];
        this->data[3] = _v[3];

        return *this;
      }

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
      /// \param[in] _pt another vector
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

      /// \brief Equal to operator
      /// \param[in] _v the other vector
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
      public: bool operator==(const Vector4<T> &_v) const
      {
        return equal(this->data[0], _v[0]) && equal(this->data[1], _v[1]) &&
               equal(this->data[2], _v[2]) && equal(this->data[3], _v[3]);
      }

      /// \brief Not equal to operator
      /// \param[in] _pt the other vector
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
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
      /// \return The value. Throws an IndexException if _index is out of
      /// bounds.
      /// \throws IndexException if _index is >= 4.
      public: inline T operator[](size_t _index) const
      {
        if (_index > 3)
          throw IndexException();
        return this->data[_index];
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

      /// \brief Get the w value.
      /// \return The w component of the vector
      public: inline T W() const
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

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pt Vector4 to output
      /// \return The stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Vector4<T> &_pt)
      {
        _out << _pt[0] << " " << _pt[1] << " " << _pt[2] << " " << _pt[3];
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _pt Vector4 to read values into
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, ignition::math::Vector4<T> &_pt)
      {
        T x, y, z, w;

        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> x >> y >> z >> w;
        _pt.Set(x, y, z, w);
        return _in;
      }

      /// \brief Data values, 0==x, 1==y, 2==z, 3==w
      private: T data[4];
    };

    template<typename T>
    const Vector4<T> Vector4<T>::Zero(0, 0, 0, 0);

    template<typename T>
    const Vector4<T> Vector4<T>::One(1, 1, 1, 1);

    typedef Vector4<int> Vector4i;
    typedef Vector4<double> Vector4d;
    typedef Vector4<float> Vector4f;
  }
}
#endif
