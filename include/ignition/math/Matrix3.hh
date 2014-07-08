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

#ifndef _IGNITION_MATRIX3_HH_
#define _IGNITION_MATRIX3_HH_

#include <cstring>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace ignition
{
  namespace math
  {
    /// \class Matrix3 Matrix3.hh ignition/math/Matrix3.hh
    /// \brief A 3x3 matrix class
    template<typename T>
    class IGNITION_VISIBLE Matrix3
    {
      /// \brief Identity matrix
      public: static const Matrix3<T> Identity;

      /// \brief Zero matrix
      public: static const Matrix3<T> Zero;

      /// \brief Constructor
      public: Matrix3()
      {
        std::memset(this->data, 0, sizeof(this->data[0][0])*9);
      }

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix3(const Matrix3<T> &_m)
      {
        std::memcpy(this->data, _m.data, sizeof(this->data[0][0])*9);
      }

      /// \brief Constructor
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      public: Matrix3(T _v00, T _v01, T _v02,
                      T _v10, T _v11, T _v12,
                      T _v20, T _v21, T _v22)
      {
        this->data[0][0] = _v00;
        this->data[0][1] = _v01;
        this->data[0][2] = _v02;
        this->data[1][0] = _v10;
        this->data[1][1] = _v11;
        this->data[1][2] = _v12;
        this->data[2][0] = _v20;
        this->data[2][1] = _v21;
        this->data[2][2] = _v22;
      }

      /// \brief Construct Matrix3 from a quaternion.
      /// \param[in] _q Quaternion.
      public: Matrix3(const Quaternion<T> &_q)
      {
        Quaternion<T> qt = _q;
        qt.Normalize();
        this->Set(1 - 2*qt.y()*qt.y() - 2 *qt.z()*qt.z(),
                  2 * qt.x()*qt.y() - 2*qt.z()*qt.w(),
                  2 * qt.x() * qt.z() + 2 * qt.y() * qt.w(),
                  2 * qt.x() * qt.y() + 2 * qt.z() * qt.w(),
                  1 - 2*qt.x()*qt.x() - 2 * qt.z()*qt.z(),
                  2 * qt.y() * qt.z() - 2 * qt.x() * qt.w(),
                  2 * qt.x() * qt.z() - 2 * qt.y() * qt.w(),
                  2 * qt.y() * qt.z() + 2 * qt.x() * qt.w(),
                  1 - 2 * qt.x()*qt.x() - 2 * qt.y()*qt.y());
      }

      /// \brief Desctructor
      public: virtual ~Matrix3() {}

      /// \brief Set values
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      public: void Set(T _v00, T _v01, T _v02,
                       T _v10, T _v11, T _v12,
                       T _v20, T _v21, T _v22)
      {
        this->data[0][0] = _v00;
        this->data[0][1] = _v01;
        this->data[0][2] = _v02;
        this->data[1][0] = _v10;
        this->data[1][1] = _v11;
        this->data[1][2] = _v12;
        this->data[2][0] = _v20;
        this->data[2][1] = _v21;
        this->data[2][2] = _v22;
      }

      /// \brief Set the matrix from three axis (1 per column)
      /// \param[in] _xAxis The x axis
      /// \param[in] _yAxis The y axis
      /// \param[in] _zAxis The z axis
      public: void Axes(const Vector3<T> &_xAxis,
                        const Vector3<T> &_yAxis,
                        const Vector3<T> &_zAxis)
      {
        this->Col(0, _xAxis);
        this->Col(1, _yAxis);
        this->Col(2, _zAxis);
      }

      /// \brief Set the matrix from an axis and angle
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      public: void Axis(const Vector3<T> &_axis, T _angle)
      {
        T c = cos(_angle);
        T s = sin(_angle);
        T C = 1-c;

        this->data[0][0] = _axis.x()*_axis.x()*C + c;
        this->data[0][1] = _axis.x()*_axis.y()*C - _axis.z()*s;
        this->data[0][2] = _axis.x()*_axis.z()*C + _axis.y()*s;

        this->data[1][0] = _axis.y()*_axis.x()*C + _axis.z()*s;
        this->data[1][1] = _axis.y()*_axis.y()*C + c;
        this->data[1][2] = _axis.y()*_axis.z()*C - _axis.x()*s;

        this->data[2][0] = _axis.z()*_axis.x()*C - _axis.y()*s;
        this->data[2][1] = _axis.z()*_axis.y()*C + _axis.x()*s;
        this->data[2][2] = _axis.z()*_axis.z()*C + c;
      }

      /// \brief Set a column
      /// \param[in] _c The colum index (0, 1, 2)
      /// \param[in] _v The value to set in each row of the column
      public: void Col(unsigned int _c, const Vector3<T> &_v)
      {
        if (_c >= 3)
          throw IndexException();

        this->data[0][_c] = _v.x();
        this->data[1][_c] = _v.y();
        this->data[2][_c] = _v.z();
      }

      /// \brief returns the element wise difference of two matrices
      public: Matrix3<T> operator-(const Matrix3<T> &_m) const
      {
        return Matrix3<T>(
            this->data[0][0] - _m(0, 0),
            this->data[0][1] - _m(0, 1),
            this->data[0][2] - _m(0, 2),
            this->data[1][0] - _m(1, 0),
            this->data[1][1] - _m(1, 1),
            this->data[1][2] - _m(1, 2),
            this->data[2][0] - _m(2, 0),
            this->data[2][1] - _m(2, 1),
            this->data[2][2] - _m(2, 2));
      }

      /// \brief returns the element wise sum of two matrices
      public: Matrix3<T> operator+(const Matrix3<T> &_m) const
      {
        return Matrix3<T>(
            this->data[0][0]+_m(0, 0),
            this->data[0][1]+_m(0, 1),
            this->data[0][2]+_m(0, 2),
            this->data[1][0]+_m(1, 0),
            this->data[1][1]+_m(1, 1),
            this->data[1][2]+_m(1, 2),
            this->data[2][0]+_m(2, 0),
            this->data[2][1]+_m(2, 1),
            this->data[2][2]+_m(2, 2));
      }

      /// \brief returns the element wise scalar multiplication
      public: Matrix3<T> operator*(const T &_s) const
      {
        return Matrix3<T>(
          _s * this->data[0][0], _s * this->data[0][1], _s * this->data[0][2],
          _s * this->data[1][0], _s * this->data[1][1], _s * this->data[1][2],
          _s * this->data[2][0], _s * this->data[2][1], _s * this->data[2][2]);
      }

      /// \brief Matrix multiplication operator
      /// \param[in] _m Matrix3<T> to multiply
      /// \return product of this * _m
      public: Matrix3<T> operator*(const Matrix3<T> &_m) const
      {
        return Matrix3<T>(
            // first row
            this->data[0][0]*_m(0, 0)+
            this->data[0][1]*_m(1, 0)+
            this->data[0][2]*_m(2, 0),

            this->data[0][0]*_m(0, 1)+
            this->data[0][1]*_m(1, 1)+
            this->data[0][2]*_m(2, 1),

            this->data[0][0]*_m(0, 2)+
            this->data[0][1]*_m(1, 2)+
            this->data[0][2]*_m(2, 2),

            // second row
            this->data[1][0]*_m(0, 0)+
            this->data[1][1]*_m(1, 0)+
            this->data[1][2]*_m(2, 0),

            this->data[1][0]*_m(0, 1)+
            this->data[1][1]*_m(1, 1)+
            this->data[1][2]*_m(2, 1),

            this->data[1][0]*_m(0, 2)+
            this->data[1][1]*_m(1, 2)+
            this->data[1][2]*_m(2, 2),

            // third row
            this->data[2][0]*_m(0, 0)+
            this->data[2][1]*_m(1, 0)+
            this->data[2][2]*_m(2, 0),

            this->data[2][0]*_m(0, 1)+
            this->data[2][1]*_m(1, 1)+
            this->data[2][2]*_m(2, 1),

            this->data[2][0]*_m(0, 2)+
            this->data[2][1]*_m(1, 2)+
            this->data[2][2]*_m(2, 2));
      }

      /// \brief Multiplication operator
      /// \param _vec Vector3
      /// \return Resulting vector from multiplication
      public: Vector3<T> operator*(const Vector3<T> &_vec) const
      {
        return Vector3<T>(
            this->data[0][0]*_vec.x() + this->data[0][1]*_vec.y() +
            this->data[0][2]*_vec.z(),
            this->data[1][0]*_vec.x() + this->data[1][1]*_vec.y() +
            this->data[1][2]*_vec.z(),
            this->data[2][0]*_vec.x() + this->data[2][1]*_vec.y() +
            this->data[2][2]*_vec.z());
      }

      /// \brief Matrix multiplication operator for scaling.
      /// \param[in] _s Scaling factor.
      /// \param[in] _m Input matrix.
      /// \return A scaled matrix.
      public: friend inline Matrix3<T> operator*(T _s, const Matrix3<T> &_m)
      {
        return _m * _s;
      }

      /// \brief Equality test operator
      /// \param[in] _m Matrix3<T> to test
      /// \return True if equal (using the default tolerance of 1e-6)
      public: bool operator==(const Matrix3<T> &_m) const
      {
        return math::equal(this->data[0][0], _m(0, 0)) &&
               math::equal(this->data[0][1], _m(0, 1)) &&
               math::equal(this->data[0][2], _m(0, 2)) &&

               math::equal(this->data[1][0], _m(1, 0)) &&
               math::equal(this->data[1][1], _m(1, 1)) &&
               math::equal(this->data[1][2], _m(1, 2)) &&

               math::equal(this->data[2][0], _m(2, 0)) &&
               math::equal(this->data[2][1], _m(2, 1)) &&
               math::equal(this->data[2][2], _m(2, 2));
      }

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline const T &operator()(size_t _row, size_t _col) const
      {
        if (_row >= 3 || _col >= 3)
          throw IndexException();
        return this->data[_row][_col];
      }

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline T &operator()(size_t _row, size_t _col)
      {
        if (_row >= 3 || _col >=3)
          throw IndexException();
        return this->data[_row][_col];
      }

      /// \brief Stream insertion operator
      /// \param[in] _out Output stream
      /// \param[in] _m Matrix to output
      /// \return the stream
      public: friend std::ostream IGNITION_VISIBLE &operator<<(
                  std::ostream &_out, const ignition::math::Matrix3<T> &_m)
      {
        _out << precision(_m(0, 0), 6) << " "
             << precision(_m(0, 1), 6) << " "
             << precision(_m(0, 2), 6) << " "
             << precision(_m(1, 0), 6) << " "
             << precision(_m(1, 1), 6) << " "
             << precision(_m(1, 2), 6) << " "
             << precision(_m(2, 0), 6) << " "
             << precision(_m(2, 1), 6) << " "
             << precision(_m(2, 2), 6);

        return _out;
      }
      /// \brief Stream extraction operator
      /// \param _in input stream
      /// \param _pt Matrix3 to read values into
      /// \return the stream
      public: friend std::istream IGNITION_VISIBLE &operator>>(
                  std::istream &_in, ignition::math::Matrix3<T> &_m)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        T d[9];
        _in >> d[0] >> d[1] >> d[2]
            >> d[3] >> d[4] >> d[5]
            >> d[6] >> d[7] >> d[8];

        _m.Set(d[0], d[1], d[2],
               d[3], d[4], d[5],
               d[6], d[7], d[8]);
        return _in;
      }

      /// \brief the 3x3 matrix
      private: T data[3][3];
    };

    template<typename T>
    const Matrix3<T> Matrix3<T>::Identity(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1);

    template<typename T>
    const Matrix3<T> Matrix3<T>::Zero(
        0, 0, 0,
        0, 0, 0,
        0, 0, 0);

    typedef Matrix3<int> Matrix3i;
    typedef Matrix3<double> Matrix3d;
    typedef Matrix3<float> Matrix3f;
  }
}

#endif
