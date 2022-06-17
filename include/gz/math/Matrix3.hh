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
#ifndef GZ_MATH_MATRIX3_HH_
#define GZ_MATH_MATRIX3_HH_

#include <algorithm>
#include <cstring>
#include <utility>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    template <typename T> class Quaternion;

    /// \class Matrix3 Matrix3.hh gz/math/Matrix3.hh
    /// \brief A 3x3 matrix class.
    ///
    /// The following two type definitions are provided:
    ///
    /// * \ref Matrix3i : Equivalent to Matrix3<int>
    /// * \ref Matrix3f : Equivalent to Matrix3<float>
    /// * \ref Matrix3d : Equivalent to Matrix3<double>
    /// ## Examples
    ///
    /// * C++
    ///
    /// \snippet examples/matrix3_example.cc complete
    ///
    /// * Ruby
    /// \code{.rb}
    /// # Modify the RUBYLIB environment variable to include the Gazebo Math
    /// # library install path. For example, if you install to /user:
    /// #
    /// # $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
    /// #
    /// require 'gz/math'
    ///
    /// # Construct a default matrix3.
    /// m = Gz::Math::Matrix3d.new
    /// printf("The default constructed matrix m has the following "+
    ///        "values.\n\t" +
    ///        "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
    ///        m.(0, 0), m.(0, 1), m.(0, 2),
    ///        m.(1, 0), m.(1, 1), m.(1, 2),
    ///        m.(2, 0), m.(2, 1), m.(2, 2))
    ///
    /// # Set the first column of the matrix.
    /// m.SetCol(0, Gz::Math::Vector3d.new(3, 4, 5))
    /// printf("Setting the first column of the matrix m to 3, 4, 5.\n\t" +
    ///        "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
    ///        m.(0, 0), m.(0, 1), m.(0, 2),
    ///        m.(1, 0), m.(1, 1), m.(1, 2),
    ///        m.(2, 0), m.(2, 1), m.(2, 2))
    ///
    /// # Transpose the matrix.
    /// t = m.Transposed()
    /// printf("The transposed matrix t has the values.\n\t"+
    ///        "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
    ///        t.(0, 0), t.(0, 1), t.(0, 2),
    ///        t.(1, 0), t.(1, 1), t.(1, 2),
    ///        t.(2, 0), t.(2, 1), t.(2, 2))
    ///
    /// # Multiply the two matrices.
    /// m = m * t
    /// printf("m * t = " +
    ///        "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
    ///        m.(0, 0), m.(0, 1), m.(0, 2),
    ///        m.(1, 0), m.(1, 1), m.(1, 2),
    ///        m.(2, 0), m.(2, 1), m.(2, 2))
    /// \endcode
    template<typename T>
    class Matrix3
    {
      /// \brief A Matrix3 initialized to identity.
      /// This is equivalent to math::Matrix3<T>(1, 0, 0, 0, 1, 0, 0, 0, 1).
      public: static const Matrix3<T> &Identity;

      /// \brief A Matrix3 initialized to zero.
      /// This is equivalent to math::Matrix3<T>(0, 0, 0, 0, 0, 0, 0, 0, 0).
      public: static const Matrix3<T> &Zero;

      /// \brief Default constructor that initializes the matrix3 to zero.
      public: Matrix3()
      {
        std::memset(this->data, 0, sizeof(this->data[0][0])*9);
      }

      /// \brief Copy constructor.
      /// \param _m Matrix to copy
      public: Matrix3(const Matrix3<T> &_m) = default;

      /// \brief Construct a matrix3 using nine values.
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      public: constexpr Matrix3(T _v00, T _v01, T _v02,
                                T _v10, T _v11, T _v12,
                                T _v20, T _v21, T _v22)
      : data{{_v00, _v01, _v02},
             {_v10, _v11, _v12},
             {_v20, _v21, _v22}}
      {
      }

      /// \brief Construct 3x3 rotation Matrix from a quaternion.
      /// \param[in] _q Quaternion to set the Matrix3 from.
      public: explicit Matrix3(const Quaternion<T> &_q)
      {
        Quaternion<T> qt = _q;
        qt.Normalize();
        this->Set(1 - 2*qt.Y()*qt.Y() - 2 *qt.Z()*qt.Z(),
                  2 * qt.X()*qt.Y() - 2*qt.Z()*qt.W(),
                  2 * qt.X() * qt.Z() + 2 * qt.Y() * qt.W(),
                  2 * qt.X() * qt.Y() + 2 * qt.Z() * qt.W(),
                  1 - 2*qt.X()*qt.X() - 2 * qt.Z()*qt.Z(),
                  2 * qt.Y() * qt.Z() - 2 * qt.X() * qt.W(),
                  2 * qt.X() * qt.Z() - 2 * qt.Y() * qt.W(),
                  2 * qt.Y() * qt.Z() + 2 * qt.X() * qt.W(),
                  1 - 2 * qt.X()*qt.X() - 2 * qt.Y()*qt.Y());
      }

      /// \brief Desctructor
      public: ~Matrix3() = default;

      /// \brief Set a single value.
      /// \param[in] _row row index. _row is clamped to the range [0,2]
      /// \param[in] _col column index. _col is clamped to the range [0,2]
      /// \param[in] _v New value.
      public: void Set(size_t _row, size_t _col, T _v)
      {
        this->data[clamp(_row, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)]
                  [clamp(_col, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)] = _v;
      }

      /// \brief Set values.
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

      /// \brief Set the matrix from three axis (1 per column).
      /// \param[in] _xAxis The x axis, the first column of the matrix.
      /// \param[in] _yAxis The y axis, the second column of the matrix.
      /// \param[in] _zAxis The z axis, the third column of the matrix.
      /// \deprecated Use SetAxes(const Vector3<T> &, const Vector3<T> &,
      /// const Vector3<T> &,)
      public: void GZ_DEPRECATED(7) Axes(const Vector3<T> &_xAxis,
                  const Vector3<T> &_yAxis,
                  const Vector3<T> &_zAxis)
      {
        this->SetAxes(_xAxis, _yAxis, _zAxis);
      }

      /// \brief Set the matrix from three axis (1 per column).
      /// \param[in] _xAxis The x axis, the first column of the matrix.
      /// \param[in] _yAxis The y axis, the second column of the matrix.
      /// \param[in] _zAxis The z axis, the third column of the matrix.
      public: void SetAxes(const Vector3<T> &_xAxis,
                           const Vector3<T> &_yAxis,
                           const Vector3<T> &_zAxis)
      {
        this->SetCol(0, _xAxis);
        this->SetCol(1, _yAxis);
        this->SetCol(2, _zAxis);
      }

      /// \brief Set as a rotation matrix from an axis and angle.
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      /// \deprecated Use SetFromAxisAngle(const Vector3<T> &, T)
      public: void GZ_DEPRECATED(7) Axis(const Vector3<T> &_axis, T _angle)
      {
        this->SetFromAxisAngle(_axis, _angle);
      }

      /// \brief Set as a rotation matrix from an axis and angle.
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      public: void SetFromAxisAngle(const Vector3<T> &_axis, T _angle)
      {
        T c = cos(_angle);
        T s = sin(_angle);
        T C = 1-c;

        this->data[0][0] = _axis.X()*_axis.X()*C + c;
        this->data[0][1] = _axis.X()*_axis.Y()*C - _axis.Z()*s;
        this->data[0][2] = _axis.X()*_axis.Z()*C + _axis.Y()*s;

        this->data[1][0] = _axis.Y()*_axis.X()*C + _axis.Z()*s;
        this->data[1][1] = _axis.Y()*_axis.Y()*C + c;
        this->data[1][2] = _axis.Y()*_axis.Z()*C - _axis.X()*s;

        this->data[2][0] = _axis.Z()*_axis.X()*C - _axis.Y()*s;
        this->data[2][1] = _axis.Z()*_axis.Y()*C + _axis.X()*s;
        this->data[2][2] = _axis.Z()*_axis.Z()*C + c;
      }

      /// \brief Set as a rotation matrix to represent rotation from
      /// vector _v1 to vector _v2, so that
      /// _v2.Normalize() == this * _v1.Normalize() holds.
      ///
      /// \param[in] _v1 The first vector
      /// \param[in] _v2 The second vector
      /// \deprecated Use SetFrom2Axes(const Vector3<T> &, const Vector3<T> &)
      public: void GZ_DEPRECATED(7) From2Axes(
                  const Vector3<T> &_v1, const Vector3<T> &_v2)
      {
        this->SetFrom2Axes(_v1, _v2);
      }

      /// \brief Set as a rotation matrix to represent rotation from
      /// vector _v1 to vector _v2, so that
      /// _v2.Normalize() == this * _v1.Normalize() holds.
      ///
      /// \param[in] _v1 The first vector
      /// \param[in] _v2 The second vector
      public: void SetFrom2Axes(const Vector3<T> &_v1, const Vector3<T> &_v2)
      {
        const T _v1LengthSquared = _v1.SquaredLength();
        if (_v1LengthSquared <= 0.0)
        {
          // zero vector - we can't handle this
          this->Set(1, 0, 0, 0, 1, 0, 0, 0, 1);
          return;
        }

        const T _v2LengthSquared = _v2.SquaredLength();
        if (_v2LengthSquared <= 0.0)
        {
          // zero vector - we can't handle this
          this->Set(1, 0, 0, 0, 1, 0, 0, 0, 1);
          return;
        }

        const T dot = _v1.Dot(_v2) / sqrt(_v1LengthSquared * _v2LengthSquared);
        if (fabs(dot - 1.0) <= 1e-6)
        {
          // the vectors are parallel
          this->Set(1, 0, 0, 0, 1, 0, 0, 0, 1);
          return;
        }
        else if (fabs(dot + 1.0) <= 1e-6)
        {
          // the vectors are opposite
          this->Set(-1, 0, 0, 0, -1, 0, 0, 0, -1);
          return;
        }

        const Vector3<T> cross = _v1.Cross(_v2).Normalize();

        this->SetFromAxisAngle(cross, acos(dot));
      }

      /// \brief Set a column.
      /// \param[in] _c The colum index [0, 1, 2]. _col is clamped to the
      /// range [0, 2].
      /// \param[in] _v The value to set in each row of the column.
      /// \deprecated Use SetCol(unsigned int _c, const Vector3<T> &_v)
      public: void GZ_DEPRECATED(7) Col(unsigned int _c, const Vector3<T> &_v)
      {
        this->SetCol(_c, _v);
      }

      /// \brief Set a column.
      /// \param[in] _c The colum index [0, 1, 2]. _col is clamped to the
      /// range [0, 2].
      /// \param[in] _v The value to set in each row of the column.
      public: void SetCol(unsigned int _c, const Vector3<T> &_v)
      {
        unsigned int c = clamp(_c, 0u, 2u);

        this->data[0][c] = _v.X();
        this->data[1][c] = _v.Y();
        this->data[2][c] = _v.Z();
      }

      /// \brief Equal operator. this = _mat
      /// \param _mat Matrix to copy.
      /// \return This matrix.
      public: Matrix3<T> &operator=(const Matrix3<T> &_mat) = default;

      /// \brief Subtraction operator.
      /// \param[in] _m Matrix to subtract.
      /// \return The element wise difference of two matrices.
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

      /// \brief Addition operation.
      /// \param[in] _m Matrix to add.
      /// \return The element wise sum of two matrices
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

      /// \brief Scalar multiplication operator.
      /// \param[in] _s Value to multiply.
      /// \return The element wise scalar multiplication.
      public: Matrix3<T> operator*(const T &_s) const
      {
        return Matrix3<T>(
          _s * this->data[0][0], _s * this->data[0][1], _s * this->data[0][2],
          _s * this->data[1][0], _s * this->data[1][1], _s * this->data[1][2],
          _s * this->data[2][0], _s * this->data[2][1], _s * this->data[2][2]);
      }

      /// \brief Matrix multiplication operator
      /// \param[in] _m Matrix3<T> to multiply
      /// \return Product of this * _m
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

      /// \brief Multiplication operator with Vector3 on the right
      /// treated like a column vector.
      /// \param _vec Vector3
      /// \return Resulting vector from multiplication
      public: Vector3<T> operator*(const Vector3<T> &_vec) const
      {
        return Vector3<T>(
            this->data[0][0]*_vec.X() + this->data[0][1]*_vec.Y() +
            this->data[0][2]*_vec.Z(),
            this->data[1][0]*_vec.X() + this->data[1][1]*_vec.Y() +
            this->data[1][2]*_vec.Z(),
            this->data[2][0]*_vec.X() + this->data[2][1]*_vec.Y() +
            this->data[2][2]*_vec.Z());
      }

      /// \brief Matrix multiplication operator for scaling.
      /// \param[in] _s Scaling factor.
      /// \param[in] _m Input matrix.
      /// \return A scaled matrix.
      public: friend inline Matrix3<T> operator*(T _s, const Matrix3<T> &_m)
      {
        return _m * _s;
      }

      /// \brief Matrix left multiplication operator for Vector3.
      /// Treats the Vector3 like a row vector multiplying the matrix
      /// from the left.
      /// \param[in] _v Input vector.
      /// \param[in] _m Input matrix.
      /// \return The product vector.
      public: friend inline Vector3<T> operator*(const Vector3<T> &_v,
                                                 const Matrix3<T> &_m)
      {
        return Vector3<T>(
            _m(0, 0)*_v.X() + _m(1, 0)*_v.Y() + _m(2, 0)*_v.Z(),
            _m(0, 1)*_v.X() + _m(1, 1)*_v.Y() + _m(2, 1)*_v.Z(),
            _m(0, 2)*_v.X() + _m(1, 2)*_v.Y() + _m(2, 2)*_v.Z());
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _m the matrix to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the matrices are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const Matrix3 &_m, const T &_tol) const
      {
        return equal<T>(this->data[0][0], _m(0, 0), _tol)
            && equal<T>(this->data[0][1], _m(0, 1), _tol)
            && equal<T>(this->data[0][2], _m(0, 2), _tol)
            && equal<T>(this->data[1][0], _m(1, 0), _tol)
            && equal<T>(this->data[1][1], _m(1, 1), _tol)
            && equal<T>(this->data[1][2], _m(1, 2), _tol)
            && equal<T>(this->data[2][0], _m(2, 0), _tol)
            && equal<T>(this->data[2][1], _m(2, 1), _tol)
            && equal<T>(this->data[2][2], _m(2, 2), _tol);
      }

      /// \brief Equality test operator.
      /// \param[in] _m Matrix3<T> to test.
      /// \return True if equal (using the default tolerance of 1e-6).
      public: bool operator==(const Matrix3<T> &_m) const
      {
        return this->Equal(_m, static_cast<T>(1e-6));
      }

      /// \brief Set as a 3x3 rotation matrix from a quaternion.
      /// \param[in] _q Quaternion to set the matrix3 from.
      /// \return Reference to the new matrix3 object.
      public: Matrix3<T> &operator=(const Quaternion<T> &_q)
      {
        return *this = Matrix3<T>(_q);
      }

      /// \brief Inequality test operator.
      /// \param[in] _m Matrix3<T> to test.
      /// \return True if not equal (using the default tolerance of 1e-6).
      public: bool operator!=(const Matrix3<T> &_m) const
      {
        return !(*this == _m);
      }

      /// \brief Array subscript operator.
      /// \param[in] _row row index. _row is clamped to the range [0,2]
      /// \param[in] _col column index. _col is clamped to the range [0,2]
      /// \return a pointer to the row
      public: inline T operator()(size_t _row, size_t _col) const
      {
        return this->data[clamp(_row, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)]
                         [clamp(_col, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)];
      }

      /// \brief Array subscript operator.
      /// \param[in] _row row index. _row is clamped to the range [0,2]
      /// \param[in] _col column index. _col is clamped to the range [0,2]
      /// \return a pointer to the row
      public: inline T &operator()(size_t _row, size_t _col)
      {
        return this->data[clamp(_row, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)]
                         [clamp(_col, GZ_ZERO_SIZE_T, GZ_TWO_SIZE_T)];
      }

      /// \brief Return the determinant of the matrix.
      /// \return Determinant of this matrix.
      public: T Determinant() const
      {
        T t0 = this->data[2][2]*this->data[1][1]
             - this->data[2][1]*this->data[1][2];

        T t1 = -(this->data[2][2]*this->data[1][0]
                -this->data[2][0]*this->data[1][2]);

        T t2 = this->data[2][1]*this->data[1][0]
             - this->data[2][0]*this->data[1][1];

        return t0 * this->data[0][0]
             + t1 * this->data[0][1]
             + t2 * this->data[0][2];
      }

      /// \brief Return the inverse matrix.
      /// \return Inverse of this matrix.
      public: Matrix3<T> Inverse() const
      {
        T t0 = this->data[2][2]*this->data[1][1] -
                    this->data[2][1]*this->data[1][2];

        T t1 = -(this->data[2][2]*this->data[1][0] -
                      this->data[2][0]*this->data[1][2]);

        T t2 = this->data[2][1]*this->data[1][0] -
                    this->data[2][0]*this->data[1][1];

        T invDet = 1.0 / (t0 * this->data[0][0] +
                               t1 * this->data[0][1] +
                               t2 * this->data[0][2]);

        return invDet * Matrix3<T>(
          t0,
          - (this->data[2][2] * this->data[0][1] -
             this->data[2][1] * this->data[0][2]),
          + (this->data[1][2] * this->data[0][1] -
             this->data[1][1] * this->data[0][2]),
          t1,
          + (this->data[2][2] * this->data[0][0] -
             this->data[2][0] * this->data[0][2]),
          - (this->data[1][2] * this->data[0][0] -
             this->data[1][0] * this->data[0][2]),
          t2,
          - (this->data[2][1] * this->data[0][0] -
             this->data[2][0] * this->data[0][1]),
          + (this->data[1][1] * this->data[0][0] -
             this->data[1][0] * this->data[0][1]));
      }

      /// \brief Transpose this matrix.
      public: void Transpose()
      {
        std::swap(this->data[0][1], this->data[1][0]);
        std::swap(this->data[0][2], this->data[2][0]);
        std::swap(this->data[1][2], this->data[2][1]);
      }

      /// \brief Return the transpose of this matrix.
      /// \return Transpose of this matrix.
      public: Matrix3<T> Transposed() const
      {
        return Matrix3<T>(
          this->data[0][0], this->data[1][0], this->data[2][0],
          this->data[0][1], this->data[1][1], this->data[2][1],
          this->data[0][2], this->data[1][2], this->data[2][2]);
      }

      /// \brief Stream insertion operator. This operator outputs all
      /// 9 scalar values in the matrix separated by a single space.
      /// \param[in, out] _out Output stream.
      /// \param[in] _m Matrix to output.
      /// \return The stream.
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const gz::math::Matrix3<T> &_m)
      {
        for (auto i : {0, 1, 2})
        {
          for (auto j : {0, 1, 2})
          {
            if (!(i == 0 && j == 0))
              _out << " ";

            appendToStream(_out, _m(i, j));
          }
        }

        return _out;
      }

      /// \brief Stream extraction operator. This operator requires 9 space
      /// separated scalar values, such as "1 2 3 4 5 6 7 8 9".
      /// \param [in, out] _in Input stream.
      /// \param [out] _m Matrix3 to read values into.
      /// \return The stream.
      public: friend std::istream &operator>>(
                  std::istream &_in, gz::math::Matrix3<T> &_m)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        T d[9];
        _in >> d[0] >> d[1] >> d[2]
            >> d[3] >> d[4] >> d[5]
            >> d[6] >> d[7] >> d[8];

        if (!_in.fail())
        {
          _m.Set(d[0], d[1], d[2],
                 d[3], d[4], d[5],
                 d[6], d[7], d[8]);
        }
        return _in;
      }

      /// \brief the 3x3 matrix
      private: T data[3][3];
    };

    namespace detail {

      template<typename T>
      constexpr Matrix3<T> gMatrix3Identity(
          1, 0, 0,
          0, 1, 0,
          0, 0, 1);

      template<typename T>
      constexpr Matrix3<T> gMatrix3Zero(
          0, 0, 0,
          0, 0, 0,
          0, 0, 0);

    }  // namespace detail

    template<typename T>
    const Matrix3<T> &Matrix3<T>::Identity = detail::gMatrix3Identity<T>;

    template<typename T>
    const Matrix3<T> &Matrix3<T>::Zero = detail::gMatrix3Zero<T>;

    /// typedef Matrix3<int> as Matrix3i.
    typedef Matrix3<int> Matrix3i;

    /// typedef Matrix3<double> as Matrix3d.
    typedef Matrix3<double> Matrix3d;

    /// typedef Matrix3<float> as Matrix3f.
    typedef Matrix3<float> Matrix3f;
    }
  }
}
#endif
