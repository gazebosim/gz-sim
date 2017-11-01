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
#ifndef IGNITION_MATH_MATRIX4_HH_
#define IGNITION_MATH_MATRIX4_HH_

#include <algorithm>
#include <ignition/math/AffineException.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    /// \class Matrix4 Matrix4.hh ignition/math/Matrix4.hh
    /// \brief A 4x4 matrix class
    template<typename T>
    class Matrix4
    {
      /// \brief Identity matrix
      public: static const Matrix4<T> Identity;

      /// \brief Zero matrix
      public: static const Matrix4<T> Zero;

      /// \brief Constructor
      public: Matrix4()
      {
        memset(this->data, 0, sizeof(this->data[0][0])*16);
      }

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix4(const Matrix4<T> &_m)
      {
        memcpy(this->data, _m.data, sizeof(this->data[0][0])*16);
      }

      /// \brief Constructor
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v03 Row 0, Col 3 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v13 Row 1, Col 3 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      /// \param[in] _v23 Row 2, Col 3 value
      /// \param[in] _v30 Row 3, Col 0 value
      /// \param[in] _v31 Row 3, Col 1 value
      /// \param[in] _v32 Row 3, Col 2 value
      /// \param[in] _v33 Row 3, Col 3 value
      public: Matrix4(T _v00, T _v01, T _v02, T _v03,
                      T _v10, T _v11, T _v12, T _v13,
                      T _v20, T _v21, T _v22, T _v23,
                      T _v30, T _v31, T _v32, T _v33)
      {
        this->Set(_v00, _v01, _v02, _v03,
                  _v10, _v11, _v12, _v13,
                  _v20, _v21, _v22, _v23,
                  _v30, _v31, _v32, _v33);
      }

      /// \brief Construct Matrix4 from a quaternion.
      /// \param[in] _q Quaternion.
      // cppcheck-suppress noExplicitConstructor
      public: Matrix4(const Quaternion<T> &_q)
      {
        Quaternion<T> qt = _q;
        qt.Normalize();
        this->Set(1 - 2*qt.Y()*qt.Y() - 2 *qt.Z()*qt.Z(),
                  2 * qt.X()*qt.Y() - 2*qt.Z()*qt.W(),
                  2 * qt.X() * qt.Z() + 2 * qt.Y() * qt.W(),
                  0,

                  2 * qt.X() * qt.Y() + 2 * qt.Z() * qt.W(),
                  1 - 2*qt.X()*qt.X() - 2 * qt.Z()*qt.Z(),
                  2 * qt.Y() * qt.Z() - 2 * qt.X() * qt.W(),
                  0,

                  2 * qt.X() * qt.Z() - 2 * qt.Y() * qt.W(),
                  2 * qt.Y() * qt.Z() + 2 * qt.X() * qt.W(),
                  1 - 2 * qt.X()*qt.X() - 2 * qt.Y()*qt.Y(),
                  0,

                  0, 0, 0, 1);
      }

      /// \brief Construct Matrix4 from a math::Pose3
      /// \param[in] _pose Pose.
      // cppcheck-suppress noExplicitConstructor
      public: Matrix4(const Pose3<T> &_pose) : Matrix4(_pose.Rot())
      {
        this->Translate(_pose.Pos());
      }

      /// \brief Destructor
      public: virtual ~Matrix4() {}

      /// \brief Change the values
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v03 Row 0, Col 3 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v13 Row 1, Col 3 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      /// \param[in] _v23 Row 2, Col 3 value
      /// \param[in] _v30 Row 3, Col 0 value
      /// \param[in] _v31 Row 3, Col 1 value
      /// \param[in] _v32 Row 3, Col 2 value
      /// \param[in] _v33 Row 3, Col 3 value
      public: void Set(
            T _v00, T _v01, T _v02, T _v03,
            T _v10, T _v11, T _v12, T _v13,
            T _v20, T _v21, T _v22, T _v23,
            T _v30, T _v31, T _v32, T _v33)
      {
        this->data[0][0] = _v00;
        this->data[0][1] = _v01;
        this->data[0][2] = _v02;
        this->data[0][3] = _v03;

        this->data[1][0] = _v10;
        this->data[1][1] = _v11;
        this->data[1][2] = _v12;
        this->data[1][3] = _v13;

        this->data[2][0] = _v20;
        this->data[2][1] = _v21;
        this->data[2][2] = _v22;
        this->data[2][3] = _v23;

        this->data[3][0] = _v30;
        this->data[3][1] = _v31;
        this->data[3][2] = _v32;
        this->data[3][3] = _v33;
      }

      /// \brief Set the upper-left 3x3 matrix from an axis and angle
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      public: void Axis(const Vector3<T> &_axis, T _angle)
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

      /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
      /// \param[in] _t Values to set
      public: void Translate(const Vector3<T> &_t)
      {
        this->data[0][3] = _t.X();
        this->data[1][3] = _t.Y();
        this->data[2][3] = _t.Z();
      }

      /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
      /// \param[in] _x X translation value.
      /// \param[in] _y Y translation value.
      /// \param[in] _z Z translation value.
      public: void Translate(T _x, T _y, T _z)
      {
        this->data[0][3] = _x;
        this->data[1][3] = _y;
        this->data[2][3] = _z;
      }

      /// \brief Get the translational values as a Vector3
      /// \return x,y,z translation values
      public: Vector3<T> Translation() const
      {
        return Vector3<T>(this->data[0][3], this->data[1][3], this->data[2][3]);
      }

      /// \brief Get the scale values as a Vector3<T>
      /// \return x,y,z scale values
      public: Vector3<T> Scale() const
      {
        return Vector3<T>(this->data[0][0], this->data[1][1], this->data[2][2]);
      }

      /// \brief Get the rotation as a quaternion
      /// \return the rotation
      public: Quaternion<T> Rotation() const
      {
        Quaternion<T> q;
        /// algorithm from Ogre::Quaternion<T> source, which in turn is based on
        /// Ken Shoemake's article "Quaternion<T> Calculus and Fast Animation".
        T trace = this->data[0][0] + this->data[1][1] + this->data[2][2];
        T root;
        if (trace > 0)
        {
          root = sqrt(trace + 1.0);
          q.W(root / 2.0);
          root = 1.0 / (2.0 * root);
          q.X((this->data[2][1] - this->data[1][2]) * root);
          q.Y((this->data[0][2] - this->data[2][0]) * root);
          q.Z((this->data[1][0] - this->data[0][1]) * root);
        }
        else
        {
          static unsigned int s_iNext[3] = {1, 2, 0};
          unsigned int i = 0;
          if (this->data[1][1] > this->data[0][0])
            i = 1;
          if (this->data[2][2] > this->data[i][i])
            i = 2;
          unsigned int j = s_iNext[i];
          unsigned int k = s_iNext[j];

          root = sqrt(this->data[i][i] - this->data[j][j] -
                      this->data[k][k] + 1.0);

          T a, b, c;
          a = root / 2.0;
          root = 1.0 / (2.0 * root);
          b = (this->data[j][i] + this->data[i][j]) * root;
          c = (this->data[k][i] + this->data[i][k]) * root;

          switch (i)
          {
            default:
            case 0: q.X(a); break;
            case 1: q.Y(a); break;
            case 2: q.Z(a); break;
          };
          switch (j)
          {
            default:
            case 0: q.X(b); break;
            case 1: q.Y(b); break;
            case 2: q.Z(b); break;
          };
          switch (k)
          {
            default:
            case 0: q.X(c); break;
            case 1: q.Y(c); break;
            case 2: q.Z(c); break;
          };

          q.W((this->data[k][j] - this->data[j][k]) * root);
        }

        return q;
      }

      /// \brief Get the rotation as a Euler angles
      /// \param[in] _firstSolution True to get the first Euler solution,
      /// false to get the second.
      /// \return the rotation
      public: Vector3<T> EulerRotation(bool _firstSolution) const
      {
        Vector3<T> euler;
        Vector3<T> euler2;

        T m31 = this->data[2][0];
        T m11 = this->data[0][0];
        T m12 = this->data[0][1];
        T m13 = this->data[0][2];
        T m32 = this->data[2][1];
        T m33 = this->data[2][2];
        T m21 = this->data[1][0];

        if (std::abs(m31) >= 1.0)
        {
          euler.Z(0.0);
          euler2.Z(0.0);

          if (m31 < 0.0)
          {
            euler.Y(IGN_PI / 2.0);
            euler2.Y(IGN_PI / 2.0);
            euler.X(atan2(m12, m13));
            euler2.X(atan2(m12, m13));
          }
          else
          {
            euler.Y(-IGN_PI / 2.0);
            euler2.Y(-IGN_PI / 2.0);
            euler.X(atan2(-m12, -m13));
            euler2.X(atan2(-m12, -m13));
          }
        }
        else
        {
          euler.Y(-asin(m31));
          euler2.Y(IGN_PI - euler.Y());

          euler.X(atan2(m32 / cos(euler.Y()), m33 / cos(euler.Y())));
          euler2.X(atan2(m32 / cos(euler2.Y()), m33 / cos(euler2.Y())));

          euler.Z(atan2(m21 / cos(euler.Y()), m11 / cos(euler.Y())));
          euler2.Z(atan2(m21 / cos(euler2.Y()), m11 / cos(euler2.Y())));
        }

        if (_firstSolution)
          return euler;
        else
          return euler2;
      }

      /// \brief Get the transformation as math::Pose
      /// \return the pose
      public: Pose3<T> Pose() const
      {
        return Pose3<T>(this->Translation(), this->Rotation());
      }

      /// \brief Set the scale
      /// \param[in] _s scale
      public: void Scale(const Vector3<T> &_s)
      {
        this->data[0][0] = _s.X();
        this->data[1][1] = _s.Y();
        this->data[2][2] = _s.Z();
        this->data[3][3] = 1.0;
      }

      /// \brief Set the scale
      /// \param[in] _x X scale value.
      /// \param[in] _y Y scale value.
      /// \param[in] _z Z scale value.
      public: void Scale(T _x, T _y, T _z)
      {
        this->data[0][0] = _x;
        this->data[1][1] = _y;
        this->data[2][2] = _z;
        this->data[3][3] = 1.0;
      }

      /// \brief Return true if the matrix is affine
      /// \return true if the matrix is affine, false otherwise
      public: bool IsAffine() const
      {
        return equal(this->data[3][0], static_cast<T>(0)) &&
          equal(this->data[3][1], static_cast<T>(0)) &&
          equal(this->data[3][2], static_cast<T>(0)) &&
          equal(this->data[3][3], static_cast<T>(1));
      }

      /// \brief Perform an affine transformation
      /// \param _v Vector3 value for the transformation
      /// \return The result of the transformation
      /// \throws AffineException when matrix is not affine.
      public: Vector3<T> TransformAffine(const Vector3<T> &_v) const
      {
        if (!this->IsAffine())
          throw AffineException();

        return Vector3<T>(this->data[0][0]*_v.X() + this->data[0][1]*_v.Y() +
                           this->data[0][2]*_v.Z() + this->data[0][3],
                           this->data[1][0]*_v.X() + this->data[1][1]*_v.Y() +
                           this->data[1][2]*_v.Z() + this->data[1][3],
                           this->data[2][0]*_v.X() + this->data[2][1]*_v.Y() +
                           this->data[2][2]*_v.Z() + this->data[2][3]);
      }

      /// \brief Return the determinant of the matrix
      /// \return Determinant of this matrix.
      public: T Determinant() const
      {
        T v0, v1, v2, v3, v4, v5, t00, t10, t20, t30;

        v0 = this->data[2][0]*this->data[3][1]
           - this->data[2][1]*this->data[3][0];
        v1 = this->data[2][0]*this->data[3][2]
           - this->data[2][2]*this->data[3][0];
        v2 = this->data[2][0]*this->data[3][3]
           - this->data[2][3]*this->data[3][0];
        v3 = this->data[2][1]*this->data[3][2]
           - this->data[2][2]*this->data[3][1];
        v4 = this->data[2][1]*this->data[3][3]
           - this->data[2][3]*this->data[3][1];
        v5 = this->data[2][2]*this->data[3][3]
           - this->data[2][3]*this->data[3][2];

        t00 =  v5*this->data[1][1] - v4*this->data[1][2] + v3*this->data[1][3];
        t10 = -v5*this->data[1][0] + v2*this->data[1][2] - v1*this->data[1][3];
        t20 =  v4*this->data[1][0] - v2*this->data[1][1] + v0*this->data[1][3];
        t30 = -v3*this->data[1][0] + v1*this->data[1][1] - v0*this->data[1][2];

        return t00 * this->data[0][0]
             + t10 * this->data[0][1]
             + t20 * this->data[0][2]
             + t30 * this->data[0][3];
      }

      /// \brief Return the inverse matrix.
      /// This is a non-destructive operation.
      /// \return Inverse of this matrix.
      public: Matrix4<T> Inverse() const
      {
        T v0, v1, v2, v3, v4, v5, t00, t10, t20, t30;
        Matrix4<T> r;

        v0 = this->data[2][0]*this->data[3][1] -
          this->data[2][1]*this->data[3][0];
        v1 = this->data[2][0]*this->data[3][2] -
          this->data[2][2]*this->data[3][0];
        v2 = this->data[2][0]*this->data[3][3] -
          this->data[2][3]*this->data[3][0];
        v3 = this->data[2][1]*this->data[3][2] -
          this->data[2][2]*this->data[3][1];
        v4 = this->data[2][1]*this->data[3][3] -
          this->data[2][3]*this->data[3][1];
        v5 = this->data[2][2]*this->data[3][3] -
          this->data[2][3]*this->data[3][2];

        t00 = +(v5*this->data[1][1] -
            v4*this->data[1][2] + v3*this->data[1][3]);
        t10 = -(v5*this->data[1][0] -
            v2*this->data[1][2] + v1*this->data[1][3]);
        t20 = +(v4*this->data[1][0] -
            v2*this->data[1][1] + v0*this->data[1][3]);
        t30 = -(v3*this->data[1][0] -
            v1*this->data[1][1] + v0*this->data[1][2]);

        T invDet = 1 / (t00 * this->data[0][0] + t10 * this->data[0][1] +
            t20 * this->data[0][2] + t30 * this->data[0][3]);

        r(0, 0) = t00 * invDet;
        r(1, 0) = t10 * invDet;
        r(2, 0) = t20 * invDet;
        r(3, 0) = t30 * invDet;

        r(0, 1) = -(v5*this->data[0][1] -
            v4*this->data[0][2] + v3*this->data[0][3]) * invDet;
        r(1, 1) = +(v5*this->data[0][0] -
            v2*this->data[0][2] + v1*this->data[0][3]) * invDet;
        r(2, 1) = -(v4*this->data[0][0] -
            v2*this->data[0][1] + v0*this->data[0][3]) * invDet;
        r(3, 1) = +(v3*this->data[0][0] -
            v1*this->data[0][1] + v0*this->data[0][2]) * invDet;

        v0 = this->data[1][0]*this->data[3][1] -
          this->data[1][1]*this->data[3][0];
        v1 = this->data[1][0]*this->data[3][2] -
          this->data[1][2]*this->data[3][0];
        v2 = this->data[1][0]*this->data[3][3] -
          this->data[1][3]*this->data[3][0];
        v3 = this->data[1][1]*this->data[3][2] -
          this->data[1][2]*this->data[3][1];
        v4 = this->data[1][1]*this->data[3][3] -
          this->data[1][3]*this->data[3][1];
        v5 = this->data[1][2]*this->data[3][3] -
          this->data[1][3]*this->data[3][2];

        r(0, 2) = +(v5*this->data[0][1] -
            v4*this->data[0][2] + v3*this->data[0][3]) * invDet;
        r(1, 2) = -(v5*this->data[0][0] -
            v2*this->data[0][2] + v1*this->data[0][3]) * invDet;
        r(2, 2) = +(v4*this->data[0][0] -
            v2*this->data[0][1] + v0*this->data[0][3]) * invDet;
        r(3, 2) = -(v3*this->data[0][0] -
            v1*this->data[0][1] + v0*this->data[0][2]) * invDet;

        v0 = this->data[2][1]*this->data[1][0] -
          this->data[2][0]*this->data[1][1];
        v1 = this->data[2][2]*this->data[1][0] -
          this->data[2][0]*this->data[1][2];
        v2 = this->data[2][3]*this->data[1][0] -
          this->data[2][0]*this->data[1][3];
        v3 = this->data[2][2]*this->data[1][1] -
          this->data[2][1]*this->data[1][2];
        v4 = this->data[2][3]*this->data[1][1] -
          this->data[2][1]*this->data[1][3];
        v5 = this->data[2][3]*this->data[1][2] -
          this->data[2][2]*this->data[1][3];

        r(0, 3) = -(v5*this->data[0][1] -
            v4*this->data[0][2] + v3*this->data[0][3]) * invDet;
        r(1, 3) = +(v5*this->data[0][0] -
            v2*this->data[0][2] + v1*this->data[0][3]) * invDet;
        r(2, 3) = -(v4*this->data[0][0] -
            v2*this->data[0][1] + v0*this->data[0][3]) * invDet;
        r(3, 3) = +(v3*this->data[0][0] -
            v1*this->data[0][1] + v0*this->data[0][2]) * invDet;

        return r;
      }

      /// \brief Transpose this matrix.
      public: void Transpose()
      {
        std::swap(this->data[0][1], this->data[1][0]);
        std::swap(this->data[0][2], this->data[2][0]);
        std::swap(this->data[0][3], this->data[3][0]);
        std::swap(this->data[1][2], this->data[2][1]);
        std::swap(this->data[1][3], this->data[3][1]);
        std::swap(this->data[2][3], this->data[3][2]);
      }

      /// \brief Return the transpose of this matrix
      /// \return Transpose of this matrix.
      public: Matrix4<T> Transposed() const
      {
        return Matrix4<T>(
        this->data[0][0], this->data[1][0], this->data[2][0], this->data[3][0],
        this->data[0][1], this->data[1][1], this->data[2][1], this->data[3][1],
        this->data[0][2], this->data[1][2], this->data[2][2], this->data[3][2],
        this->data[0][3], this->data[1][3], this->data[2][3], this->data[3][3]);
      }

      /// \brief Equal operator. this = _mat
      /// \param _mat Incoming matrix
      /// \return itself
      public: Matrix4<T> &operator=(const Matrix4<T> &_mat)
      {
        memcpy(this->data, _mat.data, sizeof(this->data[0][0])*16);
        return *this;
      }

      /// \brief Equal operator for 3x3 matrix
      /// \param _mat Incoming matrix
      /// \return itself
      public: const Matrix4<T> &operator=(const Matrix3<T> &_mat)
      {
        this->data[0][0] = _mat(0, 0);
        this->data[0][1] = _mat(0, 1);
        this->data[0][2] = _mat(0, 2);

        this->data[1][0] = _mat(1, 0);
        this->data[1][1] = _mat(1, 1);
        this->data[1][2] = _mat(1, 2);

        this->data[2][0] = _mat(2, 0);
        this->data[2][1] = _mat(2, 1);
        this->data[2][2] = _mat(2, 2);

        return *this;
      }

      /// \brief Multiplication operator
      /// \param _mat Incoming matrix
      /// \return This matrix * _mat
      public: Matrix4<T> operator*(const Matrix4<T> &_m2) const
      {
        return Matrix4<T>(
          this->data[0][0] * _m2(0, 0) +
          this->data[0][1] * _m2(1, 0) +
          this->data[0][2] * _m2(2, 0) +
          this->data[0][3] * _m2(3, 0),

          this->data[0][0] * _m2(0, 1) +
          this->data[0][1] * _m2(1, 1) +
          this->data[0][2] * _m2(2, 1) +
          this->data[0][3] * _m2(3, 1),

          this->data[0][0] * _m2(0, 2) +
          this->data[0][1] * _m2(1, 2) +
          this->data[0][2] * _m2(2, 2) +
          this->data[0][3] * _m2(3, 2),

          this->data[0][0] * _m2(0, 3) +
          this->data[0][1] * _m2(1, 3) +
          this->data[0][2] * _m2(2, 3) +
          this->data[0][3] * _m2(3, 3),

          this->data[1][0] * _m2(0, 0) +
          this->data[1][1] * _m2(1, 0) +
          this->data[1][2] * _m2(2, 0) +
          this->data[1][3] * _m2(3, 0),

          this->data[1][0] * _m2(0, 1) +
          this->data[1][1] * _m2(1, 1) +
          this->data[1][2] * _m2(2, 1) +
          this->data[1][3] * _m2(3, 1),

          this->data[1][0] * _m2(0, 2) +
          this->data[1][1] * _m2(1, 2) +
          this->data[1][2] * _m2(2, 2) +
          this->data[1][3] * _m2(3, 2),

          this->data[1][0] * _m2(0, 3) +
          this->data[1][1] * _m2(1, 3) +
          this->data[1][2] * _m2(2, 3) +
          this->data[1][3] * _m2(3, 3),

          this->data[2][0] * _m2(0, 0) +
          this->data[2][1] * _m2(1, 0) +
          this->data[2][2] * _m2(2, 0) +
          this->data[2][3] * _m2(3, 0),

          this->data[2][0] * _m2(0, 1) +
          this->data[2][1] * _m2(1, 1) +
          this->data[2][2] * _m2(2, 1) +
          this->data[2][3] * _m2(3, 1),

          this->data[2][0] * _m2(0, 2) +
          this->data[2][1] * _m2(1, 2) +
          this->data[2][2] * _m2(2, 2) +
          this->data[2][3] * _m2(3, 2),

          this->data[2][0] * _m2(0, 3) +
          this->data[2][1] * _m2(1, 3) +
          this->data[2][2] * _m2(2, 3) +
          this->data[2][3] * _m2(3, 3),

          this->data[3][0] * _m2(0, 0) +
          this->data[3][1] * _m2(1, 0) +
          this->data[3][2] * _m2(2, 0) +
          this->data[3][3] * _m2(3, 0),

          this->data[3][0] * _m2(0, 1) +
          this->data[3][1] * _m2(1, 1) +
          this->data[3][2] * _m2(2, 1) +
          this->data[3][3] * _m2(3, 1),

          this->data[3][0] * _m2(0, 2) +
          this->data[3][1] * _m2(1, 2) +
          this->data[3][2] * _m2(2, 2) +
          this->data[3][3] * _m2(3, 2),

          this->data[3][0] * _m2(0, 3) +
          this->data[3][1] * _m2(1, 3) +
          this->data[3][2] * _m2(2, 3) +
          this->data[3][3] * _m2(3, 3));
      }

      /// \brief Multiplication operator
      /// \param _vec Vector3
      /// \return Resulting vector from multiplication
      public: Vector3<T> operator*(const Vector3<T> &_vec) const
      {
        return Vector3<T>(
            this->data[0][0]*_vec.X() + this->data[0][1]*_vec.Y() +
            this->data[0][2]*_vec.Z() + this->data[0][3],
            this->data[1][0]*_vec.X() + this->data[1][1]*_vec.Y() +
            this->data[1][2]*_vec.Z() + this->data[1][3],
            this->data[2][0]*_vec.X() + this->data[2][1]*_vec.Y() +
            this->data[2][2]*_vec.Z() + this->data[2][3]);
      }

      /// \brief Get the value at the specified row, column index
      /// \param[in] _col The column index
      /// \param[in] _row the row index
      /// \return The value at the specified index
      public: inline const T &operator()(size_t _row, size_t _col) const
      {
        if (_row >= 4 || _col >= 4)
          throw IndexException();
        return this->data[_row][_col];
      }

      /// \brief Get a mutable version the value at the specified row,
      /// column index
      /// \param[in] _col The column index
      /// \param[in] _row The row index
      /// \return The value at the specified index
      public: inline T &operator()(size_t _row, size_t _col)
      {
        if (_row >= 4 || _col >= 4)
          throw IndexException();
        return this->data[_row][_col];
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _m the matrix to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the matrices are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const Matrix4 &_m, const T &_tol) const
      {
        return equal<T>(this->data[0][0], _m(0, 0), _tol)
            && equal<T>(this->data[0][1], _m(0, 1), _tol)
            && equal<T>(this->data[0][2], _m(0, 2), _tol)
            && equal<T>(this->data[0][3], _m(0, 3), _tol)
            && equal<T>(this->data[1][0], _m(1, 0), _tol)
            && equal<T>(this->data[1][1], _m(1, 1), _tol)
            && equal<T>(this->data[1][2], _m(1, 2), _tol)
            && equal<T>(this->data[1][3], _m(1, 3), _tol)
            && equal<T>(this->data[2][0], _m(2, 0), _tol)
            && equal<T>(this->data[2][1], _m(2, 1), _tol)
            && equal<T>(this->data[2][2], _m(2, 2), _tol)
            && equal<T>(this->data[2][3], _m(2, 3), _tol)
            && equal<T>(this->data[3][0], _m(3, 0), _tol)
            && equal<T>(this->data[3][1], _m(3, 1), _tol)
            && equal<T>(this->data[3][2], _m(3, 2), _tol)
            && equal<T>(this->data[3][3], _m(3, 3), _tol);
      }

      /// \brief Equality operator
      /// \param[in] _m Matrix3 to test
      /// \return true if the 2 matrices are equal (using the tolerance 1e-6),
      ///  false otherwise
      public: bool operator==(const Matrix4<T> &_m) const
      {
        return this->Equal(_m, static_cast<T>(1e-6));
      }

      /// \brief Inequality test operator
      /// \param[in] _m Matrix4<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const Matrix4<T> &_m) const
      {
        return !(*this == _m);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Matrix4<T> &_m)
      {
        _out << precision(_m(0, 0), 6) << " "
             << precision(_m(0, 1), 6) << " "
             << precision(_m(0, 2), 6) << " "
             << precision(_m(0, 3), 6) << " "
             << precision(_m(1, 0), 6) << " "
             << precision(_m(1, 1), 6) << " "
             << precision(_m(1, 2), 6) << " "
             << precision(_m(1, 3), 6) << " "
             << precision(_m(2, 0), 6) << " "
             << precision(_m(2, 1), 6) << " "
             << precision(_m(2, 2), 6) << " "
             << precision(_m(2, 3), 6) << " "
             << precision(_m(3, 0), 6) << " "
             << precision(_m(3, 1), 6) << " "
             << precision(_m(3, 2), 6) << " "
             << precision(_m(3, 3), 6);

        return _out;
      }

      /// \brief Stream extraction operator
      /// \param _in input stream
      /// \param _pt Matrix4<T> to read values into
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, ignition::math::Matrix4<T> &_m)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        T d[16];
        _in >> d[0] >> d[1] >> d[2] >> d[3]
            >> d[4] >> d[5] >> d[6] >> d[7]
            >> d[8] >> d[9] >> d[10] >> d[11]
            >> d[12] >> d[13] >> d[14] >> d[15];

        _m.Set(d[0], d[1], d[2], d[3],
               d[4], d[5], d[6], d[7],
               d[8], d[9], d[10], d[11],
               d[12], d[13], d[14], d[15]);
        return _in;
      }

      /// \brief Get transform which translates to _eye and rotates the X axis
      /// so it faces the _target. The rotation is such that Z axis is in the
      /// _up direction, if possible. The coordinate system is right-handed,
      /// \param[in] _eye Coordinate frame translation.
      /// \param[in] _target Point which the X axis should face. If _target is
      /// equal to _eye, the X axis won't be rotated.
      /// \param[in] _up Direction in which the Z axis should point. If _up is
      /// zero or parallel to X, it will be set to +Z.
      /// \return Transformation matrix.
      public: static Matrix4<T> LookAt(const Vector3<T> &_eye,
          const Vector3<T> &_target, const Vector3<T> &_up = Vector3<T>::UnitZ)
      {
        // Most important constraint: direction to point X axis at
        auto front = _target - _eye;

        // Case when _eye == _target
        if (front == Vector3<T>::Zero)
          front = Vector3<T>::UnitX;
        front.Normalize();

        // Desired direction to point Z axis at
        auto up = _up;

        // Case when _up == Zero
        if (up == Vector3<T>::Zero)
          up = Vector3<T>::UnitZ;
        else
          up.Normalize();

        // Case when _up is parallel to X
        if (up.Cross(Vector3<T>::UnitX) == Vector3<T>::Zero)
          up = Vector3<T>::UnitZ;

        // Find direction to point Y axis at
        auto left = up.Cross(front);

        // Case when front is parallel to up
        if (left == Vector3<T>::Zero)
          left = Vector3<T>::UnitY;
        else
          left.Normalize();

        // Fix up direction so it's perpendicular to XY
        up = (front.Cross(left)).Normalize();

        return Matrix4<T>(
            front.X(), left.X(), up.X(), _eye.X(),
            front.Y(), left.Y(), up.Y(), _eye.Y(),
            front.Z(), left.Z(), up.Z(), _eye.Z(),
                  0,      0,         0,        1);
      }

      /// \brief The 4x4 matrix
      private: T data[4][4];
    };

    template<typename T>
    const Matrix4<T> Matrix4<T>::Identity(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    template<typename T>
    const Matrix4<T> Matrix4<T>::Zero(
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0);

    typedef Matrix4<int> Matrix4i;
    typedef Matrix4<double> Matrix4d;
    typedef Matrix4<float> Matrix4f;
  }
}
#endif
