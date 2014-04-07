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

#include <assert.h>

#include "ignition/math/Vector3d.hh"

namespace ignition
{
  namespace math
  {
    /// \class Matrix3 Matrix3.hh ignition/math.hh
    /// \brief A 3x3 matrix class
    class Matrix3
    {
      /// \brief Constructor
      public: Matrix3();

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix3(const Matrix3 &_m);

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
      public: Matrix3(double _v00, double _v01, double _v02,
                      double _v10, double _v11, double _v12,
                      double _v20, double _v21, double _v22);

      /// \brief Desctructor
      public: virtual ~Matrix3();

      /// \brief Set the matrix from three axis (1 per column)
      /// \param[in] _xAxis The x axis
      /// \param[in] _yAxis The y axis
      /// \param[in] _zAxis The z axis
      public: void SetFromAxes(const Vector3d &_xAxis,
                               const Vector3d &_yAxis,
                               const Vector3d &_zAxis);


      /// \brief Set the matrix from an axis and angle
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      public: void SetFromAxis(const Vector3d &_axis, double _angle);

      /// \brief Set a column
      /// \param[in] _c The colum index (0, 1, 2)
      /// \param[in] _v The value to set in each row of the column
      public: void SetCol(unsigned int _c, const Vector3d &_v);

      /// \brief returns the element wise difference of two matrices
      public: Matrix3 operator-(const Matrix3 &_m) const
      {
        return Matrix3(
        // first row
        this->m[0][0]-_m[0][0], this->m[0][1]-_m[0][1], this->m[0][2]-_m[0][2],
        this->m[1][0]-_m[1][0], this->m[1][1]-_m[1][1], this->m[1][2]-_m[1][2],
        this->m[2][0]-_m[2][0], this->m[2][1]-_m[2][1], this->m[2][2]-_m[2][2]);
      }

      /// \brief returns the element wise sum of two matrices
      public: Matrix3 operator+(const Matrix3 &_m) const
      {
        return Matrix3(
        // first row
        this->m[0][0]+_m[0][0], this->m[0][1]+_m[0][1], this->m[0][2]+_m[0][2],
        this->m[1][0]+_m[1][0], this->m[1][1]+_m[1][1], this->m[1][2]+_m[1][2],
        this->m[2][0]+_m[2][0], this->m[2][1]+_m[2][1], this->m[2][2]+_m[2][2]);
      }

      /// \brief returns the element wise scalar multiplication
      public: Matrix3 operator*(const double &_s) const
      {
        return Matrix3(
          // first row
          _s * this->m[0][0], _s * this->m[0][1], _s * this->m[0][2],
          _s * this->m[1][0], _s * this->m[1][1], _s * this->m[1][2],
          _s * this->m[2][0], _s * this->m[2][1], _s * this->m[2][2]);
      }

      /// \brief Multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _m input matrix
      /// \return a scaled matrix
      public: friend inline Matrix3 operator*(double _s,
                                              const Matrix3 &_m)
      { return _m * _s; }

      /// \brief Matrix multiplication operator
      /// \param[in] _m Matrix3 to multiply
      /// \return product of this * _m
      public: Matrix3 operator*(const Matrix3 &_m) const
      {
        return Matrix3(
          // first row
          this->m[0][0]*_m[0][0]+this->m[0][1]*_m[1][0]+this->m[0][2]*_m[2][0],
          this->m[0][0]*_m[0][1]+this->m[0][1]*_m[1][1]+this->m[0][2]*_m[2][1],
          this->m[0][0]*_m[0][2]+this->m[0][1]*_m[1][2]+this->m[0][2]*_m[2][2],
          // second row
          this->m[1][0]*_m[0][0]+this->m[1][1]*_m[1][0]+this->m[1][2]*_m[2][0],
          this->m[1][0]*_m[0][1]+this->m[1][1]*_m[1][1]+this->m[1][2]*_m[2][1],
          this->m[1][0]*_m[0][2]+this->m[1][1]*_m[1][2]+this->m[1][2]*_m[2][2],
          // third row
          this->m[2][0]*_m[0][0]+this->m[2][1]*_m[1][0]+this->m[2][2]*_m[2][0],
          this->m[2][0]*_m[0][1]+this->m[2][1]*_m[1][1]+this->m[2][2]*_m[2][1],
          this->m[2][0]*_m[0][2]+this->m[2][1]*_m[1][2]+this->m[2][2]*_m[2][2]);
      }

      /// \brief Equality test operator
      /// \param[in] _m Matrix3 to test
      /// \return True if equal (using the default tolerance of 1e-6)
      public: bool operator==(const Matrix3 &_m) const;

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline const double *operator[](size_t _row) const
              {
                assert(_row < 3);
                return this->m[_row];
              }

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline double *operator[](size_t _row)
              {
                assert(_row < 3);
                return this->m[_row];
              }


      /// \brief Stream insertion operator
      /// \param[in] _out Output stream
      /// \param[in] _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const ignition::math::Matrix3 &_m)
            {
              for (int i = 0; i < 3; i++)
              {
                for (int j = 0; j < 3; j++)
                {
                  _out << _m.m[i][j] << " ";
                }
                _out << "\n";
              }

              return _out;
            }

      /// \brief the 3x3 matrix
      protected: double m[3][3];

      friend class Matrix4;
    };
  }
}
#endif
