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
#ifndef IGN_MATRIX3
#error This class should not be used directly. Use Matrix3d.hh, \
Matrix3f.hh, or Matrix3i.hh.
#endif

/// \class Matrix3 Matrix3.hh ignition/math.hh
/// \brief A 3x3 matrix class
class IGNITION_VISIBLE IGN_MATRIX3
{
  /// \brief Identity matrix
  public: static const IGN_MATRIX3 Identity;

  /// \brief Zero matrix
  public: static const IGN_MATRIX3 Zero;

  /// \brief Constructor
  public: IGN_MATRIX3();

  /// \brief Copy constructor
  /// \param _m Matrix to copy
  public: IGN_MATRIX3(const IGN_MATRIX3 &_m);

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
  public: IGN_MATRIX3(IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02,
                  IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12,
                  IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22);

  /// \brief Desctructor
  public: virtual ~IGN_MATRIX3();

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
  public: void Set(IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02,
                  IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12,
                  IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22);

  /// \brief Set the matrix from three axis (1 per column)
  /// \param[in] _xAxis The x axis
  /// \param[in] _yAxis The y axis
  /// \param[in] _zAxis The z axis
  public: void SetFromAxes(const IGN_VECTOR3 &_xAxis,
                           const IGN_VECTOR3 &_yAxis,
                           const IGN_VECTOR3 &_zAxis);

  /// \brief Set the matrix from an axis and angle
  /// \param[in] _axis the axis
  /// \param[in] _angle ccw rotation around the axis in radians
  public: void SetFromAxis(const IGN_VECTOR3 &_axis, IGN_NUMERIC _angle);

  /// \brief Set a column
  /// \param[in] _c The colum index (0, 1, 2)
  /// \param[in] _v The value to set in each row of the column
  public: void SetCol(unsigned int _c, const IGN_VECTOR3 &_v);

  /// \brief returns the element wise difference of two matrices
  public: IGN_MATRIX3 operator-(const IGN_MATRIX3 &_m) const;

  /// \brief returns the element wise sum of two matrices
  public: IGN_MATRIX3 operator+(const IGN_MATRIX3 &_m) const;

  /// \brief returns the element wise scalar multiplication
  public: IGN_MATRIX3 operator*(const IGN_NUMERIC &_s) const;

  /// \brief Matrix multiplication operator
  /// \param[in] _m IGN_MATRIX3 to multiply
  /// \return product of this * _m
  public: IGN_MATRIX3 operator*(const IGN_MATRIX3 &_m) const;

  /// \brief Multiplication operator
  /// \param _vec Vector3
  /// \return Resulting vector from multiplication
  public: IGN_VECTOR3 operator*(const IGN_VECTOR3 &_vec) const;

  /// \brief Matrix multiplication operator for scaling.
  /// \param[in] _s Scaling factor.
  /// \param[in] _m Input matrix.
  /// \return A scaled matrix.
  public: friend inline IGN_MATRIX3 operator*(IGN_NUMERIC _s,
                                              const IGN_MATRIX3 &_m)
          { return _m * _s; }

  /// \brief Equality test operator
  /// \param[in] _m IGN_MATRIX3 to test
  /// \return True if equal (using the default tolerance of 1e-6)
  public: bool operator==(const IGN_MATRIX3 &_m) const;

  /// \brief Array subscript operator
  /// \param[in] _row row index
  /// \return a pointer to the row
  public: inline const IGN_NUMERIC &operator()(size_t _row, size_t _col) const
          {
            if (_row >= 3 || _col >= 3)
              throw IndexException();
            return this->data[_row][_col];
          }

  /// \brief Array subscript operator
  /// \param[in] _row row index
  /// \return a pointer to the row
  public: inline IGN_NUMERIC &operator()(size_t _row, size_t _col)
          {
            if (_row >= 3 || _col >=3)
              throw IndexException();
            return this->data[_row][_col];
          }

  /// \brief the 3x3 matrix
  private: IGN_NUMERIC data[3][3];
};

/// \brief Stream insertion operator
/// \param[in] _out Output stream
/// \param[in] _m Matrix to output
/// \return the stream
std::ostream IGNITION_VISIBLE
&operator<<(std::ostream &_out, const ignition::math::IGN_MATRIX3 &_m);

/// \brief Stream extraction operator
/// \param _in input stream
/// \param _pt Matrix3 to read values into
/// \return the stream
std::istream IGNITION_VISIBLE
&operator>>(std::istream &_in, ignition::math::IGN_MATRIX3 &_pt);
