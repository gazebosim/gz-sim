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
#ifndef IGN_MATRIX4
#error This class should not be used directly. Use IGN_MATRIX4d.hh, \
IGN_MATRIX4f.hh, or IGN_MATRIX4i.hh.
#endif

class IGN_MATRIX3;
class IGN_VECTOR3;
class IGN_POSE3;
class IGN_QUATERNION;

/// \class Matrix4 Matrix4[dfi].hh ignition/math.hh
/// \brief A 4x4 matrix class
class IGNITION_VISIBLE IGN_MATRIX4
{
  /// \brief Identity matrix
  public: static const IGN_MATRIX4 Identity;

  /// \brief Zero matrix
  public: static const IGN_MATRIX4 Zero;

  /// \brief Constructor
  public: IGN_MATRIX4();

  /// \brief Copy constructor
  /// \param _m Matrix to copy
  public: IGN_MATRIX4(const IGN_MATRIX4 &_m);

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
  public: IGN_MATRIX4(
        IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02, IGN_NUMERIC _v03,
        IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12, IGN_NUMERIC _v13,
        IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22, IGN_NUMERIC _v23,
        IGN_NUMERIC _v30, IGN_NUMERIC _v31, IGN_NUMERIC _v32, IGN_NUMERIC _v33);

  /// \brief Destructor
  public: virtual ~IGN_MATRIX4();

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
        IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02, IGN_NUMERIC _v03,
        IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12, IGN_NUMERIC _v13,
        IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22, IGN_NUMERIC _v23,
        IGN_NUMERIC _v30, IGN_NUMERIC _v31, IGN_NUMERIC _v32, IGN_NUMERIC _v33);


  /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
  /// \param[in] _t Values to set
  public: void SetTranslate(const IGN_VECTOR3 &_t);

  /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
  /// \param[in] _x X translation value.
  /// \param[in] _y Y translation value.
  /// \param[in] _z Z translation value.
  public: void SetTranslate(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z);

  /// \brief Get the translational values as a Vector3
  /// \return x,y,z translation values
  public: IGN_VECTOR3 GetTranslation() const;

  /// \brief Get the scale values as a IGN_VECTOR3
  /// \return x,y,z scale values
  public: IGN_VECTOR3 GetScale() const;

  /// \brief Get the rotation as a quaternion
  /// \return the rotation
  public: IGN_QUATERNION GetRotation() const;

  /// \brief Get the rotation as a Euler angles
  /// \param[in] _firstSolution True to get the first Euler solution,
  /// false to get the second.
  /// \return the rotation
  public: IGN_VECTOR3 GetEulerRotation(bool _firstSolution) const;

  /// \brief Get the transformation as math::Pose
  /// \return the pose
  public: IGN_POSE3 GetAsPose() const;

  /// \brief Set the scale
  /// \param[in] _s scale
  public: void SetScale(const IGN_VECTOR3 &_s);

  /// \brief Set the scale
  /// \param[in] _x X scale value.
  /// \param[in] _y Y scale value.
  /// \param[in] _z Z scale value.
  public: void SetScale(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z);

  /// \brief Return true if the matrix is affine
  /// \return true if the matrix is affine, false otherwise
  public: bool IsAffine() const;

  /// \brief Perform an affine transformation
  /// \param _v Vector3 value for the transformation
  /// \return The result of the transformation
  /// \throws AffineException when matrix is not affine.
  public: IGN_VECTOR3 TransformAffine(const IGN_VECTOR3 &_v) const;

  /// \brief Return the inverse matrix.
  /// This is a non-destructive operation.
  /// \return Inverse of this matrix.
  public: IGN_MATRIX4 Inverse() const;

  /// \brief Equal operator. this = _mat
  /// \param _mat Incoming matrix
  /// \return itself
  public: IGN_MATRIX4 &operator=(const IGN_MATRIX4 &_mat);

  /// \brief Equal operator for 3x3 matrix
  /// \param _mat Incoming matrix
  /// \return itself
  public: const IGN_MATRIX4 &operator=(const IGN_MATRIX3 &_mat);

  /// \brief Multiplication operator
  /// \param _mat Incoming matrix
  /// \return This matrix * _mat
  public: IGN_MATRIX4 operator*(const IGN_MATRIX4 &_mat) const;

  /// \brief Multiplication operator
  /// \param _vec Vector3
  /// \return Resulting vector from multiplication
  public: IGN_VECTOR3 operator*(const IGN_VECTOR3 &_vec) const;

  /// \brief Get the value at the specified row, column index
  /// \param[in] _col The column index
  /// \param[in] _row the row index
  /// \return The value at the specified index
  public: inline const IGN_NUMERIC &operator()(size_t _row, size_t _col) const
          {
            if (_row >= 4 || _col >= 4)
              throw IndexException();
            return this->data[_row][_col];
          }

  /// \brief Get a mutable version the value at the specified row, column index
  /// \param[in] _col The column index
  /// \param[in] _row The row index
  /// \return The value at the specified index
  public: inline IGN_NUMERIC &operator()(size_t _row, size_t _col)
          {
            if (_row >= 4 || _col >= 4)
              throw IndexException();
            return this->data[_row][_col];
          }

  /// \brief Equality operator
  /// \param[in] _m Matrix3 to test
  /// \return true if the 2 matrices are equal (using the tolerance 1e-6),
  ///  false otherwise
  public: bool operator==(const IGN_MATRIX4 &_m) const;

  /// \brief The 4x4 matrix
  private: IGN_NUMERIC data[4][4];
};

/// \brief Stream insertion operator
/// \param _out output stream
/// \param _m Matrix to output
/// \return the stream
std::ostream IGNITION_VISIBLE
&operator<<(std::ostream &_out, const ignition::math::IGN_MATRIX4 &_m);

/// \brief Stream extraction operator
/// \param _in input stream
/// \param _pt Matrix4 to read values into
/// \return the stream
std::istream IGNITION_VISIBLE
&operator>>(std::istream &_in, ignition::math::IGN_MATRIX4 &_pt);
