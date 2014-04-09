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
#ifndef IGN_VECTOR4
#error This class should not be used directly. Use Vector4d.hh,\
IGN_VECTOR4f.hh, or IGN_VECTOR4i.hh.
#endif

class IGN_MATRIX4;

/// \class Vector4 Vector4.hh ignitino/math.hh
/// \brief IGN_NUMERIC Generic x, y, z, w vector
class IGN_VECTOR4
{
  /// \brief Constructor
  public: IGN_VECTOR4();

  /// \brief Constructor with component values
  /// \param[in] _x value along x axis
  /// \param[in] _y value along y axis
  /// \param[in] _z value along z axis
  /// \param[in] _w value along w axis
  public: IGN_VECTOR4(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y,
              const IGN_NUMERIC &_z, const IGN_NUMERIC &_w);

  /// \brief Copy constructor
  /// \param[in] _v vector
  public: IGN_VECTOR4(const IGN_VECTOR4 &_v);


  /// \brief Destructor
  public: virtual ~IGN_VECTOR4();

  /// \brief Calc distance to the given point
  /// \param[in] _pt the point
  /// \return the distance
  public: IGN_NUMERIC Distance(const IGN_VECTOR4 &_pt) const;

  /// \brief Returns the length (magnitude) of the vector
  public: IGN_NUMERIC GetLength() const;

  /// \brief Return the square of the length (magnitude) of the vector
  /// \return the length
  public: IGN_NUMERIC GetSquaredLength() const;

  /// \brief Normalize the vector length
  public: void Normalize();

  /// \brief Set the contents of the vector
  /// \param[in] _x value along x axis
  /// \param[in] _y value along y axis
  /// \param[in] _z value along z axis
  /// \param[in] _w value along w axis
  public: void Set(IGN_NUMERIC _x = 0, IGN_NUMERIC _y = 0,
              IGN_NUMERIC _z = 0, IGN_NUMERIC _w = 0);

  /// \brief Assignment operator
  /// \param[in] _v the vector
  /// \return a reference to this vector
  public: IGN_VECTOR4 &operator =(const IGN_VECTOR4 &_v);

  /// \brief Assignment operator
  /// \param[in] _value
  public: IGN_VECTOR4 &operator =(IGN_NUMERIC _value);

  /// \brief Addition operator
  /// \param[in] _v the vector to add
  /// \result a sum vector
  public: IGN_VECTOR4 operator+(const IGN_VECTOR4 &_v) const;

  /// \brief Addition operator
  /// \param[in] _v the vector to add
  /// \return this vector
  public: const IGN_VECTOR4 &operator+=(const IGN_VECTOR4 &_v);

  /// \brief Subtraction operator
  /// \param[in] _v the vector to substract
  /// \return a vector
  public: IGN_VECTOR4 operator-(const IGN_VECTOR4 &_v) const;

  /// \brief Subtraction assigment operators
  /// \param[in] _v the vector to substract
  /// \return this vector
  public: const IGN_VECTOR4 &operator-=(const IGN_VECTOR4 &_v);

  /// \brief Division assignment operator
  /// \remarks Performs element wise division,
  /// which has limited use.
  /// \param[in] _v the vector to perform element wise division with
  /// \return a result vector
  public: const IGN_VECTOR4 operator/(const IGN_VECTOR4 &_v) const;

  /// \brief Division assignment operator
  /// \remarks Performs element wise division,
  /// which has limited use.
  /// \param[in] _v the vector to perform element wise division with
  /// \return this
  public: const IGN_VECTOR4 &operator/=(const IGN_VECTOR4 &_v);

  /// \brief Division assignment operator
  /// \remarks Performs element wise division,
  /// which has limited use.
  /// \param[in] _pt another vector
  /// \return a result vector
  public: const IGN_VECTOR4 operator/(IGN_NUMERIC _v) const;

  /// \brief Division operator
  /// \param[in] _v scaling factor
  /// \return a vector
  public: const IGN_VECTOR4 &operator/=(IGN_NUMERIC _v);

  /// \brief Multiplication operator.
  /// \remarks Performs element wise multiplication,
  /// which has limited use.
  /// \param[in] _pt another vector
  /// \return result vector
  public: const IGN_VECTOR4 operator*(const IGN_VECTOR4 &_pt) const;

  /// \brief Matrix multiplication operator.
  /// \param[in] _m matrix
  /// \return the vector multiplied by _m
  public: const IGN_VECTOR4 operator*(const IGN_MATRIX4 &_m) const;

  /// \brief Multiplication assignment operator
  /// \remarks Performs element wise multiplication,
  /// which has limited use.
  /// \param[in] _pt a vector
  /// \return this
  public: const IGN_VECTOR4 &operator*=(const IGN_VECTOR4 &_pt);

  /// \brief Multiplication operators
  /// \param[in] _v scaling factor
  /// \return a  scaled vector
  public: const IGN_VECTOR4 operator*(IGN_NUMERIC _v) const;

  /// \brief Multiplication assignment operator
  /// \param[in] _v scaling factor
  /// \return this
  public: const IGN_VECTOR4 &operator*=(IGN_NUMERIC _v);

  /// \brief Equal to operator
  /// \param[in] _pt the other vector
  /// \return true if each component is equal withing a
  /// default tolerence (1e-6), false otherwise
  public: bool operator ==(const IGN_VECTOR4 &_pt) const;

  /// \brief Not equal to operator
  /// \param[in] _pt the other vector
  /// \return true if each component is equal withing a
  /// default tolerence (1e-6), false otherwise
  public: bool operator!=(const IGN_VECTOR4 &_pt) const;

  /// \brief See if a point is finite (e.g., not nan)
  /// \return true if finite, false otherwise
  public: bool IsFinite() const;

  /// \brief Array subscript operator
  /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z, 3 == w.
  /// \return The value. Throws an IndexException if _index is out of
  /// bounds.
  /// \throws IndexException if _index is >= 4.
  public: inline IGN_NUMERIC operator[](size_t _index) const
          {
            if (_index > 3)
              throw IndexException();
            return this->data[_index];
          }

  /// \brief Get the x value.
  /// \return The x component of the vector
  public: inline IGN_NUMERIC x() const
          {
            return this->data[0];
          }

  /// \brief Get the y value.
  /// \return The y component of the vector
  public: inline IGN_NUMERIC y() const
          {
            return this->data[1];
          }

  /// \brief Get the z value.
  /// \return The z component of the vector
  public: inline IGN_NUMERIC z() const
          {
            return this->data[2];
          }

  /// \brief Get the w value.
  /// \return The w component of the vector
  public: inline IGN_NUMERIC w() const
          {
            return this->data[3];
          }

  /// \brief Set the x value.
  /// \param[in] _v Value for the x component.
  public: inline void x(const IGN_NUMERIC &_v)
          {
            this->data[0] = _v;
          }

  /// \brief Set the y value.
  /// \param[in] _v Value for the y component.
  public: inline void y(const IGN_NUMERIC &_v)
          {
            this->data[1] = _v;
          }

  /// \brief Set the z value.
  /// \param[in] _v Value for the z component.
  public: inline void z(const IGN_NUMERIC &_v)
          {
            this->data[2] = _v;
          }

  /// \brief Set the w value.
  /// \param[in] _v Value for the w component.
  public: inline void w(const IGN_NUMERIC &_v)
          {
            this->data[3] = _v;
          }

  /// \brief Data values, 0==x, 1==y, 2==z, 3==w
  private: IGN_NUMERIC data[4];
};

/// \brief Stream insertion operator
/// \param[in] _out output stream
/// \param[in] _pt Vector4 to output
/// \return The stream
std::ostream &operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR4 &_pt);

/// \brief Stream extraction operator
/// \param[in] _in input stream
/// \param[in] _pt Vector4 to read values into
/// \return the stream
std::istream &operator>>(std::istream &_in, ignition::math::IGN_VECTOR4 &_pt);
