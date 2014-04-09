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
#ifndef IGN_VECTOR2
#error This class should not be used directly. Use Vector2d.hh, Vector2f.hh, or Vector2i.hh.
#endif

/// \brief Two dimensional (x, y) vector.
class IGN_VECTOR2
{
  /// \brief Default Constructor
  public: IGN_VECTOR2();

  /// \brief Constructor
  /// \param[in] _x value along x
  /// \param[in] _y value along y
  public: IGN_VECTOR2(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y);

  /// \brief Copy constructor
  /// \param[in] _v the value
  public: IGN_VECTOR2(const IGN_VECTOR2 &_v);

  /// \brief Destructor
  public: virtual ~IGN_VECTOR2();

  /// \brief Calc distance to the given point
  /// \param[in] _pt The point to measure to
  /// \return the distance
  public: double Distance(const IGN_VECTOR2 &_pt) const;

  /// \brief  Normalize the vector length
  public: void Normalize();

  /// \brief Set the contents of the vector
  /// \param[in] _x value along x
  /// \param[in] _y value along y
  public: void Set(IGN_NUMERIC _x, IGN_NUMERIC _y);

  /// \brief Return the cross product of this vector and _v
  /// \param[in] _v the vector
  /// \return the cross product
  public: IGN_VECTOR2 Cross(const IGN_VECTOR2 &_v) const;

  /// \brief Assignment operator
  /// \param[in] _v a value for x and y element
  /// \return this
  public: IGN_VECTOR2 &operator =(const IGN_VECTOR2 &_v);

  /// \brief Assignment operator
  /// \param[in] _v the value for x and y element
  /// \return this
  public: const IGN_VECTOR2 &operator =(IGN_NUMERIC _v);

  /// \brief Addition operator
  /// \param[in] _v vector to add
  /// \return sum vector
  public: IGN_VECTOR2 operator+(const IGN_VECTOR2 &_v) const;

  /// \brief Addition assignment operator
  /// \param[in] _v the vector to add
  // \return this
  public: const IGN_VECTOR2 &operator+=(const IGN_VECTOR2 &_v);

  /// \brief Subtraction operator
  /// \param[in] _v the vector to substract
  /// \return the subtracted vector
  public: IGN_VECTOR2 operator-(const IGN_VECTOR2 &_v) const;

  /// \brief Subtraction assignment operator
  /// \param[in] _v the vector to substract
  /// \return this
  public: const IGN_VECTOR2 &operator-=(const IGN_VECTOR2 &_v);

  /// \brief Division operator
  /// \remarks this is an element wise division
  /// \param[in] _v a vector
  /// \result a result
  public: const IGN_VECTOR2 operator/(const IGN_VECTOR2 &_v) const;

  /// \brief Division operator
  /// \remarks this is an element wise division
  /// \param[in] _v a vector
  /// \return this
  public: const IGN_VECTOR2 &operator/=(const IGN_VECTOR2 &_v);

  /// \brief Division operator
  /// \param[in] _v the value
  /// \return a vector
  public: const IGN_VECTOR2 operator/(IGN_NUMERIC _v) const;

  /// \brief Division operator
  /// \param[in] _v the divisor
  /// \return a vector
  public: const IGN_VECTOR2 &operator/=(IGN_NUMERIC _v);

  /// \brief Multiplication operators
  /// \param[in] _v the vector
  /// \return the result
  public: const IGN_VECTOR2 operator*(const IGN_VECTOR2 &_v) const;

  /// \brief Multiplication assignment operator
  /// \remarks this is an element wise multiplication
  /// \param[in] _v the vector
  /// \return this
  public: const IGN_VECTOR2 &operator*=(const IGN_VECTOR2 &_v);

  /// \brief Multiplication operators
  /// \param[in] _v the scaling factor
  /// \return a scaled vector
  public: const IGN_VECTOR2 operator*(IGN_NUMERIC _v) const;

  /// \brief Multiplication assignment operator
  /// \param[in] _v the scaling factor
  /// \return a scaled vector
  public: const IGN_VECTOR2 &operator*=(IGN_NUMERIC _v);

  /// \brief Equal to operator
  /// \param[in] _v the vector to compare to
  /// \return true if the elements of the 2 vectors are equal within
  /// a tolerence (1e-6)
  public: bool operator ==(const IGN_VECTOR2 &_v) const;

  /// \brief Not equal to operator
  /// \return true if elements are of diffent values (tolerence 1e-6)
  public: bool operator!=(const IGN_VECTOR2 &_v) const;

  /// \brief See if a point is finite (e.g., not nan)
  /// \return true if finite, false otherwise
  public: bool IsFinite() const;

  /// \brief Array subscript operator
  /// \param[in] _index the index
  /// \return the value. Throws an IndexException if _index is out of
  /// bounds.
  /// \throws IndexException if _index is >= 2.
  public: inline IGN_NUMERIC operator[](size_t _index) const
          {
            if (_index > 1)
              throw IndexException();
            return this->data[_index];
          }

  /// \brief Return the x value.
  /// \return Value of the X component.
  /// \throws N/A.
  public: inline IGN_NUMERIC x() const
          {return this->data[0];}

  /// \brief Return the y value.
  /// \return Value of the Y component.
  /// \throws N/A.
  public: inline IGN_NUMERIC y() const
          {return this->data[1];}

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

  /// \brief The x and y values.
  private: IGN_NUMERIC data[2];
};

/// \brief Stream extraction operator
/// \param[in] _out output stream
/// \param[in] _pt Vector2 to output
/// \return The stream
/// \throws N/A.
std::ostream &operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR2 &_pt);

/// \brief Stream extraction operator
/// \param[in] _in input stream
/// \param[in] _pt Vector2 to read values into
/// \return The stream
/// \throws N/A.
std::istream &operator>>(std::istream &_in, ignition::math::IGN_VECTOR2 &_pt);
