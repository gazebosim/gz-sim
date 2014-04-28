/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef IGN_VECTOR3
#error This class should not be used directly. Use Vector3d.hh, \
Vector3f.hh, or Vector3i.hh.
#endif

#include <cmath>

/// \brief The Vector3 class represents the generic vector containing 3
/// elements.  Since it's commonly used to keep coordinate system
/// related information, its elements are labeled by x, y, z.
class IGNITION_VISIBLE IGN_VECTOR3
{
  /// \brief math::IGN_VECTOR3(0, 0, 0)
  public: static const IGN_VECTOR3 Zero;

  /// \brief math::IGN_VECTOR3(1, 1, 1)
  public: static const IGN_VECTOR3 One;

  /// \brief math::IGN_VECTOR3(1, 0, 0)
  public: static const IGN_VECTOR3 UnitX;

  /// \brief math::IGN_VECTOR3(0, 1, 0)
  public: static const IGN_VECTOR3 UnitY;

  /// \brief math::IGN_VECTOR3(0, 0, 1)
  public: static const IGN_VECTOR3 UnitZ;

  /// \brief Constructor
  public: IGN_VECTOR3();

  /// \brief Constructor
  /// \param[in] _x value along x
  /// \param[in] _y value along y
  /// \param[in] _z value along z
  public: IGN_VECTOR3(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y,
                      const IGN_NUMERIC &_z);

  /// \brief Copy constructor
  /// \param[in] _v a vector
  public: IGN_VECTOR3(const IGN_VECTOR3 &_v);

  /// \brief Destructor
  public: virtual ~IGN_VECTOR3();

  /// \brief Return the sum of the values
  /// \return the sum
  public: IGN_NUMERIC GetSum() const;

  /// \brief Calc distance to the given point
  /// \param[in] _pt the point
  /// \return the distance
  public: IGN_NUMERIC Distance(const IGN_VECTOR3 &_pt) const;

  /// \brief Calc distance to the given point
  /// \param[in] _x value along x
  /// \param[in] _y value along y
  /// \param[in] _z value along z
  /// \return the distance
  public: IGN_NUMERIC Distance(IGN_NUMERIC _x, IGN_NUMERIC _y,
                               IGN_NUMERIC _z) const;

  /// \brief Returns the length (magnitude) of the vector
  /// \ return the length
  public: IGN_NUMERIC GetLength() const;

  /// \brief Return the square of the length (magnitude) of the vector
  /// \return the squared length
  public: IGN_NUMERIC GetSquaredLength() const;

  /// \brief Normalize the vector length
  /// \return unit length vector
  public: IGN_VECTOR3 Normalize();

  /// \brief Round to near whole number, return the result.
  /// \return the result
  public: IGN_VECTOR3 Round();

  /// \brief Get a rounded version of this vector
  /// \return a rounded vector
  public: IGN_VECTOR3 GetRounded() const;

  /// \brief Set the contents of the vector
  /// \param[in] _x value along x
  /// \param[in] _y value along y
  /// \param[in] _z value aling z
  public: inline void Set(IGN_NUMERIC _x = 0, IGN_NUMERIC _y = 0,
                          IGN_NUMERIC _z = 0)
          {
            this->data[0] = _x;
            this->data[1] = _y;
            this->data[2] = _z;
          }

  /// \brief Return the cross product of this vector and pt
  /// \return the product
  public: IGN_VECTOR3 Cross(const IGN_VECTOR3 &_pt) const;

  /// \brief Return the dot product of this vector and pt
  /// \return the product
  public: IGN_NUMERIC Dot(const IGN_VECTOR3 &_pt) const;

  /// \brief Get the absolute value of the vector
  /// \return a vector with positive elements
  public: IGN_VECTOR3 GetAbs() const;

  /// \brief Return a vector that is perpendicular to this one.
  /// \return an orthogonal vector
  public: IGN_VECTOR3 GetPerpendicular() const;

  /// \brief Get a normal vector to a triangle
  /// \param[in] _v1 first vertex of the triangle
  /// \param[in] _v2 second vertex
  /// \param[in] _v3 third vertex
  /// \return the normal
  public: static IGN_VECTOR3 GetNormal(const IGN_VECTOR3 &_v1,
              const IGN_VECTOR3 &_v2, const IGN_VECTOR3 &_v3);

  /// \brief Get distance to a line
  /// \param[in] _pt1 first point on the line
  /// \param[in] _pt2 second point on the line
  /// \return the minimum distance from this point to the line
  public: IGN_NUMERIC GetDistToLine(const IGN_VECTOR3 &_pt1,
              const IGN_VECTOR3 &_pt2);

  /// \brief Set this vector's components to the maximum of itself and the
  ///        passed in vector
  /// \param[in] _v the maximum clamping vector
  public: void SetToMax(const IGN_VECTOR3 &_v);

  /// \brief Set this vector's components to the minimum of itself and the
  ///        passed in vector
  /// \param[in] _v the minimum clamping vector
  public: void SetToMin(const IGN_VECTOR3 &_v);

  /// \brief Get the maximum value in the vector
  /// \return the maximum element
  public: IGN_NUMERIC GetMax() const;

  /// \brief Get the minimum value in the vector
  /// \return the minimum element
  public: IGN_NUMERIC GetMin() const;

  /// \brief Assignment operator
  /// \param[in] _v a new value
  /// \return this
  public: IGN_VECTOR3 &operator =(const IGN_VECTOR3 &_v);

  /// \brief Assignment operator
  /// \param[in] _value assigned to all elements
  /// \return this
  public: IGN_VECTOR3 &operator =(IGN_NUMERIC _value);

  /// \brief Addition operator
  /// \param[in] _v vector to add
  /// \return the sum vector
  public: IGN_VECTOR3 operator+(const IGN_VECTOR3 &_v) const;

  /// \brief Addition assignment operator
  /// \param[in] _v vector to add
  public: const IGN_VECTOR3 &operator+=(const IGN_VECTOR3 &_v);

  /// \brief Negation operator
  /// \return negative of this vector
  public: inline IGN_VECTOR3 operator-() const
          {
            return IGN_VECTOR3(-this->data[0], -this->data[1], -this->data[2]);
          }

  /// \brief Subtraction operators
  /// \param[in] _pt a vector to substract
  /// \return a vector
  public: inline IGN_VECTOR3 operator-(const IGN_VECTOR3 &_pt) const
          {
            return IGN_VECTOR3(this->data[0] - _pt[0],
                               this->data[1] - _pt[1],
                               this->data[2] - _pt[2]);
          }

  /// \brief Subtraction operators
  /// \param[in] _pt subtrahend
  public: const IGN_VECTOR3 &operator-=(const IGN_VECTOR3 &_pt);

  /// \brief Division operator
  /// \brief[in] _pt the vector divisor
  /// \remarks this is an element wise division
  /// \return a vector
  public: const IGN_VECTOR3 operator/(const IGN_VECTOR3 &_pt) const;

  /// \brief Division assignment operator
  /// \brief[in] _pt the vector divisor
  /// \remarks this is an element wise division
  /// \return a vector
  public: const IGN_VECTOR3 &operator/=(const IGN_VECTOR3 &_pt);

  /// \brief Division operator
  /// \remarks this is an element wise division
  /// \return a vector
  public: const IGN_VECTOR3 operator/(IGN_NUMERIC _v) const;

  /// \brief Division operator
  /// \remarks this is an element wise division
  /// \return this
  public: const IGN_VECTOR3 &operator/=(IGN_NUMERIC _v);

  /// \brief Multiplication operator
  /// \remarks this is an element wise multiplication, not a cross product
  /// \param[in] _v
  public: IGN_VECTOR3 operator*(const IGN_VECTOR3 &_p) const;

  /// \brief Multiplication operators
  /// \remarks this is an element wise multiplication, not a cross product
  /// \param[in] _v a vector
  /// \return this
  public: const IGN_VECTOR3 &operator*=(const IGN_VECTOR3 &_v);

  /// \brief Multiplication operators
  /// \param[in] _s the scaling factor
  /// \return a scaled vector
  public: inline IGN_VECTOR3 operator*(IGN_NUMERIC _s) const
          {
            return IGN_VECTOR3(this->data[0] * _s,
                               this->data[1] * _s,
                               this->data[2] * _s);
          }

  /// \brief Multiplication operator
  /// \param[in] _v scaling factor
  /// \return this
  public: const IGN_VECTOR3 &operator*=(IGN_NUMERIC _v);

  /// \brief Equal to operator
  /// \param[in] _pt The vector to compare against
  /// \return true if each component is equal withing a
  /// default tolerence (1e-6), false otherwise
  public: bool operator ==(const IGN_VECTOR3 &_pt) const;

  /// \brief Not equal to operator
  /// \param[in] _v The vector to compare against
  /// \return true if each component is equal withing a
  /// default tolerence (1e-6), false otherwise
  public: bool operator!=(const IGN_VECTOR3 &_v) const;

  /// \brief See if a point is finite (e.g., not nan)
  public: bool IsFinite() const;

  /// \brief Corrects any nan values
  public: inline void Correct()
          {
            // std::isfinite works with floating point values, need to explicit
            // cast to avoid ambiguity in vc++.
            if (!std::isfinite(static_cast<double>(this->data[0])))
              this->data[0] = 0;
            if (!std::isfinite(static_cast<double>(this->data[1])))
              this->data[1] = 0;
            if (!std::isfinite(static_cast<double>(this->data[2])))
              this->data[2] = 0;
          }

  /// \brief Array subscript operator
  /// \param[in] _index The index, where 0 == x, 1 == y, 2 == z.
  /// \return The value. Throws an IndexException if _index is out of
  /// bounds.
  /// \throws IndexException if _index is >= 3.
  public: IGN_NUMERIC operator[](size_t _index) const
          {
            if (_index > 2)
              throw IndexException();
            return this->data[_index];
          }

  /// \brief Round all values to _precision decimal places
  /// \param[in] _precision the decimal places
  public: void Round(int _precision);

  /// \brief Equality test
  /// \remarks This is equivalent to the == operator
  /// \param[in] _v the other vector
  /// \return true if the 2 vectors have the same values, false otherwise
  public: bool Equal(const IGN_VECTOR3 &_v) const;

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

  /// \brief The x, y, and z values
  private: IGN_NUMERIC data[3];
};

/// \brief Stream insertion operator
/// \param _out output stream
/// \param _pt IGN_VECTOR3 to output
/// \return the stream
std::ostream &operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR3 &_pt);

/// \brief Stream extraction operator
/// \param _in input stream
/// \param _pt vector3 to read values into
/// \return the stream
std::istream &operator>>(std::istream &_in, ignition::math::IGN_VECTOR3 &_pt);
