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
#ifndef IGNITION_MATH_ANGLE_HH_
#define IGNITION_MATH_ANGLE_HH_

#include <iostream>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

/// \def IGN_RTOD(d)
/// \brief Macro that converts radians to degrees
/// \param[in] r radians
/// \return degrees
#define IGN_RTOD(r) ((r) * 180 / IGN_PI)

/// \def IGN_DTOR(d)
/// \brief Converts degrees to radians
/// \param[in] d degrees
/// \return radians
#define IGN_DTOR(d) ((d) * IGN_PI / 180)

/// \def IGN_NORMALIZE(a)
/// \brief Macro that normalizes an angle in the range -Pi to Pi
/// \param[in] a angle
/// \return the angle, in range
#define IGN_NORMALIZE(a) (atan2(sin(a), cos(a)))

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Angle Angle.hh ignition/math/Angle.hh
    /// \brief The Angle class is used to simplify and clarify the use of
    /// radians and degrees measurements. A default constructed Angle instance
    /// has a value of zero radians/degrees.
    ///
    /// Unless otherwise specified, the Angle class assumes units are in
    /// radians. An example of this are the stream insertion (<<) and
    /// extraction (>>) operators.
    ///
    /// ## Example
    ///
    /// \snippet examples/angle_example.cc complete
    class IGNITION_MATH_VISIBLE Angle
    {
      /// \brief An angle with a value of zero.
      /// Equivalent to math::Angle(0).
      public: static const Angle Zero;

      /// \brief An angle with a value of Pi.
      /// Equivalent to math::Angle(IGN_PI).
      public: static const Angle Pi;

      /// \brief An angle with a value of Pi * 0.5.
      /// Equivalent to math::Angle(IGN_PI * 0.5).
      public: static const Angle HalfPi;

      /// \brief An angle with a value of Pi * 2.
      /// Equivalent to math::Angle(IGN_PI * 2).
      public: static const Angle TwoPi;

      /// \brief Default constructor that initializes an Angle to zero
      /// radians/degrees.
      public: Angle();

      /// \brief Conversion constructor that initializes an Angle to the
      /// specified radians. This constructor supports implicit conversion
      /// of a double to an Angle. For example:
      ///
      /// \code
      /// Angle a = 3.14;
      /// \endcode
      //
      /// \param[in] _radian The radians used to initialize this Angle.
      // cppcheck-suppress noExplicitConstructor
      public: Angle(const double _radian);

      /// \brief Copy constructor that initializes this Angle to the value
      /// contained in the _angle parameter.
      /// \param[in] _angle Angle to copy
      public: Angle(const Angle &_angle);

      /// \brief Move constructor
      /// \param[in] _angle Angle to move
      public: Angle(Angle &&_angle) noexcept;

      /// \brief Destructor
      public: virtual ~Angle();

      /// \brief Copy assignment operator
      /// \param[in] _angle Angle to copy
      public: Angle& operator=(const Angle &_angle);

      /// \brief Move assignment operator
      /// \param[in] _angle Angle to move
      public: Angle& operator=(Angle &&_angle) noexcept;

      /// \brief Set the value from an angle in radians.
      /// \param[in] _radian Radian value.
      /// \sa SetRadian(double)
      public: void Radian(double _radian);

      /// \brief Set the value from an angle in radians.
      /// \param[in] _radian Radian value.
      public: void SetRadian(double _radian);

      /// \brief Set the value from an angle in degrees
      /// \param[in] _degree Degree value
      /// \sa SetDegree(double)
      public: void Degree(double _degree);

      /// \brief Set the value from an angle in degrees
      /// \param[in] _degree Degree value
      public: void SetDegree(double _degree);

      /// \brief Get the angle in radians.
      /// \return Double containing the angle's radian value.
      public: double Radian() const;

      /// \brief Get the angle in degrees.
      /// \return Double containing the angle's degree value.
      public: double Degree() const;

      /// \brief Normalize the angle in the range -Pi to Pi. This
      /// modifies the value contained in this Angle instance.
      /// \sa Normalized()
      public: void Normalize();

      /// \brief Return the normalized angle in the range -Pi to Pi. This
      /// does not modify the value contained in this Angle instance.
      /// \return The normalized value of this Angle.
      public: Angle Normalized() const;

      /// \brief Return the angle's radian value
      /// \return double containing the angle's radian value
      public: double operator()() const;

      /// \brief Dereference operator
      /// \return Double containing the angle's radian value
      public: inline double operator*() const
              {
                return value;
              }

      /// \brief Subtraction operator, result = this - _angle.
      /// \param[in] _angle Angle for subtraction.
      /// \return The new angle.
      public: Angle operator-(const Angle &_angle) const;

      /// \brief Addition operator, result = this + _angle.
      /// \param[in] _angle Angle for addition.
      /// \return The new angle.
      public: Angle operator+(const Angle &_angle) const;

      /// \brief Multiplication operator, result = this * _angle.
      /// \param[in] _angle Angle for multiplication.
      /// \return The new angle
      public: Angle operator*(const Angle &_angle) const;

      /// \brief Division operator, result = this / _angle.
      /// \param[in] _angle Angle for division.
      /// \return The new angle.
      public: Angle operator/(const Angle &_angle) const;

      /// \brief Subtraction set operator, this = this - _angle.
      /// \param[in] _angle Angle for subtraction.
      /// \return The new angle.
      public: Angle operator-=(const Angle &_angle);

      /// \brief Addition set operator, this = this + _angle.
      /// \param[in] _angle Angle for addition.
      /// \return The new angle.
      public: Angle operator+=(const Angle &_angle);

      /// \brief Multiplication set operator, this = this * _angle.
      /// \param[in] _angle Angle for multiplication.
      /// \return The new angle.
      public: Angle operator*=(const Angle &_angle);

      /// \brief Division set operator, this = this / _angle.
      /// \param[in] _angle Angle for division.
      /// \return The new angle.
      public: Angle operator/=(const Angle &_angle);

      /// \brief Equality operator, result = this == _angle.
      /// \param[in] _angle Angle to check for equality.
      /// \return True if this == _angle.
      public: bool operator==(const Angle &_angle) const;

      /// \brief Inequality operator
      /// \param[in] _angle Angle to check for inequality.
      /// \return True if this != _angle.
      public: bool operator!=(const Angle &_angle) const;

      /// \brief Less than operator.
      /// \param[in] _angle Angle to check.
      /// \return True if this < _angle.
      public: bool operator<(const Angle &_angle) const;

      /// \brief Less than or equal operator.
      /// \param[in] _angle Angle to check.
      /// \return True if this <= _angle.
      public: bool operator<=(const Angle &_angle) const;

      /// \brief Greater than operator.
      /// \param[in] _angle Angle to check.
      /// \return True if this > _angle.
      public: bool operator>(const Angle &_angle) const;

      /// \brief Greater than or equal operator.
      /// \param[in] _angle Angle to check.
      /// \return True if this >= _angle.
      public: bool operator>=(const Angle &_angle) const;

      /// \brief Stream insertion operator. Outputs in radians.
      /// \param[in] _out Output stream.
      /// \param[in] _a Angle to output.
      /// \return The output stream.
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const ignition::math::Angle &_a)
      {
        _out << _a.Radian();
        return _out;
      }

      /// \brief Stream extraction operator. Assumes input is in radians.
      /// \param[in,out] _in Input stream.
      /// \param[out] _a Angle to read value into.
      /// \return The input stream.
      public: friend std::istream &operator>>(std::istream &_in,
                                              ignition::math::Angle &_a)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _a.value;
        return _in;
      }

      /// The angle in radians
      private: double value{0};
    };
    }
  }
}

#endif
