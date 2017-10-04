/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_COLOR_HH_
#define IGNITION_MATH_COLOR_HH_

#include <iostream>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    /// \class Color Color.hh ignition/math/Color.hh
    /// \brief Defines a color using a red (R), green (G), blue (B), and alpha
    /// (A) component. Each color component is in the range [0..1].
    class IGNITION_VISIBLE Color
    {
      /// \brief (1, 1, 1)
      public: static const Color White;
      /// \brief (0, 0, 0)
      public: static const Color Black;
      /// \brief (1, 0, 0)
      public: static const Color Red;
      /// \brief (0, 1, 0)
      public: static const Color Green;
      /// \brief (0, 0, 1)
      public: static const Color Blue;
      /// \brief (1, 1, 0)
      public: static const Color Yellow;
      /// \brief (1, 0, 1)
      public: static const Color Magenta;
      /// \brief (0, 1, 1)
      public: static const Color Cyan;

      /// \def RGBA
      /// \brief A RGBA packed value as an unsigned int
      public: typedef unsigned int RGBA;

      /// \def BGRA
      /// \brief A BGRA packed value as an unsigned int
      public: typedef unsigned int BGRA;

      /// \def ARGB
      /// \brief A ARGB packed value as an unsigned int
      public: typedef unsigned int ARGB;

      /// \def ABGR
      /// \brief A ABGR packed value as an unsigned int
      public: typedef unsigned int ABGR;

      /// \brief Constructor
      public: Color();

      /// \brief Constructor
      /// \param[in] _r Red value (range 0 to 1)
      /// \param[in] _g Green value (range 0 to 1
      /// \param[in] _b Blue value (range 0 to 1
      /// \param[in] _a Alpha value (0=transparent, 1=opaque)
      public: Color(const float _r, const float _g, const float _b,
                  const float _a = 1.0);

      /// \brief Copy Constructor
      /// \param[in] _clr Color to copy
      public: Color(const Color &_clr);

      /// \brief Destructor
      public: virtual ~Color();

      /// \brief Reset the color to default values to red=0, green=0,
      /// blue=0, alpha=1.
      public: void Reset();

      /// \brief Set the contents of the vector
      /// \param[in] _r Red value (range 0 to 1)
      /// \param[in] _g Green value (range 0 to 1)
      /// \param[in] _b Blue value (range 0 to 1)
      /// \param[in] _a Alpha value (0=transparent, 1=opaque)
      public: void Set(const float _r = 1, const float _g = 1,
                  const float _b = 1, const float _a = 1);

      /// \brief Get the color in HSV colorspace
      /// \return HSV values in a Vector3f format. A vector3f containing
      /// {NAN_F, NAN_F, NAN_F} is returned on error.
      public: Vector3f HSV() const;

      /// \brief Set a color based on HSV values
      /// \param[in] _h Hue(0..360)
      /// \param[in] _s Saturation(0..1)
      /// \param[in] _v Value(0..1)
      public: void SetFromHSV(const float _h, const float _s, const float _v);

      /// \brief Get the color in YUV colorspace
      /// \return the YUV  color
      public: Vector3f YUV() const;

      /// \brief Set from yuv
      /// \param[in] _y value
      /// \param[in] _u value
      /// \param[in] _v value
      public: void SetFromYUV(const float _y, const float _u, const float _v);

      /// \brief Equal operator
      /// \param[in] _pt Color to copy
      /// \return Reference to this color
      public: Color &operator=(const Color &_pt);

      /// \brief Array index operator
      /// \param[in] _index Color component index(0=red, 1=green, 2=blue)
      /// \return r, g, b, or a when _index is 0, 1, 2 or 3. A NAN_F value is
      /// returned if the _index is invalid
      public: float operator[](const unsigned int _index);

      /// \brief Get as uint32 RGBA packed value
      /// \return the color
      public: RGBA AsRGBA() const;

      /// \brief Get as uint32 BGRA packed value
      /// \return the color
      public: BGRA AsBGRA() const;

      /// \brief Get as uint32 ARGB packed value
      /// \return the color
      public: ARGB AsARGB() const;

      /// \brief Get as uint32 ABGR packed value
      /// \return the color
      public: ABGR AsABGR() const;

      /// \brief Set from uint32 RGBA packed value
      /// \param[in] _v the new color
      public: void SetFromRGBA(const RGBA _v);

      /// \brief Set from uint32 BGRA packed value
      /// \param[in] _v the new color
      public: void SetFromBGRA(const BGRA _v);

      /// \brief Set from uint32 ARGB packed value
      /// \param[in] _v the new color
      public: void SetFromARGB(const ARGB _v);

      /// \brief Set from uint32 ABGR packed value
      /// \param[in] _v the new color
      public: void SetFromABGR(const ABGR _v);

      /// \brief Addition operator (this + _pt)
      /// \param[in] _pt Color to add
      /// \return The resulting color
      public: Color operator+(const Color &_pt) const;

      /// \brief Add _v to all color components
      /// \param[in] _v Value to add to each color component
      /// \return The resulting color
      public: Color operator+(const float &_v) const;

      /// \brief Addition equal operator
      /// \param[in] _pt Color to add
      /// \return The resulting color
      public: const Color &operator+=(const Color &_pt);

      /// \brief Subtraction operator
      /// \param[in] _pt The color to substract
      /// \return The resulting color
      public: Color operator-(const Color &_pt) const;

      /// \brief Subtract _v from all color components
      /// \param[in] _v Value to subtract
      /// \return The resulting color
      public: Color operator-(const float &_v) const;

      /// \brief Subtraction equal operator
      /// \param[in] _pt Color to subtract
      /// \return The resulting color
      public: const Color &operator-=(const Color &_pt);

      /// \brief Division operator
      /// \param[in] _pt Color to divide by
      /// \return The resulting color
      public: const Color operator/(const Color &_pt) const;

      /// \brief Divide all color component by _v
      /// \param[in] _v The value to divide by
      /// \return The resulting color
      public: const Color operator/(const float &_v) const;

      /// \brief Division equal operator
      /// \param[in] _pt Color to divide by
      /// \return The resulting color
      public: const Color &operator/=(const Color &_pt);

      /// \brief Multiplication operator
      /// \param[in] _pt The color to muliply by
      /// \return The resulting color
      public: const Color operator*(const Color &_pt) const;

      /// \brief Multiply all color components by _v
      /// \param[in] _v The value to multiply by
      /// \return The resulting color
      public: const Color operator*(const float &_v) const;

      /// \brief Multiplication equal operator
      /// \param[in] _pt The color to muliply by
      /// \return The resulting color
      public: const Color &operator*=(const Color &_pt);

      /// \brief Equality operator
      /// \param[in] _pt The color to check for equality
      /// \return True if the this color equals _pt
      public: bool operator==(const Color &_pt) const;

      /// \brief Inequality operator
      /// \param[in] _pt The color to check for inequality
      /// \return True if the this color does not equal _pt
      public: bool operator!=(const Color &_pt) const;

      /// \brief Clamp the color values to valid ranges
      private: void Clamp();

      /// \brief Stream insertion operator
      /// \param[in] _out the output stream
      /// \param[in] _pt the color
      /// \return the output stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const Color &_pt)
      {
        _out << _pt.r << " " << _pt.g << " " << _pt.b << " " << _pt.a;
        return _out;
      }

      /// \brief Stream insertion operator
      /// \param[in] _in the input stream
      /// \param[in] _pt
      public: friend std::istream &operator>> (std::istream &_in, Color &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.r >> _pt.g >> _pt.b >> _pt.a;
        return _in;
      }

      /// \brief Get the red value
      /// \return The red value
      public: float R() const;

      /// \brief Get the green value
      /// \return The green value
      public: float G() const;

      /// \brief Get the blue value
      /// \return The blue value
      public: float B() const;

      /// \brief Get the alpha value
      /// \return The alpha value
      public: float A() const;

      /// \brief Get a mutable reference to the red value
      /// \return The red value
      public: float &R();

      /// \brief Get a mutable reference to the green value
      /// \return The green value
      public: float &G();

      /// \brief Get a mutable reference to the blue value
      /// \return The blue value
      public: float &B();

      /// \brief Get a mutable reference to the alpha value
      /// \return The alpha value
      public: float &A();

      /// \brief Set the red value
      /// \param _r New red value
      public: void R(const float _r);

      /// \brief Set the green value
      /// \param _r New green value
      public: void G(const float _g);

      /// \brief Set the blue value
      /// \param _r New blue value
      public: void B(const float _b);

      /// \brief Set the alpha value
      /// \param _r New alpha value
      public: void A(const float _a);

      /// \brief Red value
      private: float r = 0;

      /// \brief Green value
      private: float g = 0;

      /// \brief Blue value
      private: float b = 0;

      /// \brief Alpha value
      private: float a = 1;
    };
  }
}
#endif
