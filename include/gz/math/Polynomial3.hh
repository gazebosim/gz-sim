/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_MATH_POLYNOMIAL3_HH_
#define GZ_MATH_POLYNOMIAL3_HH_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <gz/math/Interval.hh>
#include <gz/math/Vector4.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Polynomial3 Polynomial3.hh gz/math/Polynomial3.hh
    /// \brief The Polynomial3 class represents a cubic polynomial
    /// with real coefficients p(x) = c0 x^3 + c1 x^2 + c2 x + c3.
    /// ## Example
    ///
    /// \snippet examples/polynomial3_example.cc complete
    template <typename T>
    class Polynomial3
    {
      /// \brief Constructor
      public: Polynomial3() = default;

      /// \brief Constructor
      /// \param[in] _coeffs coefficients c0 through c3, left to right
      public: explicit Polynomial3(Vector4<T> _coeffs)
      : coeffs(std::move(_coeffs))
      {
      }

      /// \brief Make a constant polynomial
      /// \return a p(x) = `_value` polynomial
      public: static Polynomial3 Constant(T _value)
      {
        return Polynomial3(Vector4<T>(0., 0., 0., _value));
      }

      /// \brief Get the polynomial coefficients
      /// \return this polynomial coefficients
      public: const Vector4<T> &Coeffs() const { return this->coeffs; }

      /// \brief Evaluate the polynomial at `_x`
      /// For non-finite `_x`, this function
      /// computes p(z) as z tends to `_x`.
      /// \param[in] _x polynomial argument
      /// \return the result of evaluating p(`_x`)
      public: T Evaluate(const T &_x) const
      {
        using std::isnan, std::isfinite;
        if (isnan(_x))
        {
          return _x;
        }
        if (!isfinite(_x))
        {
          using std::abs, std::copysign;
          const T epsilon =
              std::numeric_limits<T>::epsilon();
          if (abs(this->coeffs[0]) >= epsilon)
          {
            return _x * copysign(T(1.), this->coeffs[0]);
          }
          if (abs(this->coeffs[1]) >= epsilon)
          {
            return copysign(_x, this->coeffs[1]);
          }
          if (abs(this->coeffs[2]) >= epsilon)
          {
            return _x * copysign(T(1.), this->coeffs[2]);
          }
          return this->coeffs[3];
        }
        const T _x2 = _x * _x;
        const T _x3 = _x2 * _x;

        return (this->coeffs[0] * _x3 + this->coeffs[1] * _x2 +
                this->coeffs[2] * _x + this->coeffs[3]);
      }

      /// \brief Call operator overload
      /// \see Polynomial3::Evaluate()
      public: T operator()(const T &_x) const
      {
        return this->Evaluate(_x);
      }

      /// \brief Compute polynomial minimum in an `_interval`
      /// \param[in] _interval polynomial argument interval to check
      /// \param[out] _xMin polynomial argument that yields minimum,
      ///   or NaN if the interval is empty
      /// \return the polynomial minimum in the given interval,
      ///   or NaN if the interval is empty
      public: T Minimum(const Interval<T> &_interval, T &_xMin) const
      {
        if (_interval.Empty())
        {
          _xMin = std::numeric_limits<T>::quiet_NaN();
          return std::numeric_limits<T>::quiet_NaN();
        }
        T yMin;
        // For open intervals, assume continuity in the limit
        const T &xLeft = _interval.LeftValue();
        const T &xRight = _interval.RightValue();
        const T yLeft = this->Evaluate(xLeft);
        const T yRight = this->Evaluate(xRight);
        if (yLeft <= yRight)
        {
          yMin = yLeft;
          _xMin = xLeft;
        }
        else
        {
          yMin = yRight;
          _xMin = xRight;
        }
        using std::abs, std::sqrt;  // enable ADL
        constexpr T epsilon = std::numeric_limits<T>::epsilon();
        if (abs(this->coeffs[0]) >= epsilon)
        {
          // Polynomial function p(x) is cubic, look
          // for local minima within the given interval

          // Find local extrema by computing the roots
          // of p'(x), a quadratic polynomial function
          const T a = this->coeffs[0] * T(3.);
          const T b = this->coeffs[1] * T(2.);
          const T c = this->coeffs[2];

          const T discriminant = b * b - T(4.) * a * c;
          if (discriminant >= T(0.))
          {
            // Roots of p'(x) are real, check local minima
            const T x = (-b + sqrt(discriminant)) / (T(2.) * a);
            if (_interval.Contains(x))
            {
              const T y = this->Evaluate(x);
              if (y < yMin)
              {
                _xMin = x;
                yMin = y;
              }
            }
          }
        }
        else if (abs(this->coeffs[1]) >= epsilon)
        {
          // Polynomial function p(x) is quadratic,
          // look for global minima if concave
          const T a = this->coeffs[1];
          const T b = this->coeffs[2];
          if (a > T(0.))
          {
            const T x = -b / (T(2.) * a);
            if (_interval.Contains(x))
            {
              const T y = this->Evaluate(x);
              if (y < yMin)
              {
                _xMin = x;
                yMin = y;
              }
            }
          }
        }
        return yMin;
      }

      /// \brief Compute polynomial minimum in an `_interval`
      /// \param[in] _interval polynomial argument interval to check
      /// \return the polynomial minimum in the given interval (may
      ///   not be finite), or NaN if the interval is empty
      public: T Minimum(const Interval<T> &_interval) const
      {
        T xMin;
        return this->Minimum(_interval, xMin);
      }

      /// \brief Compute polynomial minimum
      /// \param[out] _xMin polynomial argument that yields minimum
      /// \return the polynomial minimum (may not be finite)
      public: T Minimum(T &_xMin) const
      {
        return this->Minimum(Interval<T>::Unbounded, _xMin);
      }

      /// \brief Compute polynomial minimum
      /// \return the polynomial minimum (may not be finite)
      public: T Minimum() const
      {
        T xMin;
        return this->Minimum(Interval<T>::Unbounded, xMin);
      }

      /// \brief Prints polynomial as p(`_x`) to `_out` stream
      /// \param[in] _out Output stream to print to
      /// \param[in] _x Argument name to be used
      public: void Print(std::ostream &_out, const std::string &_x = "x") const
      {
        constexpr T epsilon =
            std::numeric_limits<T>::epsilon();
        bool streamStarted = false;
        for (size_t i = 0; i < 4; ++i)
        {
          using std::abs;  // enable ADL
          const T magnitude = abs(this->coeffs[i]);
          const bool sign = this->coeffs[i] < T(0);
          const int exponent = 3 - i;
          if (magnitude >= epsilon)
          {
            if (streamStarted)
            {
              if (sign)
              {
                _out << " - ";
              }
              else
              {
                _out << " + ";
              }
            }
            else if (sign)
            {
              _out << "-";
            }
            if (exponent > 0)
            {
              if ((magnitude - T(1)) > epsilon)
              {
                _out << magnitude << " ";
              }
              _out <<  _x;
              if (exponent > 1)
              {
                _out << "^" << exponent;
              }
            }
            else
            {
              _out << magnitude;
            }
            streamStarted = true;
          }
        }
        if (!streamStarted)
        {
          _out << this->coeffs[3];
        }
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _p Polynomial3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(
        std::ostream &_out, const gz::math::Polynomial3<T> &_p)
      {
        _p.Print(_out, "x");
        return _out;
      }

      /// \brief Polynomial coefficients
      private: Vector4<T> coeffs;
    };
    using Polynomial3f = Polynomial3<float>;
    using Polynomial3d = Polynomial3<double>;
    }
  }
}

#endif
