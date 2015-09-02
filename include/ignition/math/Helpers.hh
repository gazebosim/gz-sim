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
#ifndef _IGNITION_MATH_FUNCTIONS_HH_
#define _IGNITION_MATH_FUNCTIONS_HH_

#ifndef _USE_MATH_DEFINES
# define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <iostream>
#include <vector>

/// \brief Double maximum value. This value will be similar to 1.79769e+308
#define IGN_DBL_MAX std::numeric_limits<double>::max()

/// \brief Double min value. This value will be similar to 2.22507e-308
#define IGN_DBL_MIN std::numeric_limits<double>::min()

/// \brief Double low value, equivalent to -IGN_DBL_MAX
#define IGN_DBL_LOW std::numeric_limits<double>::lowest()

/// \brief Double positive infinite value
#define IGN_DBL_INF std::numeric_limits<double>::infinity()

/// \brief Float maximum value. This value will be similar to 3.40282e+38
#define IGN_FLT_MAX std::numeric_limits<float>::max()

/// \brief Float minimum value. This value will be similar to 1.17549e-38
#define IGN_FLT_MIN std::numeric_limits<float>::min()

/// \brief Float lowest value, equivalent to -IGN_FLT_MAX
#define IGN_FLT_LOW std::numeric_limits<float>::lowest()

/// \brief 32bit unsigned integer maximum value
#define IGN_UINT32_MAX std::numeric_limits<uint32_t>::max()

/// \brief 32bit unsigned integer minimum value
#define IGN_UINT32_MIN std::numeric_limits<uint32_t>::min()

/// \brief 32bit unsigned integer lowest value. This is equivalent to
/// IGN_UINT32_MIN, and is defined here for completeness.
#define IGN_UINT32_LOW std::numeric_limits<uint32_t>::lowest()

/// \brief 32bit integer maximum value
#define IGN_INT32_MAX std::numeric_limits<int32_t>::max()

/// \brief 32bit integer minimum value
#define IGN_INT32_MIN std::numeric_limits<int32_t>::min()

/// \brief 32bit integer minimum value. This is equivalent to IGN_INT32_MIN,
/// and is defined here for completeness.
#define IGN_INT32_LOW std::numeric_limits<int32_t>::lowest()

/// \brief Define IGN_PI, IGN_PI_2, and IGN_PI_4.
/// This was put here for Windows support.
#ifdef M_PI
#define IGN_PI M_PI
#define IGN_PI_2 M_PI_2
#define IGN_PI_4 M_PI_4
#else
#define IGN_PI   3.14159265358979323846
#define IGN_PI_2 1.57079632679489661923
#define IGN_PI_4 0.78539816339744830962
#endif

namespace ignition
{
  /// \brief Math classes and function useful in robot applications.
  namespace math
  {
    /// \brief Returns the representation of a quiet not a number (NAN)
    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const float NAN_F = std::numeric_limits<float>::quiet_NaN();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const int NAN_I = std::numeric_limits<int>::quiet_NaN();

    /// \brief Simple clamping function
    /// \param[in] _v value
    /// \param[in] _min minimum
    /// \param[in] _max maximum
    template<typename T>
    inline T clamp(T _v, T _min, T _max)
    {
      return std::max(std::min(_v, _max), _min);
    }

    /// \brief check if a float is NaN
    /// \param[in] _v the value
    /// \return true if _v is not a number, false otherwise
    inline bool isnan(float _v)
    {
      return (std::isnan)(_v);
    }

    /// \brief check if a double is NaN
    /// \param[in] _v the value
    /// \return true if _v is not a number, false otherwise
    inline bool isnan(double _v)
    {
      return (std::isnan)(_v);
    }

    /// \brief Fix a nan value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN, _v otherwise.
    inline float fixnan(float _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0f : _v;
    }

    /// \brief Fix a nan value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN, _v otherwise.
    inline double fixnan(double _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0 : _v;
    }

    /// \brief Check if parameter is even.
    /// \param[in] _v Value to check.
    /// \return True if _v is even.
    inline bool isEven(const int _v)
    {
      return !(_v % 2);
    }

    /// \brief Check if parameter is even.
    /// \param[in] _v Value to check.
    /// \return True if _v is even.
    inline bool isEven(const unsigned int _v)
    {
      return !(_v % 2);
    }

    /// \brief Check if parameter is odd.
    /// \param[in] _v Value to check.
    /// \return True if _v is odd.
    inline bool isOdd(const int _v)
    {
      return (_v % 2) != 0;
    }

    /// \brief Check if parameter is odd.
    /// \param[in] _v Value to check.
    /// \return True if _v is odd.
    inline bool isOdd(const unsigned int _v)
    {
      return (_v % 2) != 0;
    }

    /// \brief get mean of vector of values
    /// \param[in] _values the vector of values
    /// \return the mean
    template<typename T>
    inline T mean(const std::vector<T> &_values)
    {
      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += _values[i];
      return sum / _values.size();
    }

    /// \brief get variance of vector of values
    /// \param[in] _values the vector of values
    /// \return the squared deviation
    template<typename T>
    inline T variance(const std::vector<T> &_values)
    {
      T avg = mean<T>(_values);

      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += (_values[i] - avg) * (_values[i] - avg);
      return sum / _values.size();
    }

    /// \brief get the maximum value of vector of values
    /// \param[in] _values the vector of values
    /// \return maximum
    template<typename T>
    inline T max(const std::vector<T> &_values)
    {
      T max = std::numeric_limits<T>::min();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] > max)
          max = _values[i];
      return max;
    }

    /// \brief get the minimum value of vector of values
    /// \param[in] _values the vector of values
    /// \return minimum
    template<typename T>
    inline T min(const std::vector<T> &_values)
    {
      T min = std::numeric_limits<T>::max();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] < min)
          min = _values[i];
      return min;
    }

    /// \brief check if two values are equal, within a tolerance
    /// \param[in] _a the first value
    /// \param[in] _b the second value
    /// \param[in] _epsilon the tolerance
    template<typename T>
    inline bool equal(const T &_a, const T &_b,
                      const T &_epsilon = 1e-6)
    {
      return std::abs(_a - _b) <= _epsilon;
    }

    /// \brief get value at a specified precision
    /// \param[in] _a the number
    /// \param[in] _precision the precision
    /// \return the value for the specified precision
    template<typename T>
    inline T precision(const T &_a, const unsigned int &_precision)
    {
      return std::round(_a * pow(10, _precision)) / pow(10, _precision);
    }

    /// \brief Is this a power of 2?
    /// \param[in] _x the number
    /// \return true if _x is a power of 2, false otherwise
    inline bool isPowerOfTwo(unsigned int _x)
    {
      return ((_x != 0) && ((_x & (~_x + 1)) == _x));
    }

    /// \brief Get the smallest power of two that is greater or equal to
    /// a given value
    /// \param[in] _x the number
    /// \return the same value if _x is already a power of two. Otherwise,
    /// it returns the smallest power of two that is greater than _x
    inline unsigned int roundUpPowerOfTwo(unsigned int _x)
    {
      if (_x == 0)
        return 1;

      if (isPowerOfTwo(_x))
        return _x;

      while (_x & (_x - 1))
        _x = _x & (_x - 1);

      _x = _x << 1;

      return _x;
    }

    /// \brief parse string into an integer
    /// \param[in] _input the string
    /// \return an integer, 0 or 0 and a message in the error stream
    inline int parseInt(const std::string &_input)
    {
      const char *p = _input.c_str();
      if (!*p || *p == '?')
        return NAN_I;

      int s = 1;
      while (*p == ' ')
        p++;

      if (*p == '-')
      {
        s = -1;
        p++;
      }

      double acc = 0;
      while (*p >= '0' && *p <= '9')
        acc = acc * 10 + *p++ - '0';

      if (*p)
      {
        std::cerr << "Invalid int numeric format[" << _input << "]\n";
        return NAN_I;
      }

      return static_cast<int>(s * acc);
    }

    /// \brief parse string into float
    /// \param _input the string
    /// \return a floating point number (can be NaN) or 0 with a message in the
    /// error stream
    inline double parseFloat(const std::string &_input)
    {
      const char *p = _input.c_str();
      if (!*p || *p == '?')
        return NAN_D;
      int s = 1;
      while (*p == ' ')
        p++;

      if (*p == '-')
      {
        s = -1;
        p++;
      }

      double acc = 0;
      while (*p >= '0' && *p <= '9')
        acc = acc * 10 + *p++ - '0';

      if (*p == '.')
      {
        double k = 0.1;
        p++;
        while (*p >= '0' && *p <= '9')
        {
          acc += (*p++ - '0') * k;
          k *= 0.1;
        }
      }
      if (*p == 'e')
      {
        int es = 1;
        int f = 0;
        p++;
        if (*p == '-')
        {
          es = -1;
          p++;
        }
        else if (*p == '+')
        {
          es = 1;
          p++;
        }
        while (*p >= '0' && *p <= '9')
          f = f * 10 + *p++ - '0';

        acc *= pow(10, f*es);
      }

      if (*p)
      {
        std::cerr << "Invalid double numeric format[" << _input << "]\n";
        return NAN_D;
      }
      return s * acc;
    }
  }
}

/** \def IGNITION_VISIBLE
 * Use to represent "symbol visible" if supported
 */

/** \def IGNITION_HIDDEN
 * Use to represent "symbol hidden" if supported
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define IGNITION_VISIBLE __attribute__ ((dllexport))
    #else
      #define IGNITION_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define IGNITION_VISIBLE __attribute__ ((dllimport))
    #else
      #define IGNITION_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define IGNITION_HIDDEN
#else
  #if __GNUC__ >= 4
    #define IGNITION_VISIBLE __attribute__ ((visibility ("default")))
    #define IGNITION_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define IGNITION_VISIBLE
    #define IGNITION_HIDDEN
  #endif
#endif

#endif
