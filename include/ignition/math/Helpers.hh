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
#ifndef IGNITION_MATH_FUNCTIONS_HH_
#define IGNITION_MATH_FUNCTIONS_HH_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ignition/math/config.hh>
#include "ignition/math/Export.hh"

/// \brief The default tolerance value used by MassMatrix3::IsValid(),
/// MassMatrix3::IsPositive(), and MassMatrix3::ValidMoments()
template <typename T>
constexpr T IGN_MASSMATRIX3_DEFAULT_TOLERANCE = T(10);

/// \brief Double maximum value. This value will be similar to 1.79769e+308
/// \deprecated Use static const value instead.
#define IGN_DBL_MAX ignition::math::DPRCT_MAX_D

/// \brief Double min value. This value will be similar to 2.22507e-308
/// \deprecated Use static const value instead.
#define IGN_DBL_MIN ignition::math::DPRCT_MIN_D

/// \brief Double low value, equivalent to -IGN_DBL_MAX
/// \deprecated Use static const value instead.
#define IGN_DBL_LOW ignition::math::DPRCT_LOW_D

/// \brief Double positive infinite value
/// \deprecated Use static const value instead.
#define IGN_DBL_INF ignition::math::DPRCT_INF_D

/// \brief Float maximum value. This value will be similar to 3.40282e+38
/// \deprecated Use static const value instead.
#define IGN_FLT_MAX ignition::math::DPRCT_MAX_F

/// \brief Float minimum value. This value will be similar to 1.17549e-38
/// \deprecated Use static const value instead.
#define IGN_FLT_MIN ignition::math::DPRCT_MIN_F

/// \brief Float lowest value, equivalent to -IGN_FLT_MAX
/// \deprecated Use static const value instead.
#define IGN_FLT_LOW ignition::math::DPRCT_LOW_F

/// \brief Float positive infinite value
/// \deprecated Use static const value instead.
#define IGN_FLT_INF ignition::math::DPRCT_INF_F

/// \brief 16bit unsigned integer maximum value
/// \deprecated Use static const value instead.
#define IGN_UINT16_MAX ignition::math::DPRCT_MAX_UI16

/// \brief 16bit unsigned integer minimum value
/// \deprecated Use static const value instead.
#define IGN_UINT16_MIN ignition::math::DPRCT_MIN_UI16

/// \brief 16bit unsigned integer lowest value. This is equivalent to
/// IGN_UINT16_MIN, and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_UINT16_LOW ignition::math::DPRCT_LOW_UI16

/// \brief 16-bit unsigned integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_UINT16_INF ignition::math::DPRCT_INF_UI16

/// \brief 16bit integer maximum value
/// \deprecated Use static const value instead.
#define IGN_INT16_MAX ignition::math::DPRCT_MAX_I16

/// \brief 16bit integer minimum value
/// \deprecated Use static const value instead.
#define IGN_INT16_MIN ignition::math::DPRCT_MIN_I16

/// \brief 16bit integer lowest value. This is equivalent to IGN_INT16_MIN,
/// and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_INT16_LOW ignition::math::DPRCT_LOW_I16

/// \brief 16-bit integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_INT16_INF ignition::math::DPRCT_INF_I16

/// \brief 32bit unsigned integer maximum value
/// \deprecated Use static const value instead.
#define IGN_UINT32_MAX ignition::math::DPRCT_MAX_UI32

/// \brief 32bit unsigned integer minimum value
/// \deprecated Use static const value instead.
#define IGN_UINT32_MIN ignition::math::DPRCT_MIN_UI32

/// \brief 32bit unsigned integer lowest value. This is equivalent to
/// IGN_UINT32_MIN, and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_UINT32_LOW ignition::math::DPRCT_LOW_UI32

/// \brief 32-bit unsigned integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_UINT32_INF ignition::math::DPRCT_INF_UI32

/// \brief 32bit integer maximum value
/// \deprecated Use static const value instead.
#define IGN_INT32_MAX ignition::math::DPRCT_MAX_I32

/// \brief 32bit integer minimum value
/// \deprecated Use static const value instead.
#define IGN_INT32_MIN ignition::math::DPRCT_MIN_I32

/// \brief 32bit integer minimum value. This is equivalent to IGN_INT32_MIN,
/// and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_INT32_LOW ignition::math::DPRCT_LOW_I32

/// \brief 32-bit integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_INT32_INF ignition::math::DPRCT_INF_I32

/// \brief 64bit unsigned integer maximum value
/// \deprecated Use static const value instead.
#define IGN_UINT64_MAX ignition::math::DPRCT_MAX_UI64

/// \brief 64bit unsigned integer minimum value
/// \deprecated Use static const value instead.
#define IGN_UINT64_MIN ignition::math::DPRCT_MIN_UI64

/// \brief 64bit unsigned integer lowest value. This is equivalent to
/// IGN_UINT64_MIN, and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_UINT64_LOW ignition::math::DPRCT_LOW_UI64

/// \brief 64-bit unsigned integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_UINT64_INF ignition::math::DPRCT_INF_UI64

/// \brief 64bit integer maximum value
/// \deprecated Use static const value instead.
#define IGN_INT64_MAX ignition::math::DPRCT_MAX_I64

/// \brief 64bit integer minimum value
/// \deprecated Use static const value instead.
#define IGN_INT64_MIN ignition::math::DPRCT_MIN_I64

/// \brief 64bit integer lowest value. This is equivalent to IGN_INT64_MIN,
/// and is defined here for completeness.
/// \deprecated Use static const value instead.
#define IGN_INT64_LOW ignition::math::DPRCT_LOW_I64

/// \brief 64-bit integer positive infinite value
/// \deprecated Use static const value instead.
#define IGN_INT64_INF ignition::math::DPRCT_INF_I64

/// \brief Define IGN_PI, IGN_PI_2, and IGN_PI_4.
/// This was put here for Windows support.
#ifdef M_PI
#define IGN_PI M_PI
#define IGN_PI_2 M_PI_2
#define IGN_PI_4 M_PI_4
#define IGN_SQRT2 M_SQRT2
#else
#define IGN_PI   3.14159265358979323846
#define IGN_PI_2 1.57079632679489661923
#define IGN_PI_4 0.78539816339744830962
#define IGN_SQRT2 1.41421356237309504880
#endif

/// \brief Define IGN_FP_VOLATILE for FP equality comparisons
/// Use volatile parameters when checking floating point equality on
/// the 387 math coprocessor to work around bugs from the 387 extra precision
#if defined __FLT_EVAL_METHOD__  &&  __FLT_EVAL_METHOD__ == 2
#define IGN_FP_VOLATILE volatile
#else
#define IGN_FP_VOLATILE
#endif

/// \brief Compute sphere volume
/// \param[in] _radius Sphere radius
#define IGN_SPHERE_VOLUME(_radius) (4.0*IGN_PI*std::pow(_radius, 3)/3.0)

/// \brief Compute cylinder volume
/// \param[in] _r Cylinder base radius
/// \param[in] _l Cylinder length
#define IGN_CYLINDER_VOLUME(_r, _l) (_l * IGN_PI * std::pow(_r, 2))

/// \brief Compute box volume
/// \param[in] _x X length
/// \param[in] _y Y length
/// \param[in] _z Z length
#define IGN_BOX_VOLUME(_x, _y, _z) (_x *_y * _z)

/// \brief Compute box volume from a vector
/// \param[in] _v Vector3d that contains the box's dimensions.
#define IGN_BOX_VOLUME_V(_v) (_v.X() *_v.Y() * _v.Z())

namespace ignition
{
  /// \brief Math classes and function useful in robot applications.
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \brief size_t type with a value of 0
    static const size_t IGN_ZERO_SIZE_T  = 0u;

    /// \brief size_t type with a value of 1
    static const size_t IGN_ONE_SIZE_T   = 1u;

    /// \brief size_t type with a value of 2
    static const size_t IGN_TWO_SIZE_T   = 2u;

    /// \brief size_t type with a value of 3
    static const size_t IGN_THREE_SIZE_T = 3u;

    /// \brief size_t type with a value of 4
    static const size_t IGN_FOUR_SIZE_T  = 4u;

    /// \brief size_t type with a value of 5
    static const size_t IGN_FIVE_SIZE_T  = 5u;

    /// \brief size_t type with a value of 6
    static const size_t IGN_SIX_SIZE_T   = 6u;

    /// \brief size_t type with a value of 7
    static const size_t IGN_SEVEN_SIZE_T = 7u;

    /// \brief size_t type with a value of 8
    static const size_t IGN_EIGHT_SIZE_T = 8u;

    /// \brief size_t type with a value of 9
    static const size_t IGN_NINE_SIZE_T  = 9u;

    /// \brief Double maximum value. This value will be similar to 1.79769e+308
    static const double MAX_D = std::numeric_limits<double>::max();

    /// \brief Double min value. This value will be similar to 2.22507e-308
    static const double MIN_D = std::numeric_limits<double>::min();

    /// \brief Double low value, equivalent to -MAX_D
    static const double LOW_D = std::numeric_limits<double>::lowest();

    /// \brief Double positive infinite value
    static const double INF_D = std::numeric_limits<double>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    /// \brief Float maximum value. This value will be similar to 3.40282e+38
    static const float MAX_F = std::numeric_limits<float>::max();

    /// \brief Float minimum value. This value will be similar to 1.17549e-38
    static const float MIN_F = std::numeric_limits<float>::min();

    /// \brief Float low value, equivalent to -MAX_F
    static const float LOW_F = std::numeric_limits<float>::lowest();

    /// \brief float positive infinite value
    static const float INF_F = std::numeric_limits<float>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const float NAN_F = std::numeric_limits<float>::quiet_NaN();

    /// \brief 16bit unsigned integer maximum value
    static const uint16_t MAX_UI16 = std::numeric_limits<uint16_t>::max();

    /// \brief 16bit unsigned integer minimum value
    static const uint16_t MIN_UI16 = std::numeric_limits<uint16_t>::min();

    /// \brief 16bit unsigned integer lowest value. This is equivalent to
    /// IGN_UINT16_MIN, and is defined here for completeness.
    static const uint16_t LOW_UI16 = std::numeric_limits<uint16_t>::lowest();

    /// \brief 16-bit unsigned integer positive infinite value
    static const uint16_t INF_UI16 = std::numeric_limits<uint16_t>::infinity();

    /// \brief 16bit unsigned integer maximum value
    static const int16_t MAX_I16 = std::numeric_limits<int16_t>::max();

    /// \brief 16bit unsigned integer minimum value
    static const int16_t MIN_I16 = std::numeric_limits<int16_t>::min();

    /// \brief 16bit unsigned integer lowest value. This is equivalent to
    /// IGN_INT16_MIN, and is defined here for completeness.
    static const int16_t LOW_I16 = std::numeric_limits<int16_t>::lowest();

    /// \brief 16-bit unsigned integer positive infinite value
    static const int16_t INF_I16 = std::numeric_limits<int16_t>::infinity();

    /// \brief 32bit unsigned integer maximum value
    static const uint32_t MAX_UI32 = std::numeric_limits<uint32_t>::max();

    /// \brief 32bit unsigned integer minimum value
    static const uint32_t MIN_UI32 = std::numeric_limits<uint32_t>::min();

    /// \brief 32bit unsigned integer lowest value. This is equivalent to
    /// IGN_UINT32_MIN, and is defined here for completeness.
    static const uint32_t LOW_UI32 = std::numeric_limits<uint32_t>::lowest();

    /// \brief 32-bit unsigned integer positive infinite value
    static const uint32_t INF_UI32 = std::numeric_limits<uint32_t>::infinity();

    /// \brief 32bit unsigned integer maximum value
    static const int32_t MAX_I32 = std::numeric_limits<int32_t>::max();

    /// \brief 32bit unsigned integer minimum value
    static const int32_t MIN_I32 = std::numeric_limits<int32_t>::min();

    /// \brief 32bit unsigned integer lowest value. This is equivalent to
    /// IGN_INT32_MIN, and is defined here for completeness.
    static const int32_t LOW_I32 = std::numeric_limits<int32_t>::lowest();

    /// \brief 32-bit unsigned integer positive infinite value
    static const int32_t INF_I32 = std::numeric_limits<int32_t>::infinity();

    /// \brief 64bit unsigned integer maximum value
    static const uint64_t MAX_UI64 = std::numeric_limits<uint64_t>::max();

    /// \brief 64bit unsigned integer minimum value
    static const uint64_t MIN_UI64 = std::numeric_limits<uint64_t>::min();

    /// \brief 64bit unsigned integer lowest value. This is equivalent to
    /// IGN_UINT64_MIN, and is defined here for completeness.
    static const uint64_t LOW_UI64 = std::numeric_limits<uint64_t>::lowest();

    /// \brief 64-bit unsigned integer positive infinite value
    static const uint64_t INF_UI64 = std::numeric_limits<uint64_t>::infinity();

    /// \brief 64bit unsigned integer maximum value
    static const int64_t MAX_I64 = std::numeric_limits<int64_t>::max();

    /// \brief 64bit unsigned integer minimum value
    static const int64_t MIN_I64 = std::numeric_limits<int64_t>::min();

    /// \brief 64bit unsigned integer lowest value. This is equivalent to
    /// IGN_INT64_MIN, and is defined here for completeness.
    static const int64_t LOW_I64 = std::numeric_limits<int64_t>::lowest();

    /// \brief 64-bit unsigned integer positive infinite value
    static const int64_t INF_I64 = std::numeric_limits<int64_t>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const int NAN_I = std::numeric_limits<int>::quiet_NaN();

    // variables created to deprecate macros in this file
    static const double IGN_DEPRECATED(3) DPRCT_MAX_D = MAX_D;
    static const double IGN_DEPRECATED(3) DPRCT_MIN_D = MIN_D;
    static const double IGN_DEPRECATED(3) DPRCT_LOW_D = LOW_D;
    static const double IGN_DEPRECATED(3) DPRCT_INF_D = INF_D;
    static const float IGN_DEPRECATED(3) DPRCT_MAX_F = MAX_F;
    static const float IGN_DEPRECATED(3) DPRCT_MIN_F = MIN_F;
    static const float IGN_DEPRECATED(3) DPRCT_LOW_F = LOW_F;
    static const float IGN_DEPRECATED(3) DPRCT_INF_F = INF_F;
    static const uint16_t IGN_DEPRECATED(3) DPRCT_MAX_UI16 = MAX_UI16;
    static const uint16_t IGN_DEPRECATED(3) DPRCT_MIN_UI16 = MIN_UI16;
    static const uint16_t IGN_DEPRECATED(3) DPRCT_LOW_UI16 = LOW_UI16;
    static const uint16_t IGN_DEPRECATED(3) DPRCT_INF_UI16 = INF_UI16;
    static const int16_t IGN_DEPRECATED(3) DPRCT_MAX_I16 = MAX_I16;
    static const int16_t IGN_DEPRECATED(3) DPRCT_MIN_I16 = MIN_I16;
    static const int16_t IGN_DEPRECATED(3) DPRCT_LOW_I16 = LOW_I16;
    static const int16_t IGN_DEPRECATED(3) DPRCT_INF_I16 = INF_I16;
    static const uint32_t IGN_DEPRECATED(3) DPRCT_MAX_UI32 = MAX_UI32;
    static const uint32_t IGN_DEPRECATED(3) DPRCT_MIN_UI32 = MIN_UI32;
    static const uint32_t IGN_DEPRECATED(3) DPRCT_LOW_UI32 = LOW_UI32;
    static const uint32_t IGN_DEPRECATED(3) DPRCT_INF_UI32 = INF_UI32;
    static const int32_t IGN_DEPRECATED(3) DPRCT_MAX_I32 = MAX_I32;
    static const int32_t IGN_DEPRECATED(3) DPRCT_MIN_I32 = MIN_I32;
    static const int32_t IGN_DEPRECATED(3) DPRCT_LOW_I32 = LOW_I32;
    static const int32_t IGN_DEPRECATED(3) DPRCT_INF_I32 = INF_I32;
    static const uint64_t IGN_DEPRECATED(3) DPRCT_MAX_UI64 = MAX_UI64;
    static const uint64_t IGN_DEPRECATED(3) DPRCT_MIN_UI64 = MIN_UI64;
    static const uint64_t IGN_DEPRECATED(3) DPRCT_LOW_UI64 = LOW_UI64;
    static const uint64_t IGN_DEPRECATED(3) DPRCT_INF_UI64 = INF_UI64;
    static const int64_t IGN_DEPRECATED(3) DPRCT_MAX_I64 = MAX_I64;
    static const int64_t IGN_DEPRECATED(3) DPRCT_MIN_I64 = MIN_I64;
    static const int64_t IGN_DEPRECATED(3) DPRCT_LOW_I64 = LOW_I64;
    static const int64_t IGN_DEPRECATED(3) DPRCT_INF_I64 = INF_I64;

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

    /// \brief The signum function.
    ///
    /// Returns 0 for zero values, -1 for negative values,
    /// +1 for positive values.
    /// \param[in] _value The value.
    /// \return The signum of the value.
    template<typename T>
    inline int sgn(T _value)
    {
      return (T(0) < _value) - (_value < T(0));
    }

    /// \brief The signum function.
    ///
    /// Returns 0 for zero values, -1 for negative values,
    /// +1 for positive values.
    /// \param[in] _value The value.
    /// \return The signum of the value.
    template<typename T>
    inline int signum(T _value)
    {
      return sgn(_value);
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
                      const T &_epsilon = T(1e-6))
    {
      IGN_FP_VOLATILE T diff = std::abs(_a - _b);
      return diff <= _epsilon;
    }

    /// \brief inequality test, within a tolerance
    /// \param[in] _a the first value
    /// \param[in] _b the second value
    /// \param[in] _epsilon the tolerance
    template<typename T>
    inline bool lessOrNearEqual(const T &_a, const T &_b,
                            const T &_epsilon = 1e-6)
    {
      return _a < _b + _epsilon;
    }

    /// \brief inequality test, within a tolerance
    /// \param[in] _a the first value
    /// \param[in] _b the second value
    /// \param[in] _epsilon the tolerance
    template<typename T>
    inline bool greaterOrNearEqual(const T &_a, const T &_b,
                               const T &_epsilon = 1e-6)
    {
      return _a > _b - _epsilon;
    }

    /// \brief get value at a specified precision
    /// \param[in] _a the number
    /// \param[in] _precision the precision
    /// \return the value for the specified precision
    template<typename T>
    inline T precision(const T &_a, const unsigned int &_precision)
    {
      auto p = std::pow(10, _precision);
      return static_cast<T>(std::round(_a * p) / p);
    }

    /// \brief Sort two numbers, such that _a <= _b
    /// \param[out] _a the first number
    /// \param[out] _b the second number
    template<typename T>
    inline void sort2(T &_a, T &_b)
    {
      using std::swap;
      if (_b < _a)
        swap(_a, _b);
    }

    /// \brief Sort three numbers, such that _a <= _b <= _c
    /// \param[out] _a the first number
    /// \param[out] _b the second number
    /// \param[out] _c the third number
    template<typename T>
    inline void sort3(T &_a, T &_b, T &_c)
    {
      // _a <= _b
      sort2(_a, _b);
      // _a <= _c, _b <= _c
      sort2(_b, _c);
      // _a <= _b <= _c
      sort2(_a, _b);
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

    /// \brief Round a number up to the nearest multiple. For example, if
    /// the input number is 12 and the multiple is 10, the result is 20.
    /// If the input number is negative, then the nearest multiple will be
    /// greater than or equal to the input number. For example, if the input
    /// number is -9 and the multiple is 2 then the output is -8.
    /// \param[in] _num Input number to round up.
    /// \param[in] _multiple The multiple. If the multiple is <= zero, then
    /// the input number is returned.
    /// \return The nearest multiple of _multiple that is greater than
    /// or equal to _num.
    inline int roundUpMultiple(int _num, int _multiple)
    {
      if (_multiple == 0)
        return _num;

      int remainder = std::abs(_num) % _multiple;
      if (remainder == 0)
        return _num;

      if (_num < 0)
        return -(std::abs(_num) - remainder);
      else
        return _num + _multiple - remainder;
    }

    /// \brief parse string into an integer
    /// \param[in] _input the string
    /// \return an integer, 0 or 0 and a message in the error stream
    inline int parseInt(const std::string &_input)
    {
      // Return NAN_I if it is empty
      if (_input.empty())
      {
        return NAN_I;
      }
      // Return 0 if it is all spaces
      else if (_input.find_first_not_of(' ') == std::string::npos)
      {
        return 0;
      }

      // Otherwise try standard library
      try
      {
        return std::stoi(_input);
      }
      // if that fails, return NAN_I
      catch(...)
      {
        return NAN_I;
      }
    }

    /// \brief parse string into float
    /// \param _input the string
    /// \return a floating point number (can be NaN) or 0 with a message in the
    /// error stream
    inline double parseFloat(const std::string &_input)
    {
      // Return NAN_D if it is empty
      if (_input.empty())
      {
        return NAN_D;
      }
      // Return 0 if it is all spaces
      else if (_input.find_first_not_of(' ') == std::string::npos)
      {
        return 0;
      }

      // Otherwise try standard library
      try
      {
        return std::stod(_input);
      }
      // if that fails, return NAN_D
      catch(...)
      {
        return NAN_D;
      }
    }

    /// \brief Convert a std::chrono::steady_clock::time_point to a seconds and
    /// nanoseconds pair.
    /// \param[in] _time The time point to convert.
    /// \return A pair where the first element is the number of seconds and
    /// the second is the number of nanoseconds.
    inline std::pair<int64_t, int64_t> timePointToSecNsec(
        const std::chrono::steady_clock::time_point &_time)
    {
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _time.time_since_epoch());
      auto now_s = std::chrono::duration_cast<std::chrono::seconds>(
        _time.time_since_epoch());
      int64_t seconds = now_s.count();
      int64_t nanoseconds = std::chrono::duration_cast
        <std::chrono::nanoseconds>(now_ns - now_s).count();
      return {seconds, nanoseconds};
    }

    /// \brief Convert seconds and nanoseconds to
    /// std::chrono::steady_clock::time_point.
    /// \param[in] _sec The seconds to convert.
    /// \param[in] _nanosec The nanoseconds to convert.
    /// \return A std::chrono::steady_clock::time_point based on the number of
    /// seconds and the number of nanoseconds.
    inline std::chrono::steady_clock::time_point secNsecToTimePoint(
        const uint64_t &_sec, const uint64_t &_nanosec)
    {
      auto duration = std::chrono::seconds(_sec) + std::chrono::nanoseconds(
        _nanosec);
      std::chrono::steady_clock::time_point result;
      using std::chrono::duration_cast;
      result += duration_cast<std::chrono::steady_clock::duration>(duration);
      return result;
    }

    /// \brief Convert seconds and nanoseconds to
    /// std::chrono::steady_clock::duration.
    /// \param[in] _sec The seconds to convert.
    /// \param[in] _nanosec The nanoseconds to convert.
    /// \return A std::chrono::steady_clock::duration based on the number of
    /// seconds and the number of nanoseconds.
    inline std::chrono::steady_clock::duration secNsecToDuration(
        const uint64_t &_sec, const uint64_t &_nanosec)
    {
      return std::chrono::seconds(_sec) + std::chrono::nanoseconds(
        _nanosec);
    }

    /// \brief Convert a std::chrono::steady_clock::duration to a seconds and
    /// nanoseconds pair.
    /// \param[in] _dur The duration to convert.
    /// \return A pair where the first element is the number of seconds and
    /// the second is the number of nanoseconds.
    inline std::pair<int64_t, int64_t> durationToSecNsec(
        const std::chrono::steady_clock::duration &_dur)
    {
      auto s = std::chrono::duration_cast<std::chrono::seconds>(_dur);
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_dur-s);
      return {s.count(), ns.count()};
    }

    // TODO(anyone): Replace this with std::chrono::days.
    /// This will exist in C++-20
    typedef std::chrono::duration<uint64_t, std::ratio<86400>> days;

    /// \brief break down durations
    /// NOTE: the template arguments must be properly ordered according
    /// to magnitude and there can be no duplicates.
    /// This function uses the braces initializer to split all the templated
    /// duration. The initializer will be called recursievely due the `...`
    /// \param[in] d Duration to break down
    /// \return A tuple based on the durations specified
    template<class...Durations, class DurationIn>
    std::tuple<Durations...> breakDownDurations(DurationIn d) {
      std::tuple<Durations...> retval;
      using discard = int[];
      (void)discard{0, (void((
        (std::get<Durations>(retval) =
          std::chrono::duration_cast<Durations>(d)),
        (d -= std::chrono::duration_cast<DurationIn>(
          std::get<Durations>(retval))))), 0)...};
      return retval;
    }

    /// \brief Convert a std::chrono::steady_clock::time_point to a string
    /// \param[in] _point The std::chrono::steady_clock::time_point to convert.
    /// \return A string formatted with the time_point
    inline std::string timePointToString(
      const std::chrono::steady_clock::time_point &_point)
    {
      auto duration = _point - secNsecToTimePoint(0, 0);
      auto cleanDuration = breakDownDurations<days,
                                              std::chrono::hours,
                                              std::chrono::minutes,
                                              std::chrono::seconds,
                                              std::chrono::milliseconds>(
                                                duration);
      std::ostringstream output_string;
      output_string << std::setw(2) << std::setfill('0')
                    << std::get<0>(cleanDuration).count() << " "
                    << std::setw(2) << std::setfill('0')
                    << std::get<1>(cleanDuration).count() << ":"
                    << std::setw(2) << std::setfill('0')
                    << std::get<2>(cleanDuration).count() << ":"
                    << std::setfill('0') << std::setw(6)
                    << std::fixed << std::setprecision(3)
                    << std::get<3>(cleanDuration).count() +
                       std::get<4>(cleanDuration).count()/1000.0;
      return output_string.str();
    }

    /// \brief Convert a std::chrono::steady_clock::duration to a string
    /// \param[in] _duration The std::chrono::steady_clock::duration to convert.
    /// \return A string formatted with the duration
    inline std::string durationToString(
      const std::chrono::steady_clock::duration &_duration)
    {
      auto cleanDuration = breakDownDurations<days,
                                              std::chrono::hours,
                                              std::chrono::minutes,
                                              std::chrono::seconds,
                                              std::chrono::milliseconds>(
                                                _duration);
      std::ostringstream outputString;
      outputString << std::setw(2) << std::setfill('0')
                    << std::get<0>(cleanDuration).count() << " "
                    << std::setw(2) << std::setfill('0')
                    << std::get<1>(cleanDuration).count() << ":"
                    << std::setw(2) << std::setfill('0')
                    << std::get<2>(cleanDuration).count() << ":"
                    << std::setfill('0') << std::setw(6)
                    << std::fixed << std::setprecision(3)
                    << std::get<3>(cleanDuration).count() +
                       std::get<4>(cleanDuration).count()/1000.0;
      return outputString.str();
    }

    // The following regex takes a time string in the general format of
    // "dd hh:mm:ss.nnn" where n is milliseconds, if just one number is
    // provided, it is assumed to be seconds
    static const std::regex time_regex(
        "^([0-9]+ ){0,1}"                       // day:
                                                // Any positive integer

        "(?:([1-9]:|[0-1][0-9]:|2[0-3]:){0,1}"  // hour:
                                                // 1 - 9:
                                                // 01 - 19:
                                                // 20 - 23:

        "([0-9]:|[0-5][0-9]:)){0,1}"            // minute:
                                                // 0 - 9:
                                                // 00 - 59:

        "(?:([0-9]|[0-5][0-9]){0,1}"            // second:
                                                // 0 - 9
                                                // 00 - 59

        "(\\.[0-9]{1,3}){0,1})$");              // millisecond:
                                                // .0 - .9
                                                // .00 - .99
                                                // .000 - 0.999


    /// \brief Check if the given string represents a time.
    /// An example time string is "0 00:00:00.000", which has the format
    /// "DAYS HOURS:MINUTES:SECONDS.MILLISECONDS"
    /// \return True if the regex was able to split the string otherwise False
    inline bool isTimeString(const std::string &_timeString)
    {
      std::smatch matches;

      // `matches` should always be a size of 6 as there are 6 matching
      // groups in the regex.
      // 1. The whole regex
      // 2. The days
      // 3. The hours
      // 4. The minutes
      // 5. The seconds
      // 6. The milliseconds
      // Note that the space will remain in the day match, the colon
      // will remain in the hour and minute matches, and the period will
      // remain in the millisecond match
      if (!std::regex_search(_timeString, matches, time_regex) ||
          matches.size() != 6)
        return false;

      std::string dayString = matches[1];

      // Days are the only unbounded number, so check first to see if stoi
      // runs successfully
      if (!dayString.empty())
      {
        // Erase the space
        dayString.erase(dayString.length() - 1);
        try
        {
          std::stoi(dayString);
        }
        catch (const std::out_of_range &)
        {
          return false;
        }
      }

      return true;
    }

    /// \brief Split a std::chrono::steady_clock::duration to a string
    /// \param[in] _timeString The string to convert in general format
    /// \param[out] numberDays number of days in the string
    /// \param[out] numberHours number of hours in the string
    /// \param[out] numberMinutes number of minutes in the string
    /// \param[out] numberSeconds number of seconds in the string
    /// \param[out] numberMilliseconds number of milliseconds in the string
    /// \return True if the regex was able to split the string otherwise False
    inline bool splitTimeBasedOnTimeRegex(
        const std::string &_timeString,
        uint64_t & numberDays, uint64_t & numberHours,
        uint64_t & numberMinutes, uint64_t & numberSeconds,
        uint64_t & numberMilliseconds)
    {
      std::smatch matches;

      // `matches` should always be a size of 6 as there are 6 matching
      // groups in the regex.
      // 1. The whole regex
      // 2. The days
      // 3. The hours
      // 4. The minutes
      // 5. The seconds
      // 6. The milliseconds
      // We can also index them as such below.
      // Note that the space will remain in the day match, the colon
      // will remain in the hour and minute matches, and the period will
      // remain in the millisecond match
      if (!std::regex_search(_timeString, matches, time_regex) ||
          matches.size() != 6)
        return false;

      std::string dayString = matches[1];
      std::string hourString = matches[2];
      std::string minuteString = matches[3];
      std::string secondString = matches[4];
      std::string millisecondString = matches[5];

      // Days are the only unbounded number, so check first to see if stoi
      // runs successfully
      if (!dayString.empty())
      {
        // Erase the space
        dayString.erase(dayString.length() - 1);
        try
        {
          numberDays = std::stoi(dayString);
        }
        catch (const std::out_of_range &)
        {
          return false;
        }
      }

      if (!hourString.empty())
      {
        // Erase the colon
        hourString.erase(hourString.length() - 1);
        numberHours = std::stoi(hourString);
      }

      if (!minuteString.empty())
      {
        // Erase the colon
        minuteString.erase(minuteString.length() - 1);
        numberMinutes = std::stoi(minuteString);
      }

      if (!secondString.empty())
      {
        numberSeconds = std::stoi(secondString);
      }

      if (!millisecondString.empty())
      {
        // Erase the period
        millisecondString.erase(0, 1);

        // Multiplier because "4" = 400 ms, "04" = 40 ms, and "004" = 4 ms
        numberMilliseconds = std::stoi(millisecondString) *
          static_cast<uint64_t>(1000 / pow(10, millisecondString.length()));
      }
      return true;
    }

    /// \brief Convert a string to a std::chrono::steady_clock::duration
    /// \param[in] _timeString The string to convert in general format
    /// "dd hh:mm:ss.nnn" where n is millisecond value
    /// \return A std::chrono::steady_clock::duration containing the
    /// string's time value. If it isn't possible to convert, the duration will
    /// be zero.
    inline std::chrono::steady_clock::duration stringToDuration(
        const std::string &_timeString)
    {
      using namespace std::chrono_literals;
      std::chrono::steady_clock::duration duration{
        std::chrono::steady_clock::duration::zero()};

      if (_timeString.empty())
        return duration;

      uint64_t numberDays = 0;
      uint64_t numberHours = 0;
      uint64_t numberMinutes = 0;
      uint64_t numberSeconds = 0;
      uint64_t numberMilliseconds = 0;

      if (!splitTimeBasedOnTimeRegex(_timeString, numberDays, numberHours,
                                     numberMinutes, numberSeconds,
                                     numberMilliseconds))
      {
        return duration;
      }

      // TODO(anyone): Replace below day conversion with std::chrono::days.
      /// This will exist in C++-20
      duration = std::chrono::steady_clock::duration::zero();
      auto delta = std::chrono::milliseconds(numberMilliseconds) +
        std::chrono::seconds(numberSeconds) +
        std::chrono::minutes(numberMinutes) +
        std::chrono::hours(numberHours) +
        std::chrono::hours(24 * numberDays);
      duration += delta;

      return duration;
    }

    /// \brief Convert a string to a std::chrono::steady_clock::time_point
    /// \param[in] _timeString The string to convert in general format
    /// "dd hh:mm:ss.nnn" where n is millisecond value
    /// \return A std::chrono::steady_clock::time_point containing the
    /// string's time value. If it isn't possible to convert, the time will
    /// be negative 1 second.
    inline std::chrono::steady_clock::time_point stringToTimePoint(
        const std::string &_timeString)
    {
      using namespace std::chrono_literals;
      std::chrono::steady_clock::time_point timePoint{-1s};

      if (_timeString.empty())
        return timePoint;

      uint64_t numberDays = 0;
      uint64_t numberHours = 0;
      uint64_t numberMinutes = 0;
      uint64_t numberSeconds = 0;
      uint64_t numberMilliseconds = 0;

      if (!splitTimeBasedOnTimeRegex(_timeString, numberDays, numberHours,
                                     numberMinutes, numberSeconds,
                                     numberMilliseconds))
      {
        return timePoint;
      }

      // TODO(anyone): Replace below day conversion with std::chrono::days.
      /// This will exist in C++-20
      timePoint = math::secNsecToTimePoint(0, 0);
      auto duration = std::chrono::milliseconds(numberMilliseconds) +
        std::chrono::seconds(numberSeconds) +
        std::chrono::minutes(numberMinutes) +
        std::chrono::hours(numberHours) +
        std::chrono::hours(24 * numberDays);
      timePoint += duration;

      return timePoint;
    }

    // Degrade precision on Windows, which cannot handle 'long double'
    // values properly. See the implementation of Unpair.
    // 32 bit ARM processors also define 'long double' to be the same
    // size as 'double', and must also be degraded
#if defined _MSC_VER || defined __arm__
    using PairInput = uint16_t;
    using PairOutput = uint32_t;
#else
    using PairInput = uint32_t;
    using PairOutput = uint64_t;
#endif

    /// \brief A pairing function that maps two values to a unique third
    /// value. This is an implement of Szudzik's function.
    /// \param[in] _a First value, must be a non-negative integer. On
    /// Windows this value is uint16_t. On Linux/OSX this value is uint32_t.
    /// \param[in] _b Second value, must be a non-negative integer. On
    /// Windows this value is uint16_t. On Linux/OSX this value is uint32_t.
    /// \return A unique non-negative integer value. On Windows the return
    /// value is uint32_t. On Linux/OSX the return value is uint64_t
    /// \sa Unpair
    PairOutput IGNITION_MATH_VISIBLE Pair(
        const PairInput _a, const PairInput _b);

    /// \brief The reverse of the Pair function. Accepts a key, produced
    /// from the Pair function, and returns a tuple consisting of the two
    /// non-negative integer values used to create the _key.
    /// \param[in] _key A non-negative integer generated from the Pair
    /// function. On Windows this value is uint32_t. On Linux/OSX, this
    /// value is uint64_t.
    /// \return A tuple that consists of the two non-negative integers that
    /// will generate _key when used with the Pair function. On Windows the
    /// tuple contains two uint16_t values. On Linux/OSX the tuple contains
    /// two uint32_t values.
    /// \sa Pair
    std::tuple<PairInput, PairInput> IGNITION_MATH_VISIBLE Unpair(
        const PairOutput _key);
    }
  }
}

#endif
