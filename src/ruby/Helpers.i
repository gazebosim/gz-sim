/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module helpers
%{
#include <gz/math/Helpers.hh>
%}

%include "std_string.i"
%include "std_vector.i"

/// \brief Define GZ_PI, GZ_PI_2, and GZ_PI_4.
/// This was put here for Windows support.
#ifdef M_PI
#define GZ_PI M_PI
#define GZ_PI_2 M_PI_2
#define GZ_PI_4 M_PI_4
#define GZ_SQRT2 M_SQRT2
#else
#define GZ_PI   3.14159265358979323846
#define GZ_PI_2 1.57079632679489661923
#define GZ_PI_4 0.78539816339744830962
#define GZ_SQRT2 1.41421356237309504880
#endif

// TODO(CH3): Deprecated. Remove on tock.
#define IGN_PI GZ_PI
#define IGN_PI_2 GZ_PI_2
#define IGN_PI_4 GZ_PI_4
#define IGN_SQRT2 GZ_SQRT2

// The uppercase functions in the pythoncode block are defined with `#define` in cpp
// but in python this may generate some issues. A workaround is to create a Python function.
// With sort functions the issue is with the referenced arguments of a templated function,
// the workaround it's to define the functions in Python (sort2, sort3).

%pythoncode %{
import math
def gz_sphere_volume(_radius):
   return (4.0*GZ_PI*math.pow(_radius, 3)/3.0)

def gz_cylinder_volume(_r, _l):
  return (_l * GZ_PI * math.pow(_r, 2))

def gz_box_volume(_x, _y, _z):
  return (_x *_y * _z)

def gz_box_volume_v(_v):
  return (_v.x() *_v.y() * _v.z())

# TODO(CH3): Deprecated. Remove on tock.
def ign_sphere_volume(_radius):
   return gz_sphere_volume(_radius)

# TODO(CH3): Deprecated. Remove on tock.
def ign_cylinder_volume(_r, _l):
  return gz_cylinder_volume(_r, _l)

# TODO(CH3): Deprecated. Remove on tock.
def ign_box_volume(_x, _y, _z):
  return gz_box_volume(_x, _y, _z)

# TODO(CH3): Deprecated. Remove on tock.
def ign_box_volume_v(_v):
  return gz_box_volume_v(_v)

def sort2(_a, _b):
    def swap(s1, s2):
        return s2, s1
    if (_b < _a):
        _a, _b = swap(_a, _b)
    return _a, _b

def sort3(_a, _b, _c):
    _a, _b = sort2(_a, _b)
    _b, _c = sort2(_b, _c)
    _a, _b = sort2(_a, _b)
    return _a, _b, _c
%}

%template(VectorFloat) std::vector<float>;
%template(VectorDouble) std::vector<double>;
%template(VectorInt) std::vector<int>;

// When we use %rename this may conflict with the cpp templates, we will see a error:
// "warning 503: Can't wrap 'my_function< int >' unless renamed to a valid identifier"
// And you cannot try to anticipate the rename in your %template:
// %template(MyIntFunction) my_function<int>;
// Because then you'll get
// error : Template 'myfunction' undefined.
// Workaround it's to split the template functions and the normal ones
namespace gz
{
  /// \brief Math classes and function useful in robot applications.
  namespace math
  {
  %rename(clamp) clamp;
  template<typename T>
  T clamp(T _v, T _min, T _max);

  %rename(sgn) sgn;
  template<typename T>
  int sgn(T _value);

  %rename(signum) signum;
  template<typename T>
  int signum(T _value);

  %rename(mean) mean;
  template<typename T>
  T mean(const std::vector<T> &_values);

  %rename(variance) variance;
  template<typename T>
  T variance(const std::vector<T> &_values);

  %rename(max) max;
  template<typename T>
  T max(const std::vector<T> &_values);

  %rename(min) min;
  template<typename T>
  T min(const std::vector<T> &_values);

  %rename(equal) equal;
  template<typename T>
  bool equal(const T &_a, const T &_b,
             const T &_epsilon = T(1e-6));

  %rename(precision) precision;
  template<typename T>
  T precision(const T &_a, const unsigned int &_precision);

  %rename(less_or_near_equal) lessOrNearEqual;
  template<typename T>
  bool lessOrNearEqual(const T &_a, const T &_b,
                       const T &_epsilon = 1e-6);

  %rename(greater_or_near_equal) greaterOrNearEqual;
  template<typename T>
  bool greaterOrNearEqual(const T &_a, const T &_b,
                          const T &_epsilon = 1e-6);

  %template(clamp) clamp<float>;
  %template(clamp) clamp<int>;
  %template(sgn) sgn<float>;
  %template(sgn) sgn<int>;
  %template(signum) signum<float>;
  %template(signum) signum<int>;
  %template(mean) mean<float>;
  %template(variance) variance<float>;
  %template(max) max<float>;
  %template(min) min<float>;
  %template(precision) precision<float>;
  %template(precision) precision<int>;
  %template(equal) equal<float>;
  %template(equal) equal<int>;
  %template(less_or_near_equal) lessOrNearEqual<float>;
  %template(less_or_near_equal) lessOrNearEqual<int>;
  %template(greater_or_near_equal) greaterOrNearEqual<float>;
  %template(greater_or_near_equal) greaterOrNearEqual<int>;
  }
}

namespace gz
{
  /// \brief Math classes and function useful in robot applications.
  namespace math
  {
    %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

    static const size_t GZ_ZERO_SIZE_T  = 0u;

    static const size_t GZ_ONE_SIZE_T   = 1u;

    static const size_t GZ_TWO_SIZE_T   = 2u;

    static const size_t GZ_THREE_SIZE_T = 3u;

    static const size_t GZ_FOUR_SIZE_T  = 4u;

    static const size_t GZ_FIVE_SIZE_T  = 5u;

    static const size_t GZ_SIX_SIZE_T   = 6u;

    static const size_t GZ_SEVEN_SIZE_T = 7u;

    static const size_t GZ_EIGHT_SIZE_T = 8u;

    static const size_t GZ_NINE_SIZE_T  = 9u;

    // TODO(CH3): Deprecated. Remove on tock.
    constexpr auto GZ_DEPRECATED(7) IGN_ZERO_SIZE_T  = &GZ_ZERO_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_ONE_SIZE_T   = &GZ_ONE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_TWO_SIZE_T   = &GZ_TWO_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_THREE_SIZE_T = &GZ_THREE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_FOUR_SIZE_T  = &GZ_FOUR_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_FIVE_SIZE_T  = &GZ_FIVE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_SIX_SIZE_T   = &GZ_SIX_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_SEVEN_SIZE_T = &GZ_SEVEN_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_EIGHT_SIZE_T = &GZ_EIGHT_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_NINE_SIZE_T  = &GZ_NINE_SIZE_T;

    static const double MAX_D = std::numeric_limits<double>::max();

    static const double MIN_D = std::numeric_limits<double>::min();

    static const double LOW_D = std::numeric_limits<double>::lowest();

    static const double INF_D = std::numeric_limits<double>::infinity();

    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    static const float MAX_F = std::numeric_limits<float>::max();

    static const float MIN_F = std::numeric_limits<float>::min();

    static const float LOW_F = std::numeric_limits<float>::lowest();

    static const float INF_F = std::numeric_limits<float>::infinity();

    static const float NAN_F = std::numeric_limits<float>::quiet_NaN();

    static const uint16_t MAX_UI16 = std::numeric_limits<uint16_t>::max();

    static const uint16_t MIN_UI16 = std::numeric_limits<uint16_t>::min();

    static const uint16_t LOW_UI16 = std::numeric_limits<uint16_t>::lowest();

    static const uint16_t INF_UI16 = std::numeric_limits<uint16_t>::infinity();

    static const int16_t MAX_I16 = std::numeric_limits<int16_t>::max();

    static const int16_t MIN_I16 = std::numeric_limits<int16_t>::min();

    static const int16_t LOW_I16 = std::numeric_limits<int16_t>::lowest();

    static const int16_t INF_I16 = std::numeric_limits<int16_t>::infinity();

    static const uint32_t MAX_UI32 = std::numeric_limits<uint32_t>::max();

    static const uint32_t MIN_UI32 = std::numeric_limits<uint32_t>::min();

    static const uint32_t LOW_UI32 = std::numeric_limits<uint32_t>::lowest();

    static const uint32_t INF_UI32 = std::numeric_limits<uint32_t>::infinity();

    static const int32_t MAX_I32 = std::numeric_limits<int32_t>::max();

    static const int32_t MIN_I32 = std::numeric_limits<int32_t>::min();

    static const int32_t LOW_I32 = std::numeric_limits<int32_t>::lowest();

    static const int32_t INF_I32 = std::numeric_limits<int32_t>::infinity();

    static const uint64_t MAX_UI64 = std::numeric_limits<uint64_t>::max();

    static const uint64_t MIN_UI64 = std::numeric_limits<uint64_t>::min();

    static const uint64_t LOW_UI64 = std::numeric_limits<uint64_t>::lowest();

    static const uint64_t INF_UI64 = std::numeric_limits<uint64_t>::infinity();

    static const int64_t MAX_I64 = std::numeric_limits<int64_t>::max();

    static const int64_t MIN_I64 = std::numeric_limits<int64_t>::min();

    static const int64_t LOW_I64 = std::numeric_limits<int64_t>::lowest();

    static const int64_t INF_I64 = std::numeric_limits<int64_t>::infinity();

    static const int NAN_I = std::numeric_limits<int>::quiet_NaN();

    bool isnan(float _v);
    bool isnan(double _v);
    float fixnan(float _v);
    double fixnan(double _v);
    bool isEven(const int _v);
    bool isEven(const unsigned int _v);
    bool isOdd(const int _v);
    bool isOdd(const unsigned int _v);

    bool isPowerOfTwo(unsigned int _x);
    unsigned int roundUpPowerOfTwo(unsigned int _x);
    int roundUpMultiple(int _num, int _multiple);
    int parseInt(const std::string &_input);
    double parseFloat(const std::string &_input);
    bool splitTimeBasedOnTimeRegex(
        const std::string &_timeString,
        uint64_t & numberDays, uint64_t & numberHours,
        uint64_t & numberMinutes, uint64_t & numberSeconds,
        uint64_t & numberMilliseconds);
  }
}
