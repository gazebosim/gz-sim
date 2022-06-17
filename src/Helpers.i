/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifdef SWIGRUBY
%begin %{
#define HAVE_ISFINITE 1
%}
#endif

%module helpers
%{
#include <gz/math/Helpers.hh>
%}

template <typename T>
constexpr T GZ_MASSMATRIX3_DEFAULT_TOLERANCE = T(10);

// TODO(CH3): Deprecated. Remove on tock.
template <typename T>
constexpr T IGN_MASSMATRIX3_DEFAULT_TOLERANCE = T(10);

#define GZ_PI   3.14159265358979323846
#define GZ_PI_2 1.57079632679489661923
#define GZ_PI_4 0.78539816339744830962
#define GZ_SQRT2 1.41421356237309504880

#define GZ_SPHERE_VOLUME(_radius) (4.0 * GZ_PI * std::pow(_radius, 3)/3.0)
#define GZ_CYLINDER_VOLUME(_r, _l) (_l * GZ_PI * std::pow(_r, 2))
#define GZ_BOX_VOLUME(_x, _y, _z) (_x *_y * _z)
#define GZ_BOX_VOLUME_V(_v) (_v.X() *_v.Y() * _v.Z())

// TODO(CH3): Deprecated. Remove on tock.
#define IGN_PI GZ_PI
#define IGN_PI_2 GZ_PI_2
#define IGN_PI_4 GZ_PI_4
#define IGN_SQRT2 GZ_SQRT2

// TODO(CH3): Deprecated. Remove on tock.
#define IGN_SPHERE_VOLUME(_radius) GZ_SPHERE_VOLUME(_radius)
#define IGN_CYLINDER_VOLUME(_r, _l) GZ_CYLINDER_VOLUME(_r, _l)
#define IGN_BOX_VOLUME(_x, _y, _z) GZ_BOX_VOLUME(_x, _y, _z)
#define IGN_BOX_VOLUME_V(_v) GZ_BOX_VOLUME_V(_v)

namespace gz
{
  namespace math
  {
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

    template<typename T>
    T clamp(T _v, T _min, T _max);

    bool isnan(float _v);

    bool isnan(double _v);
    float fixnan(float _v);
    double fixnan(double _v);
    bool isEven(const int _v);
    bool isEven(const unsigned int _v);
    bool isOdd(const int _v);
    bool isOdd(const unsigned int _v);

    template<typename T>
    int sgn(T _value);

    template<typename T>
    int signum(T _value);

    template<typename T>
    T mean(const std::vector<T> &_values);

    template<typename T>
    T variance(const std::vector<T> &_values);

    template<typename T>
    T max(const std::vector<T> &_values);

    template<typename T>
    T min(const std::vector<T> &_values);

    template<typename T>
    bool equal(const T &_a, const T &_b,
               const T &_epsilon = T(1e-6));

    template<typename T>
    bool lessOrNearEqual(const T &_a, const T &_b,
                         const T &_epsilon = 1e-6);

    template<typename T>
    bool greaterOrNearEqual(const T &_a, const T &_b,
                            const T &_epsilon = 1e-6);

    template<typename T>
    T precision(const T &_a, const unsigned int &_precision);

    template<typename T>
    void sort2(T &_a, T &_b);

    template<typename T>
    void sort3(T &_a, T &_b, T &_c);

    bool isPowerOfTwo(unsigned int _x);

    unsigned int roundUpPowerOfTwo(unsigned int _x);

    int parseInt(const std::string &_input);

    double parseFloat(const std::string &_input);

    std::pair<int64_t, int64_t> durationToSecNsec(
        const std::chrono::steady_clock::duration &_dur);

    using PairInput = uint32_t;
    using PairOutput = uint64_t;

    PairOutput Pair(const PairInput _a, const PairInput _b);
    std::tuple<PairInput, PairInput> Unpair(const PairOutput _key);
  }
}
