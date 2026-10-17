/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "OdometryNoise.hh"

#include <algorithm>
#include <cmath>

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
/// \brief Turns the median of a normal distribution's magnitude into its
/// standard deviation: 1 / 0.6745.
constexpr double kMedianToSigma{1.4826};

/// \brief Smallest track width used to size the noise.
constexpr double kMinTrack{0.05};

/// \brief Scale errors are cut at this many standard deviations, so that a
/// rare draw cannot flip a wheel.
constexpr double kScaleCut{3.0};
}

//////////////////////////////////////////////////
std::uint64_t NoiseSeed(unsigned int _worldSeed,
    const std::string &_scopedName)
{
  std::uint64_t hash{14695981039346656037ull};
  for (unsigned char c : _scopedName)
  {
    hash ^= c;
    hash *= 1099511628211ull;
  }
  for (int i = 0; i < 4; ++i)
  {
    hash ^= (_worldSeed >> (8 * i)) & 0xffu;
    hash *= 1099511628211ull;
  }
  return hash;
}

//////////////////////////////////////////////////
double OdometryNoise::ScaleSigma(double _fraction, double _track)
{
  return kMedianToSigma * _fraction * std::max(_track, kMinTrack) /
      kReferenceDistance;
}

//////////////////////////////////////////////////
double OdometryNoise::WalkStrength(double _fraction, double _track)
{
  return kMedianToSigma * _fraction * std::max(_track, kMinTrack) *
      std::sqrt(3.0) / (2.0 * std::sqrt(kReferenceDistance));
}

//////////////////////////////////////////////////
OdometryNoise::OdometryNoise(std::uint64_t _seed, double _fraction,
    double _track)
  : state(_seed)
{
  if (!std::isfinite(_fraction) || _fraction <= 0.0)
    return;

  this->walk = WalkStrength(_fraction, _track);
  const double sigma = ScaleSigma(_fraction, _track);
  this->scale = std::clamp(sigma * this->Normal(), -kScaleCut * sigma,
      kScaleCut * sigma);
}

//////////////////////////////////////////////////
double OdometryNoise::Scale() const
{
  return this->scale;
}

//////////////////////////////////////////////////
double OdometryNoise::Apply(double _travel)
{
  if (std::fpclassify(_travel) == FP_ZERO || !std::isfinite(_travel))
    return 0.0;

  double reported = _travel * (1.0 + this->scale);
  if (this->walk > 0.0)
    reported += this->walk * std::sqrt(std::abs(_travel)) * this->Normal();
  return reported;
}

//////////////////////////////////////////////////
std::uint64_t OdometryNoise::Next()
{
  // SplitMix64.
  this->state += 0x9e3779b97f4a7c15ull;
  std::uint64_t z = this->state;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}

//////////////////////////////////////////////////
double OdometryNoise::Normal()
{
  if (this->hasSpare)
  {
    this->hasSpare = false;
    return this->spare;
  }

  // Box-Muller. u1 is in (0, 1], so the logarithm is finite.
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  constexpr double kScale = 1.0 / 9007199254740992.0;
  const double u1 = (static_cast<double>(this->Next() >> 11) + 1.0) * kScale;
  const double u2 = static_cast<double>(this->Next() >> 11) * kScale;
  const double radius = std::sqrt(-2.0 * std::log(u1));
  this->spare = radius * std::sin(kTwoPi * u2);
  this->hasSpare = true;
  return radius * std::cos(kTwoPi * u2);
}
}
}
}
