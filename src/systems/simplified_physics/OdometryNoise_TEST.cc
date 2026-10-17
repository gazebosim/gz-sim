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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "OdometryNoise.hh"

using namespace gz;
using namespace sim::systems::simplified_physics;

namespace
{
constexpr double kFraction{0.015};
constexpr double kTrack{1.25};

/// \brief Position error of differential drive odometry after a straight
/// run, from the wheel travel that two noisy encoders report.
/// \param[in] _seed Seed of the left wheel; the right wheel uses the next.
/// \param[in] _distance Length of the run, in meters.
/// \param[in] _step Travel per step, in meters.
/// \return Distance between the odometry's end point and the true one.
double RunError(unsigned int _seed, double _distance, double _step)
{
  OdometryNoise left(NoiseSeed(_seed, "left"), kFraction, kTrack);
  OdometryNoise right(NoiseSeed(_seed, "right"), kFraction, kTrack);

  double x{0.0};
  double y{0.0};
  double heading{0.0};
  const int steps = static_cast<int>(std::round(_distance / _step));
  for (int i = 0; i < steps; ++i)
  {
    const double l = left.Apply(_step);
    const double r = right.Apply(_step);
    const double forward = 0.5 * (l + r);
    const double turn = (r - l) / kTrack;
    // Second order integration, like the odometry in DiffDrive.
    x += forward * std::cos(heading + 0.5 * turn);
    y += forward * std::sin(heading + 0.5 * turn);
    heading += turn;
  }
  return std::hypot(x - _distance, y);
}

/// \brief Median of a list of numbers.
double Median(std::vector<double> _values)
{
  std::sort(_values.begin(), _values.end());
  return _values[_values.size() / 2];
}
}

/////////////////////////////////////////////////
TEST(OdometryNoise, SeedDependsOnSeedAndName)
{
  EXPECT_EQ(NoiseSeed(7, "w/m/left"), NoiseSeed(7, "w/m/left"));
  EXPECT_NE(NoiseSeed(7, "w/m/left"), NoiseSeed(7, "w/m/right"));
  EXPECT_NE(NoiseSeed(7, "w/m/left"), NoiseSeed(8, "w/m/left"));

  // Pinned values: the seed must not change with the platform or the
  // standard library, or recorded runs would not repeat.
  EXPECT_EQ(5558979605539197941ull, NoiseSeed(0, ""));
  EXPECT_EQ(3399809067505294439ull, NoiseSeed(12345, "w/m/left"));
}

/////////////////////////////////////////////////
TEST(OdometryNoise, SameSeedSameNoise)
{
  OdometryNoise a(NoiseSeed(1, "j"), kFraction, kTrack);
  OdometryNoise b(NoiseSeed(1, "j"), kFraction, kTrack);
  OdometryNoise c(NoiseSeed(2, "j"), kFraction, kTrack);

  EXPECT_DOUBLE_EQ(a.Scale(), b.Scale());
  EXPECT_NE(a.Scale(), c.Scale());
  for (int i = 0; i < 100; ++i)
    EXPECT_DOUBLE_EQ(a.Apply(0.001), b.Apply(0.001));

  // Pinned values of the stream, computed independently in Python.
  OdometryNoise pinned(NoiseSeed(12345, "w/m/left"), kFraction, kTrack);
  EXPECT_NEAR(-0.0025285738180097876, pinned.Scale(), 1e-15);
  EXPECT_NEAR(0.0011479008906233126, pinned.Apply(0.001), 1e-15);
}

/////////////////////////////////////////////////
TEST(OdometryNoise, IdleStepsDrawNothing)
{
  OdometryNoise a(NoiseSeed(1, "j"), kFraction, kTrack);
  OdometryNoise b(NoiseSeed(1, "j"), kFraction, kTrack);
  std::vector<double> withIdle;
  std::vector<double> without;
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_EQ(0.0, a.Apply(0.0));
    withIdle.push_back(a.Apply(0.002));
    without.push_back(b.Apply(0.002));
  }
  EXPECT_EQ(without, withIdle);
}

/////////////////////////////////////////////////
TEST(OdometryNoise, OffMeansExact)
{
  for (const double fraction : {0.0, -1.0, std::nan("")})
  {
    OdometryNoise noise(NoiseSeed(1, "j"), fraction, kTrack);
    EXPECT_EQ(0.0, noise.Scale());
    EXPECT_EQ(0.123, noise.Apply(0.123));
    EXPECT_EQ(-0.5, noise.Apply(-0.5));
  }
}

/////////////////////////////////////////////////
TEST(OdometryNoise, ScaleErrorHasTheDocumentedSpread)
{
  const double sigma = OdometryNoise::ScaleSigma(kFraction, kTrack);
  EXPECT_NEAR(1.4826 * kFraction * kTrack / 10.0, sigma, 1e-12);

  const int count = 4000;
  double sum{0.0};
  double sumSquares{0.0};
  double largest{0.0};
  for (int i = 0; i < count; ++i)
  {
    OdometryNoise noise(NoiseSeed(static_cast<unsigned int>(i), "w"),
        kFraction, kTrack);
    sum += noise.Scale();
    sumSquares += noise.Scale() * noise.Scale();
    largest = std::max(largest, std::abs(noise.Scale()));
  }
  EXPECT_NEAR(0.0, sum / count, 0.05 * sigma);
  EXPECT_NEAR(1.0, std::sqrt(sumSquares / count) / sigma, 0.05);
  EXPECT_LE(largest, 3.0 * sigma * (1.0 + 1e-12));
}

/////////////////////////////////////////////////
TEST(OdometryNoise, StepNoiseIsZeroMeanAndScalesWithSqrtOfTravel)
{
  const double q = OdometryNoise::WalkStrength(kFraction, kTrack);
  OdometryNoise noise(NoiseSeed(3, "w"), kFraction, kTrack);
  const double scale = noise.Scale();

  const int count = 40000;
  for (const double travel : {0.001, 0.004})
  {
    double sum{0.0};
    double sumSquares{0.0};
    for (int i = 0; i < count; ++i)
    {
      const double error = noise.Apply(travel) - travel * (1.0 + scale);
      sum += error;
      sumSquares += error * error;
    }
    const double expected = q * std::sqrt(travel);
    EXPECT_NEAR(0.0, sum / count, 0.03 * expected) << travel;
    EXPECT_NEAR(1.0, std::sqrt(sumSquares / count) / expected, 0.03)
        << travel;
  }
}

/////////////////////////////////////////////////
TEST(OdometryNoise, MedianDriftOfADifferentialDriveMatchesTheFraction)
{
  // The documented contract: after a 10 m straight run, the median end
  // point error of the odometry is the drift fraction times the distance.
  const int runs = 800;
  std::vector<double> fine;
  std::vector<double> coarse;
  for (int i = 0; i < runs; ++i)
  {
    const auto seed = static_cast<unsigned int>(i);
    fine.push_back(RunError(seed, 10.0, 0.001));
    coarse.push_back(RunError(seed, 10.0, 0.01));
  }

  const double expected = kFraction * 10.0;
  EXPECT_NEAR(expected, Median(fine), 0.12 * expected);
  // Cutting the path into ten times fewer steps gives the same spread.
  EXPECT_NEAR(expected, Median(coarse), 0.12 * expected);
}
