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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_ODOMETRY_NOISE_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_ODOMETRY_NOISE_HH_

#include <cstdint>
#include <string>

#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief Seed of the noise of one wheel.
  ///
  /// The seed depends on the world seed and on the scoped name of the joint
  /// only. A wheel gets the same noise in every run with the same world seed,
  /// whatever else the world holds and in whichever order models spawn. The
  /// hash is FNV-1a, so the result does not depend on the standard library.
  ///
  /// \param[in] _worldSeed The seed of the world (`math::Rand::Seed()`).
  /// \param[in] _scopedName Scoped name of the wheel joint.
  /// \return A 64 bit seed.
  std::uint64_t NoiseSeed(unsigned int _worldSeed,
      const std::string &_scopedName);

  /// \brief Error that a wheel encoder adds to the wheel travel, so that
  /// wheel odometry drifts the way a real robot's does.
  ///
  /// Each wheel gets two errors:
  ///
  /// * a scale error s, drawn once and kept for the life of the wheel. The
  ///   wheel reports travel * (1 + s). A tire radius that differs from the
  ///   nominal one does this.
  /// * a random walk. Every step adds zero-mean noise with a standard
  ///   deviation of q * sqrt(travel of the step), with q in m / sqrt(m). The
  ///   variance of the sum over a path then depends on the length of the path
  ///   and not on how the path is cut into steps.
  ///
  /// Both are sized from one number, the drift fraction f, so that a
  /// differential drive with track b that drives a straight line of
  /// kReferenceDistance (10 m) ends with a median position error of
  /// f * 10 m in the odometry. The error is almost all lateral: the scale
  /// difference of the two wheels and the random walk turn the heading
  /// estimate, and the heading error moves the end point sideways. Both
  /// contribute half of the squared error:
  ///
  ///   sigma_s = c f b / D            (c = 1.4826, D = 10 m)
  ///   q       = c f b sqrt(3) / (2 sqrt(D))
  ///
  /// where c turns a median into a standard deviation of a normal variable.
  /// Longer paths drift further in absolute terms (the lateral error of the
  /// scale part grows with D squared), as they do on a real robot. A fixed
  /// world seed repeats the noise exactly. The generator is SplitMix64 with a
  /// Box-Muller transform, so the stream is the same on every platform.
  ///
  /// The reported wheel velocity is the noisy travel of the step divided by
  /// the step. Its noise is white, with a standard deviation of
  /// q sqrt(v / dt) on a wheel that rolls at v m/s: 7.7 percent of the wheel
  /// speed at 1 m/s, 1 ms steps, f = 0.015 and b = 0.4 m, and more at smaller
  /// steps. Use the position for odometry and filter the velocity.
  class OdometryNoise
  {
    /// \brief Length of the reference path, in meters.
    public: static constexpr double kReferenceDistance{10.0};

    /// \brief Standard deviation of the scale error for a drift fraction and
    /// a track width.
    /// \param[in] _fraction Drift fraction f.
    /// \param[in] _track Track width b, in meters.
    /// \return sigma_s, dimensionless.
    public: static double ScaleSigma(double _fraction, double _track);

    /// \brief Strength q of the random walk.
    /// \param[in] _fraction Drift fraction f.
    /// \param[in] _track Track width b, in meters.
    /// \return q in m / sqrt(m).
    public: static double WalkStrength(double _fraction, double _track);

    /// \brief Constructor. Draws the scale error.
    /// \param[in] _seed Seed, see NoiseSeed().
    /// \param[in] _fraction Drift fraction f. A non-positive or non-finite
    /// value gives no noise.
    /// \param[in] _track Track width of the model in meters. At least 5 cm
    /// is used.
    public: OdometryNoise(std::uint64_t _seed, double _fraction,
                double _track);

    /// \brief The scale error s of this wheel.
    /// \return s, dimensionless.
    public: double Scale() const;

    /// \brief Travel that the wheel reports for a step.
    /// \param[in] _travel Travel of the wheel contact point in this step, in
    /// meters, negative for backward. Zero travel gives zero and draws no
    /// random number.
    /// \return Reported travel in meters.
    public: double Apply(double _travel);

    /// \brief Next number of a standard normal distribution.
    /// \return A draw.
    private: double Normal();

    /// \brief Next 64 random bits.
    /// \return The bits.
    private: std::uint64_t Next();

    /// \brief Generator state.
    private: std::uint64_t state{0};

    /// \brief Scale error.
    private: double scale{0.0};

    /// \brief Random walk strength.
    private: double walk{0.0};

    /// \brief Second number of the last Box-Muller pair.
    private: double spare{0.0};

    /// \brief True if spare holds a number.
    private: bool hasSpare{false};
  };
}
}
}

#endif
