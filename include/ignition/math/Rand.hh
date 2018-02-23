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
#ifndef IGNITION_MATH_RAND_HH_
#define IGNITION_MATH_RAND_HH_

#include <random>
#include <cmath>
#include <cstdint>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \def GeneratorType
    /// \brief std::mt19937
    typedef std::mt19937 GeneratorType;
    /// \def UniformRealDist
    /// \brief std::uniform_real_distribution<double>
    typedef std::uniform_real_distribution<double> UniformRealDist;
    /// \def NormalRealDist
    /// \brief std::normal_distribution<double>
    typedef std::normal_distribution<double> NormalRealDist;
    /// \def UniformIntDist
    /// \brief std::uniform_int<int>
    typedef std::uniform_int_distribution<int32_t> UniformIntDist;

    /// \class Rand Rand.hh ignition/math/Rand.hh
    /// \brief Random number generator class
    class IGNITION_MATH_VISIBLE Rand
    {
      /// \brief Set the seed value.
      /// \param[in] _seed The seed used to initialize the randon number
      /// generator.
      public: static void Seed(unsigned int _seed);

      /// \brief Get the seed value.
      /// \return The seed value used to initialize the random number
      /// generator.
      public: static unsigned int Seed();

      /// \brief Get a double from a uniform distribution
      /// \param[in] _min Minimum bound for the random number
      /// \param[in] _max Maximum bound for the random number
      public: static double DblUniform(double _min = 0, double _max = 1);

      /// \brief Get a double from a normal distribution
      /// \param[in] _mean Mean value for the distribution
      /// \param[in] _sigma Sigma value for the distribution
      public: static double DblNormal(double _mean = 0, double _sigma = 1);

      /// \brief Get an integer from a uniform distribution
      /// \param[in] _min Minimum bound for the random number
      /// \param[in] _max Maximum bound for the random number
      public: static int32_t IntUniform(int _min, int _max);

      /// \brief Get an integer from a normal distribution
      /// \param[in] _mean Mean value for the distribution
      /// \param[in] _sigma Sigma value for the distribution
      public: static int32_t IntNormal(int _mean, int _sigma);

      /// \brief Get a mutable reference to the seed (create the static
      /// member if it hasn't been created yet).
      private: static uint32_t &SeedMutable();

      /// \brief Get a mutable reference to the random generator (create the
      /// static member if it hasn't been created yet).
      private: static GeneratorType &RandGenerator();
    };
    }
  }
}
#endif
