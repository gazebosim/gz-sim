/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_MATH_GAUSSMARKOVPROCESS_HH_
#define GZ_MATH_GAUSSMARKOVPROCESS_HH_

#include <chrono>
#include <gz/math/Export.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz
{
  namespace math
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    /** \class GaussMarkovProcess GaussMarkovProcess.hh\
     * gz/math/GaussMarkovProcess.hh
     **/
    /// \brief Implementation of a stationary gauss-markov process, also
    /// known as a Ornstein Ulenbeck process.
    ///
    /// See the Update(const clock::duration &) for details on the forumla
    /// used to update the process.
    ///
    /// ## Example usage
    ///
    /// \snippet examples/gauss_markov_process_example.cc complete
    class GZ_MATH_VISIBLE GaussMarkovProcess
    {
      // Default constructor. This sets all the parameters to zero.
      public: GaussMarkovProcess();

      /// \brief Create a process with the provided process parameters.
      /// This will also call Set(), and in turn Reset().
      /// \param[in] _start The start value of the process.
      /// \param[in] _theta The theta (\f$\theta\f$) parameter. A value of
      /// zero will be used if this parameter is negative.
      /// \param[in] _mu The mu (\f$\mu\f$) parameter.
      /// \param[in] _sigma The sigma (\f$\sigma\f$) parameter. A value of
      /// zero will be used if this parameter is negative.
      /// \sa Update(const clock::duration &)
      public: GaussMarkovProcess(double _start, double _theta, double _mu,
                  double _sigma);

      /// \brief Set the process parameters. This will also call Reset().
      /// \param[in] _start The start value of the process.
      /// \param[in] _theta The theta (\f$\theta\f$) parameter.
      /// \param[in] _mu The mu (\f$\mu\f$) parameter.
      /// \param[in] _sigma The sigma (\f$\sigma\f$) parameter.
      /// \sa Update(const clock::duration &)
      public: void Set(double _start, double _theta, double _mu, double _sigma);

      /// \brief Get the start value.
      /// \return The start value.
      /// \sa Set(double, double, double, double)
      public: double Start() const;

      /// \brief Get the current process value.
      /// \return The value of the process.
      public: double Value() const;

      /// \brief Get the theta (\f$\theta\f$) value.
      /// \return The theta value.
      /// \sa Set(double, double, double, double)
      public: double Theta() const;

      /// \brief Get the mu (\f$\mu\f$) value.
      /// \return The mu value.
      /// \sa Set(double, double, double, double)
      public: double Mu() const;

      /// \brief Get the sigma (\f$\sigma\f$) value.
      /// \return The sigma value.
      /// \sa Set(double, double, double, double)
      public: double Sigma() const;

      /// \brief Reset the process. This will set the current process value
      /// to the start value.
      public: void Reset();

      /// \brief Update the process and get the new value.
      ///
      /// The following equation is computed:
      ///
      /// \f$x_{t+1} += \theta * (\mu - x_t) * dt + \sigma * dW_t\f$
      ///
      /// where
      ///
      ///   * \f$\theta, \mu, \sigma\f$ are parameters specified by the
      ///     user. In order, the parameters are theta, mu, and sigma. Theta
      ///     and sigma must be greater than or equal to zero. You can think
      ///     of mu as representing the mean or equilibrium value, sigma as the
      ///     degree of volatility, and theta as the rate by which changes
      ///     dissipate and revert towards the mean.
      ///   * \f$dt\f$ is the time step in seconds.
      ///   * \f$dW_t\f$ is a random number drawm from a normal distribution
      ///     with mean of zero and variance of 1.
      ///   * \f$x_t\f$ is the current value of the Gauss-Markov process
      ///   * \f$x_{t+1}\f$ is the new value of the Gauss-Markvov process
      ///
      /// See also: https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process
      ///
      /// This implementation include a drift parameter, mu. In financial
      /// mathematics, this is known as a Vasicek model.
      ///
      /// \param[in] _dt Length of the timestep after which a new sample
      /// should be taken.
      /// \return The new value of this process.
      public: double Update(const clock::duration &_dt);

      public: double Update(double _dt);

      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif
