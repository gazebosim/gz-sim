/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_SIGNALSTATSPRIVATE_HH_
#define IGNITION_MATH_SIGNALSTATSPRIVATE_HH_

#include <memory>
#include <vector>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    /// \brief Private data class for the SignalStatistic class.
    class SignalStatisticPrivate
    {
      /// \brief Scalar representation of signal data.
      public: double data;

      /// \brief Scalar representation of extra signal data.
      /// For example, the standard deviation statistic needs an extra variable.
      public: double extraData;

      /// \brief Count of data values in mean.
      public: unsigned int count;

      /// \brief Clone the SignalStatisticPrivate object. Used for implementing
      /// copy semantics.
      public: std::unique_ptr<SignalStatisticPrivate> Clone() const
      {
        std::unique_ptr<SignalStatisticPrivate> dataPtr(
            new SignalStatisticPrivate(*this));
        return dataPtr;
      }
    };

    class SignalStatistic;

    /// \def SignalStatisticPtr
    /// \brief Shared pointer to SignalStatistic object
    typedef std::shared_ptr<SignalStatistic> SignalStatisticPtr;

    /// \def SignalStatistic_V
    /// \brief Vector of SignalStatisticPtr
    typedef std::vector<SignalStatisticPtr> SignalStatistic_V;

    /// \brief Private data class for the SignalStats class.
    class SignalStatsPrivate
    {
      /// \brief Vector of `SignalStatistic`s.
      public: SignalStatistic_V stats;

      /// \brief Clone the SignalStatsPrivate object. Used for implementing
      /// copy semantics.
      public: std::unique_ptr<SignalStatsPrivate> Clone() const
      {
        std::unique_ptr<SignalStatsPrivate> dataPtr(
            new SignalStatsPrivate(*this));
        return dataPtr;
      }
    };
    }
  }
}
#endif
