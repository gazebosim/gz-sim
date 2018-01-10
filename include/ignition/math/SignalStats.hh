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
#ifndef IGNITION_MATH_SIGNALSTATS_HH_
#define IGNITION_MATH_SIGNALSTATS_HH_

#include <map>
#include <memory>
#include <string>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \brief Forward declare private data class.
    class SignalStatisticPrivate;

    /// \class SignalStatistic SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Statistical properties of a discrete time scalar signal.
    class IGNITION_MATH_VISIBLE SignalStatistic
    {
      /// \brief Constructor
      public: SignalStatistic();

      /// \brief Destructor
      public: virtual ~SignalStatistic();

      /// \brief Copy constructor
      /// \param[in] _ss SignalStatistic to copy
      public: SignalStatistic(const SignalStatistic &_ss);

      /// \brief Get the current value of the statistical measure.
      /// \return Current value of the statistical measure.
      public: virtual double Value() const = 0;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return Short name of the statistical measure.
      public: virtual std::string ShortName() const = 0;

      /// \brief Get number of data points in measurement.
      /// \return Number of data points in measurement.
      public: virtual size_t Count() const;

      /// \brief Add a new sample to the statistical measure.
      /// \param[in] _data New signal data point.
      public: virtual void InsertData(const double _data) = 0;

      /// \brief Forget all previous data.
      public: virtual void Reset();

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Pointer to private data.
      protected: std::unique_ptr<SignalStatisticPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    /// \}

    /// \class SignalMaximum SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Computing the maximum value of a discretely sampled signal.
    class IGNITION_MATH_VISIBLE SignalMaximum : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "max"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \class SignalMean SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Computing the mean value of a discretely sampled signal.
    class IGNITION_MATH_VISIBLE SignalMean : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "mean"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \class SignalMinimum SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Computing the minimum value of a discretely sampled signal.
    class IGNITION_MATH_VISIBLE SignalMinimum : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "min"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \class SignalRootMeanSquare SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Computing the square root of the mean squared value
    /// of a discretely sampled signal.
    class IGNITION_MATH_VISIBLE SignalRootMeanSquare : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "rms"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \class SignalMaxAbsoluteValue SignalStats.hh
    /// ignition/math/SignalStats.hh
    /// \brief Computing the maximum of the absolute value
    /// of a discretely sampled signal.
    /// Also known as the maximum norm, infinity norm, or supremum norm.
    class IGNITION_MATH_VISIBLE SignalMaxAbsoluteValue : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "maxAbs"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \class SignalVariance SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Computing the incremental variance
    /// of a discretely sampled signal.
    class IGNITION_MATH_VISIBLE SignalVariance : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Value() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "var"
      public: virtual std::string ShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(const double _data);
    };
    /// \}

    /// \brief Forward declare private data class.
    class SignalStatsPrivate;

    /// \class SignalStats SignalStats.hh ignition/math/SignalStats.hh
    /// \brief Collection of statistics for a scalar signal.
    class IGNITION_MATH_VISIBLE SignalStats
    {
      /// \brief Constructor
      public: SignalStats();

      /// \brief Destructor
      public: ~SignalStats();

      /// \brief Copy constructor
      /// \param[in] _ss SignalStats to copy
      public: SignalStats(const SignalStats &_ss);

      /// \brief Get number of data points in first statistic.
      /// Technically you can have different numbers of data points
      /// in each statistic if you call InsertStatistic after InsertData,
      /// but this is not a recommended use case.
      /// \return Number of data points in first statistic.
      public: size_t Count() const;

      /// \brief Get the current values of each statistical measure,
      /// stored in a map using the short name as the key.
      /// \return Map with short name of each statistic as key
      /// and value of statistic as the value.
      public: std::map<std::string, double> Map() const;

      /// \brief Add a new sample to the statistical measures.
      /// \param[in] _data New signal data point.
      public: void InsertData(const double _data);

      /// \brief Add a new type of statistic.
      /// \param[in] _name Short name of new statistic.
      /// Valid values include:
      ///  "maxAbs"
      ///  "mean"
      ///  "rms"
      /// \return True if statistic was successfully added,
      /// false if name was not recognized or had already
      /// been inserted.
      public: bool InsertStatistic(const std::string &_name);

      /// \brief Add multiple statistics.
      /// \param[in] _names Comma-separated list of new statistics.
      /// For example, all statistics could be added with:
      ///  "maxAbs,mean,rms"
      /// \return True if all statistics were successfully added,
      /// false if any names were not recognized or had already
      /// been inserted.
      public: bool InsertStatistics(const std::string &_names);

      /// \brief Forget all previous data.
      public: void Reset();

      /// \brief Assignment operator
      /// \param[in] _v A SignalStats to copy
      /// \return this
      public: SignalStats &operator=(const SignalStats &_s);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Pointer to private data.
      private: std::unique_ptr<SignalStatsPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
    /// \}
  }
}
#endif

