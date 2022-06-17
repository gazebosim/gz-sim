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
#ifndef GZ_MATH_VECTOR3STATS_HH_
#define GZ_MATH_VECTOR3STATS_HH_

#include <string>
#include <gz/math/Helpers.hh>
#include <gz/math/SignalStats.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    /// \class Vector3Stats Vector3Stats.hh gz/math/Vector3Stats.hh
    /// \brief Collection of statistics for a Vector3 signal.
    class GZ_MATH_VISIBLE Vector3Stats
    {
      /// \brief Constructor
      public: Vector3Stats();

      /// \brief Add a new sample to the statistical measures.
      /// \param[in] _data New signal data point.
      public: void InsertData(const Vector3d &_data);

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

      /// \brief Get statistics for x component of signal.
      /// \return Statistics for x component of signal.
      public: const SignalStats &X() const;

      /// \brief Get statistics for y component of signal.
      /// \return Statistics for y component of signal.
      public: const SignalStats &Y() const;

      /// \brief Get statistics for z component of signal.
      /// \return Statistics for z component of signal.
      public: const SignalStats &Z() const;

      /// \brief Get statistics for magnitude component of signal.
      /// \return Statistics for magnitude component of signal.
      public: const SignalStats &Mag() const;

      /// \brief Get mutable reference to statistics for x component of signal.
      /// \return Statistics for x component of signal.
      public: SignalStats &X();

      /// \brief Get mutable reference to statistics for y component of signal.
      /// \return Statistics for y component of signal.
      public: SignalStats &Y();

      /// \brief Get mutable reference to statistics for z component of signal.
      /// \return Statistics for z component of signal.
      public: SignalStats &Z();

      /// \brief Get mutable reference to statistics for magnitude of signal.
      /// \return Statistics for magnitude of signal.
      public: SignalStats &Mag();

      /// \brief Pointer to private data.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

