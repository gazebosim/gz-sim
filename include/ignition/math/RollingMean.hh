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
#ifndef IGNITION_MATH_ROLLINGMEAN_HH_
#define IGNITION_MATH_ROLLINGMEAN_HH_

#include <memory>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declarations.
    class RollingMeanPrivate;

    /// \brief A class that computes the mean over a series of data points.
    /// The window size determines the maximum number of data points. The
    /// oldest value is popped off when the window size is reached and
    /// a new value is pushed in.
    class IGNITION_MATH_VISIBLE RollingMean
    {
      /// \brief Constructor
      /// \param[in] _windowSize The window size to use. This value will be
      /// ignored if it is equal to zero.
      public: explicit RollingMean(size_t _windowSize = 10);

      /// \brief Destructor.
      public: ~RollingMean();

      /// \brief Get the mean value.
      /// \return The current mean value, or
      /// std::numeric_limits<double>::quiet_NaN() if data points are not
      /// present.
      public: double Mean() const;

      /// \brief Get the number of data points.
      /// \return The number of datapoints.
      public: size_t Count() const;

      /// \brief Insert a new value.
      /// \param[in] _value New value to insert.
      public: void Push(double _value);

      /// \brief Remove all the pushed values.
      public: void Clear();

      /// \brief Set the new window size. This will also clear the data.
      /// Nothing happens if the _windowSize is zero.
      /// \param[in] _windowSize The window size to use.
      public: void SetWindowSize(size_t _windowSize);

      /// \brief Get the window size.
      /// \return The window size.
      public: size_t WindowSize() const;

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Private data pointer.
      private: std::unique_ptr<RollingMeanPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}

#endif
