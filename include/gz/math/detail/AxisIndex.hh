/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_MATH_DETAIL_AXIS_INDEX_LOOKUP_FIELD_HH_
#define GZ_MATH_DETAIL_AXIS_INDEX_LOOKUP_FIELD_HH_

#include <algorithm>
#include <map>
#include <optional>
#include <vector>

#include <cassert>

#include <gz/math/detail/InterpolationPoint.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {

      /// \brief Represents a sparse number line which can be searched via
      /// search for indices.
      template <typename T>
      class AxisIndex
      {
        /// \brief Binary tree with indices
        private: std::map<T, std::size_t> axisIndex;

        /// \brief Number of indices
        private: int numIndices{0};

        /// \brief Register the existance of a measurement at _value.
        /// \param[in] _value The position of the measurement.
        public: void AddIndexIfNotFound(T _value)
        {
          if (axisIndex.find(_value) == axisIndex.end())
          {
            axisIndex[_value] = numIndices;
            numIndices++;
          }
        }

        /// \brief Get the number of unique indices.
        /// \return The number of unique indices.
        public: std::size_t GetNumUniqueIndices() const
        {
          return axisIndex.size();
        }

        /// \brief Get the index of a measurement
        /// \param[in] _value The position of the measurement.
        /// \return The index of the measurement if found else return nullopt.
        public: std::optional<std::size_t> GetIndex(T _value) const
        {
          auto res = axisIndex.find(_value);
          if (res == axisIndex.end())
          {
            return std::nullopt;
          }
          else
          {
            return res->second;
          }
        }

        /// \brief Get interpolators for a measurement.
        /// \param[in] _value The position of the measurement.
        /// \param[in] _tol The tolerance for the search. Cannot be zero.
        /// \return The indices of the measurements that should be used for
        /// interpolation. If the value is out of range, an empty vector is
        /// returned. If the value is exact, a vector with a single index is
        /// returned otherwise return the two indices which should be used for
        /// interpolation.
        public: std::vector<InterpolationPoint1D<T>> GetInterpolators(
          const T &_value,
          double _tol = 1e-6) const
        {
          assert(_tol > 0);
          // Performs a BST to find the first element that is greater than or
          // equal to the value.
          auto it = axisIndex.lower_bound(_value);
          if (it == axisIndex.end())
          {
            // Out of range
            return {};
          }
          else if (fabs(it->first - _value) < _tol)
          {
            // Exact match
            return {InterpolationPoint1D<T>{it->first, it->second}};
          }
          else if (it == axisIndex.begin())
          {
            // Below range
            return {};
          }
          else
          {
            // Interpolate
            auto itPrev = it;
            itPrev--;
            return {InterpolationPoint1D<T>{itPrev->first, itPrev->second},
              InterpolationPoint1D<T>{it->first, it->second}};
          }
        }
      };
    }
  }
}
#endif
