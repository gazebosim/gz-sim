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
#ifndef GZ_SIM_FLAT_SET_HH_
#define GZ_SIM_FLAT_SET_HH_

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief Minimal implementation for a memory contiguous ordered set.
    /// Performs well on iteration, but slowly on insertion and deletion.
    // TODO(luca) Delete when the codebase moves to C++-23 and migrate to
    // std::flat_set.
    template<typename T>
    class FlatSet
    {
      /// \brief Sorted set data
      private: std::vector<T> data;

      /// \brief Inserts an element
      /// \return True if it was inserted, false otherwise.
      public: bool insert(T value)
      {
        const auto it = std::lower_bound(data.begin(), data.end(), value);
        if (it == data.end())
        {
          data.push_back(std::move(value));
          return true;
        }
        if (*it != value)
        {
          data.emplace(it, std::move(value));
          return true;
        }
        return false;
      }

      /// \brief Erases an element
      /// \return True if it was erased, false otherwise.
      public: bool erase(const T& value)
      {
        const auto it = std::lower_bound(data.begin(), data.end(), value);
        if (it == data.end())
        {
          return false;
        }
        data.erase(it);
        return true;
      }

      // Type aliases to make the struct compatible with STL algorithms
      using iterator = typename std::vector<T>::iterator;
      using const_iterator = typename std::vector<T>::const_iterator;

      // Forwarding methods
      iterator begin() { return data.begin(); }
      iterator end() { return data.end(); }

      const_iterator begin() const { return data.begin(); }
      const_iterator end() const { return data.end(); }
    };

    }
  }
}
#endif
