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
#ifndef GZ_SIM_DETAIL_FLAT_SET_HH_
#define GZ_SIM_DETAIL_FLAT_SET_HH_

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace detail
{

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
  /// \param value The value to insert in the set.
  /// \return True if it was inserted, false otherwise.
  public: bool insert(const T &_value)
  {
    const auto it = std::lower_bound(data.begin(), data.end(), _value);
    if (it == data.end() || *it != _value)
    {
      data.emplace(it, _value);
      return true;
    }
    return false;
  }

  /// \brief Erases an element
  /// \brief value the value to erase from the set.
  /// \return True if it was erased, false otherwise.
  public: bool erase(const T &_value)
  {
    const auto it = this->find(_value);
    if (it == data.end())
    {
      return false;
    }
    data.erase(it);
    return true;
  }

  /// \brief Check if set is empty
  /// \return True if empty, false otherwise.
  public: [[nodiscard]] bool empty() const
  {
    return data.empty();
  }

  /// \brief Get number of elements
  /// \return Number of elements.
  public: std::size_t size() const
  {
    return data.size();
  }

  /// \brief Clear all elements
  public: void clear()
  {
    data.clear();
  }

  /// \brief Type alias to make the struct compatible with STL algorithms
  public: using const_iterator = typename std::vector<T>::const_iterator;

  /// \brief Forwarding method for begin() iterator
  public: const_iterator begin() const { return data.begin(); }

  /// \brief Forwarding method for end() iterator
  public: const_iterator end() const { return data.end(); }

  /// \brief Find an element
  /// \param[in] value Element to find
  /// \return Iterator to element if found, end() otherwise.
  public: [[nodiscard]] const_iterator find(const T _value) const
  {
    const auto it = std::lower_bound(data.begin(), data.end(), _value);
    if (it != data.end() && *it == _value)
      return it;
    return data.end();
  }

  /// \brief Check if set contains an element
  /// \param[in] value Element to check
  /// \return True if contained, false otherwise.
  public: [[nodiscard]] bool contains(const T &_value) const
  {
    return this->find(_value) != this->end();
  }

  /// \brief Equality comparison
  public: bool operator==(const FlatSet<T> &_other) const
  {
    return data == _other.data;
  }

  /// \brief Inequality comparison
  public: bool operator!=(const FlatSet<T> &_other) const
  {
    return !(*this == _other);
  }
};

}  // namespace detail
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
#endif
