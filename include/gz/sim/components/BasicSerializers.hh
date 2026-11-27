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
#ifndef GZ_SIM_COMPONENTS_BASICSERIALIZERS_HH_
#define GZ_SIM_COMPONENTS_BASICSERIALIZERS_HH_

#include <cstddef>
#include <istream>
#include <iterator>
#include <ostream>
#include <string>
#include <vector>

#include <gz/sim/config.hh>

// This header holds component serializers that only depend on the standard
// library. Include it instead of Serialization.hh when a component only needs
// one of these, to avoid pulling in gz-msgs and Conversions.hh.

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  /// \brief Serializer for components that hold std::string.
  class StringSerializer
  {
    /// \brief Serialization
    /// \param[in] _out Output stream.
    /// \param[in] _data Data to serialize.
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
        const std::string &_data)
    {
      _out << _data;
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in Input stream.
    /// \param[in] _data Data to populate.
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
        std::string &_data)
    {
      _data = std::string(std::istreambuf_iterator<char>(_in), {});
      return _in;
    }
  };

  /// \brief Serializer for components that hold `std::vector<T>`, where
  /// T can be streamed with `<<` and `>>`.
  template <typename T>
  class VectorSerializer
  {
    /// \brief Serialization for `std::vector<T>` with serializable T.
    /// \param[in] _out Output stream.
    /// \param[in] _data The data to stream.
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
      const std::vector<T> &_data)
    {
      _out << _data.size();
      for (const auto& datum : _data)
        _out << " " << datum;
      return _out;
    }

    /// \brief Deserialization for `std::vector<T>` with serializable T.
    /// \param[in] _in Input stream.
    /// \param[out] _data The data to populate.
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
      std::vector<T> &_data)
    {
      size_t size;
      _in >> size;
      _data.resize(size);
      for (size_t i = 0; i < size; ++i)
      {
        _in >> _data[i];
      }
      return _in;
    }
  };
}
}
}
}

#endif
