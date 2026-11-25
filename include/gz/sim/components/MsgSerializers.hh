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
#ifndef GZ_SIM_COMPONENTS_MSGSERIALIZERS_HH_
#define GZ_SIM_COMPONENTS_MSGSERIALIZERS_HH_

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/double_v.pb.h>

#include <istream>
#include <ostream>
#include <vector>

#include <gz/sim/config.hh>

// This header holds component serializers for protobuf messages. Include it
// instead of Serialization.hh when a component only needs one of these, to
// avoid pulling in Conversions.hh.

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  /// \brief Serializer for components that hold `std::vector<double>`.
  class VectorDoubleSerializer
  {
    /// \brief Serialization
    /// \param[in] _out Output stream.
    /// \param[in] _vec Vector to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const std::vector<double> &_vec)
    {
      gz::msgs::Double_V msg;
      *msg.mutable_data() = {_vec.begin(), _vec.end()};
      auto result = msg.SerializeToOstream(&_out);
      (void)result;
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in Input stream.
    /// \param[in] _vec Vector to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                                             std::vector<double> &_vec)
    {
      gz::msgs::Double_V msg;
      if (!msg.ParseFromIstream(&_in))
      {
        return _in;
      }

      _vec = {msg.data().begin(), msg.data().end()};
      return _in;
    }
  };

  /// \brief Serializer for components that hold protobuf messages.
  class MsgSerializer
  {
    /// \brief Serialization
    /// \param[in] _out Output stream.
    /// \param[in] _msg Message to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
        const google::protobuf::Message &_msg)
    {
      auto result = _msg.SerializeToOstream(&_out);
      (void)result;
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in Input stream.
    /// \param[in] _msg Message to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
        google::protobuf::Message &_msg)
    {
      auto result = _msg.ParseFromIstream(&_in);
      (void)result;
      return _in;
    }
  };
}
}
}
}

#endif
