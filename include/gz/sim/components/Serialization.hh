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
#ifndef GZ_SIM_COMPONENTS_SERIALIZATION_HH_
#define GZ_SIM_COMPONENTS_SERIALIZATION_HH_

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message_lite.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/double_v.pb.h>

#include <string>
#include <vector>
#include <sdf/Sensor.hh>

#include <gz/sim/Conversions.hh>
#include <gz/msgs/Utility.hh>

// This header holds serialization operators which are shared among several
// components

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace traits
{
  /// \brief Type trait that determines if a gz::sim::convert from In
  /// to Out is defined.
  /// Usage:
  /// \code
  ///    constexpr bool hasGazeboConvert =
  ///       HasGazeboConvert<math::Pose, msgs::Pose>::value
  /// \endcode
  template <typename In, typename Out>
  class HasGazeboConvert
  {
    private: template <typename InArg, typename OutArg>
    static auto Test(int _test)
        -> decltype(std::declval<OutArg>() =
           gz::sim::convert<OutArg>(std::declval<const InArg &>()),
           std::true_type());

    private: template <typename, typename>
    static auto Test(...) -> std::false_type;

    public: static constexpr bool value =  // NOLINT
                decltype(Test<In, Out>(true))::value;
  };
}
/// \brief A Serializer class is used to serialize and deserialize a component.
/// It is passed in as the third template parameter to components::Component.
/// Eg.
/// \code
///   using Geometry = components::Component<sdf::Geometry, GeometryTag,
///         serializers::GeometrySerializer>
/// \endcode
/// A serializer class implements two static functions: `Serialize` and
/// `Deserialize` with the following signatures
/// \code
///     class ExampleSerializer
///     {
///       public: static std::ostream &Serialize(std::ostream &_out,
///                                              const DataType &_data);
///       public: static std::istream &Deserialize(std::istream &_in,
///                                                DataType &_data)
///     };
/// \endcode

namespace serializers
{
  /// \brief Serialization for that converts components data types to
  /// gz::msgs. This assumes that gz::sim::convert<DataType> is
  /// defined
  /// \tparam DataType Underlying data type of the component
  ///
  /// This can be used for components that can be converted to gz::msg
  /// types via gz::sim::convert. For example sdf::Geometry can be
  /// converted to msgs::Geometry so the component can be defined as
  /// \code
  ///   using Geometry = Component<sdf::Geometry, class GeometryTag,
  ///           ComponentToMsgSerializer<sdf::Geometry, msgs::Geometry>>
  /// \endcode
  template <typename DataType, typename MsgType>
  class ComponentToMsgSerializer
  {
    /// \brief Serialization
    /// \param[in] _out Output stream.
    /// \param[in] _data data to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const DataType &_data)
    {
      MsgType msg;
      // cppcheck-suppress syntaxError
      if constexpr (traits::HasGazeboConvert<DataType, MsgType>::value)
      {
        msg = gz::sim::convert<MsgType>(_data);
      }
      else
      {
        msg = gz::msgs::Convert(_data);
      }

      msg.SerializeToOstream(&_out);
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in Input stream.
    /// \param[out] _data data to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                                             DataType &_data)
    {
      MsgType msg;
      msg.ParseFromIstream(&_in);

      if constexpr (traits::HasGazeboConvert<MsgType, DataType>::value)
      {
        _data = gz::sim::convert<DataType>(msg);
      }
      else
      {
        _data = gz::msgs::Convert(msg);
      }
      return _in;
    }
  };

  /// \brief Common serializer for sensors
  using SensorSerializer = ComponentToMsgSerializer<sdf::Sensor, msgs::Sensor>;

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
      msg.SerializeToOstream(&_out);
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
      msg.ParseFromIstream(&_in);

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
      _msg.SerializeToOstream(&_out);
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in Input stream.
    /// \param[in] _msg Message to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
        google::protobuf::Message &_msg)
    {
      _msg.ParseFromIstream(&_in);
      return _in;
    }
  };

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
