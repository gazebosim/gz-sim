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

#include <gz/msgs/sensor.pb.h>

#include <ostream>
#include <istream>
#include <type_traits>
#include <utility>

#include <sdf/Sensor.hh>

#include <gz/msgs/Utility.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/components/BasicSerializers.hh>
#include <gz/sim/components/MsgSerializers.hh>

// This header holds serialization operators which are shared among several
// components. The serializers that do not need Conversions.hh are in
// BasicSerializers.hh and MsgSerializers.hh, which are included here.

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

      auto result = msg.SerializeToOstream(&_out);
      (void)result;
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
      if (!msg.ParseFromIstream(&_in))
      {
        return _in;
      }

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
}
}
}
}

#endif
