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
#ifndef IGNITION_GAZEBO_COMPONENTS_SERIALIZATION_HH_
#define IGNITION_GAZEBO_COMPONENTS_SERIALIZATION_HH_

#include <ignition/msgs/double_v.pb.h>

#include <vector>
#include <sdf/Sensor.hh>

#include <ignition/gazebo/Conversions.hh>

// This header holds serialization operators which are shared among several
// components

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace serializers
{
  /// \brief Serialization for that converts components data types to
  /// ignition::msgs. This assumes that ignition::gazebo::convert<DataType> is
  /// defined
  /// \tparam DataType Underlying data type of the component
  ///
  /// This can be used for components that can be converted to ignition::msg
  /// types via ignition::gazebo::convert. For example sdf::Geometry can be
  /// converted to msgs::Geometry so the component can be defined as
  /// \code
  ///   using Geometry = Component<sdf::Geometry, class GeometryTag,
  ///           ComponentToMsgSerializer<sdf::Geometry, msgs::Geometry>>
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
      auto msg = ignition::gazebo::convert<MsgType>(_data);
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

      _data = ignition::gazebo::convert<DataType>(msg);
      return _in;
    }
  };

  /// \brief Common serializer for sensors
  using SensorSerializer =
      serializers::ComponentToMsgSerializer<sdf::Sensor, msgs::Sensor>;

  /// \brief Serializer for components that hold `std::vector<double>`.
  class VectorDoubleSerializer
  {
    /// \brief Serialization
    /// \param[in] _out Output stream.
    /// \param[in] _geometry Geometry to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const std::vector<double> &_vec)
    {
      ignition::msgs::Double_V msg;
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
      ignition::msgs::Double_V msg;
      msg.ParseFromIstream(&_in);

      _vec = {msg.data().begin(), msg.data().end()};
      return _in;
    }
  };
}
}
}
}

#endif
