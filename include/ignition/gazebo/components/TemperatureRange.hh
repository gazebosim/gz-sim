/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_TEMPERATURERANGE_HH_
#define IGNITION_GAZEBO_COMPONENTS_TEMPERATURERANGE_HH_

#include <istream>
#include <ostream>

#include <ignition/math/Temperature.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief Data structure to hold a temperature range, in kelvin
  struct TemperatureRangeInfo
  {
    /// \brief The minimum temperature (kelvin)
    math::Temperature min;
    /// \brief The maximum temperature (kelvin)
    math::Temperature max;

    public: bool operator==(const TemperatureRangeInfo &_info) const
    {
      return (this->min == _info.min) &&
             (this->max == _info.max);
    }

    public: bool operator!=(const TemperatureRangeInfo &_info) const
    {
      return !(*this == _info);
    }
  };
}

namespace serializers
{
  /// \brief Serializer for components::TemperatureRangeInfo object
  class TemperatureRangeInfoSerializer
  {
    /// \brief Serialization for components::TemperatureRangeInfo
    /// \param[out] _out Output stream
    /// \param[in] _info Object for the stream
    /// \return The stream
    public: static std::ostream &Serialize(std::ostream &_out,
                const components::TemperatureRangeInfo &_info)
    {
      _out << _info.min << " " << _info.max;
      return _out;
    }

    /// \brief Deserialization for components::TemperatureRangeInfo
    /// \param[in] _in Input stream
    /// \param[out] _info The object to populate
    /// \return The stream
    public: static std::istream &Deserialize(std::istream &_in,
                components::TemperatureRangeInfo &_info)
    {
      _in >> _info.min >> _info.max;
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that stores a temperature range in kelvin
  using TemperatureRange = Component<TemperatureRangeInfo, class TemperatureRangeTag,
    serializers::TemperatureRangeInfoSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.TemperatureRange",
      TemperatureRange)
}
}
}
}

#endif
