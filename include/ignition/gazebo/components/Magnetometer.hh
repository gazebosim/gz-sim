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
#ifndef IGNITION_GAZEBO_COMPONENTS_MAGNETOMETER_HH_
#define IGNITION_GAZEBO_COMPONENTS_MAGNETOMETER_HH_

#include <ignition/msgs/sensor.pb.h>
#include <sdf/Sensor.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/Conversions.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains a magnetometer sensor,
  /// sdf::Magnetometer, information.
  using MagnetometerBase = Component<sdf::Sensor, class MagnetometerTag>;

  /// \brief This component holds a magnetometer sensor.
  class Magnetometer : public MagnetometerBase
  {
    // Documentation inherited
    public: Magnetometer() : MagnetometerBase()
    {
    }

    // Documentation inherited
    public: explicit Magnetometer(const sdf::Sensor &_data)
      : MagnetometerBase(_data)
    {
    }

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override
    {
      auto msg = convert<msgs::Sensor>(this->Data());
      msg.SerializeToOstream(&_out);
    }

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override
    {
      msgs::Sensor msg;
      msg.ParseFromIstream(&_in);

      this->Data() = convert<sdf::Sensor>(msg);
    }
  };

  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.Magnetometer", Magnetometer)
}
}
}
}
#endif
