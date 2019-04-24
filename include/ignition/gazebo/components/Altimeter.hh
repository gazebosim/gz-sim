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
#ifndef IGNITION_GAZEBO_COMPONENTS_ALTIMETER_HH_
#define IGNITION_GAZEBO_COMPONENTS_ALTIMETER_HH_

#include <ignition/msgs/sensor.pb.h>
#include <sdf/Sensor.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains an altimeter sensor,
  /// sdf::Altimeter, information.
  using AltimeterBase = Component<sdf::Sensor, class AltimeterTag>;

  /// \brief This component holds a magnetometer sensor.
  class Altimeter : public AltimeterBase
  {
    // Documentation inherited
    public: Altimeter() : AltimeterBase()
    {
    }

    // Documentation inherited
    public: explicit Altimeter(const sdf::Sensor &_data)
      : AltimeterBase(_data)
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
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Altimeter", Altimeter)
}
}
}
}

#endif
