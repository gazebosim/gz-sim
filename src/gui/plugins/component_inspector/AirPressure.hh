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
#ifndef IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_AIRPRESSURE_HH_
#define IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_AIRPRESSURE_HH_

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Entity.hh>
#include <sdf/AirPressure.hh>

#include "Types.hh"

namespace ignition
{
namespace gazebo
{
  /// \brief Specialized to set altimetere data.
  /// This function is called from ComponentInspector::Update. It is used to
  /// insert the Altimeter QML components into the component inspector.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
    void ignition::gazebo::setData(QStandardItem *_item,
        const sdf::AirPressure &_altimeter);

  /// \brief This function is called when a user changes values in the altimeter
  /// sensor.
  /// \param[in] _mean Mean value
  /// \param[in] _meanBias Bias mean value
  /// \param[in] _stdDev Standard deviation value
  /// \param[in] _stdDevBias Bias standard deviation value
  /// \param[in] _dynamicBiasStdDev Dynamic bias standard deviation value
  /// \param[in] _dynamicBiasCorrelationTime Dynamic bias correlation time value
  ignition::gazebo::UpdateCallback onAirPressureNoise(Entity _entity,
      double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime);

  ignition::gazebo::UpdateCallback onAirPressureReferenceAltitude(
      Entity _entity, double _referenceAltitude);
}
}
#endif
