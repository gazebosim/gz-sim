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

#ifndef GZ_SIM_GUI_COMPONENTINSPECTOR_TYPES_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOR_TYPES_HH_

#include <map>
#include <QStandardItem>

#include <sdf/Noise.hh>

#include <gz/sim/Types.hh>

namespace gz
{
namespace sim
{
  /// \brief UpdateCallback is a function defition that is used by a
  /// component to manage ECM changes.
  /// \sa void ComponentInspectorEditor::AddUpdateCallback(UpdateCallback _cb)
  using UpdateCallback = std::function<void(EntityComponentManager &)>;

  /// ComponentCreator is a function definition that a component can use
  /// to create the appropriate UI elements for an Entity based on
  /// a ComponentTypeId.
  /// \sa void ComponentInspectorEditor::RegisterComponentCreator(
  /// UpdateCallback _cb)
  using ComponentCreator = std::function<void(EntityComponentManager &,
      Entity, QStandardItem *)>;

  /// \brief Helper function that will set all noise properties.
  /// \param[out] _noise Noise to set
  /// \param[in] _mean Mean value
  /// \param[in] _meanBias Bias mean value
  /// \param[in] _stdDev Standard deviation value
  /// \param[in] _stdDevBias Bias standard deviation value
  /// \param[in] _dynamicBiasStdDev Dynamic bias standard deviation value
  /// \param[in] _dynamicBiasCorrelationTime Dynamic bias correlation time
  /// value
  void setNoise(sdf::Noise &_noise,
      double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime);
}
}
#endif
