/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <QStandardItem>

#include <gz/sim/Types.hh>

namespace gz
{
namespace sim
{
namespace inspector
{
  /// \brief Function definition that a component can use
  /// to update its UI elements based on changes from the ECM.
  ///   * _ecm Immutable reference to the ECM
  ///   * _item Item to be updated
  /// \sa ComponentInspector::AddUpdateViewCb
  using UpdateViewCb = std::function<void(const EntityComponentManager &_ecm,
      QStandardItem *_item)>;
}
}
}
#endif
