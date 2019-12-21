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
#ifndef IGNITION_GAZEBO_GUI_GUIEVENTS_HH_
#define IGNITION_GAZEBO_GUI_GUIEVENTS_HH_

#include <QEvent>
#include <ignition/math/Vector3.hh>
#include "ignition/gazebo/config.hh"

namespace ignition
{
namespace gazebo
{
namespace gui {
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
/// \brief Namespace for all events. Refer to the EventManager class for
/// more information about events.
namespace events
{
  static const QEvent::Type SnapEvent = QEvent::Type(1000);

  class SnapIntervals : public QEvent
  {
    public: SnapIntervals(math::Vector3d _xyz, math::Vector3d _rpy)
    : QEvent(SnapEvent), xyz(_xyz), rpy(_rpy)
    {
    }
    public: math::Vector3d XYZ() const
    {
      return this->xyz;
    }
    public: math::Vector3d RPY() const
    {
      return this->rpy;
    }
    private: math::Vector3d xyz;
    private: math::Vector3d rpy;
  };
}  // namespace events
}
}  // namespace gui
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_GUI_GUIEVENTS_HH_
