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

#include <set>
#include <QEvent>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/config.hh"

namespace ignition
{
namespace gazebo
{
namespace gui {
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace events
{
  class EntitiesSelected : public QEvent
  {
    public: EntitiesSelected(std::set<Entity> _entities, bool _fromUser = false)
        : QEvent(Type), entities(_entities), fromUser(_fromUser)
    {
    }
    public: std::set<Entity> Data() const
    {
      return this->entities;
    }
    public: bool FromUser() const
    {
      return this->fromUser;
    }
    static const QEvent::Type Type = QEvent::Type(QEvent::User + 1);
    private: std::set<Entity> entities;
    private: bool fromUser = false;
  };
}  // namespace events
}
}  // namespace gui
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_GUI_GUIEVENTS_HH_
