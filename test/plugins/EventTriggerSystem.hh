/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_TEST_EVENTTRIGGERSYSTEM_HH_
#define IGNITION_GAZEBO_TEST_EVENTTRIGGERSYSTEM_HH_

#include <ignition/common/Console.hh>
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
class IGNITION_GAZEBO_VISIBLE EventTriggerSystem :
  public gazebo::System,
  public gazebo::ISystemConfigure,
  public gazebo::ISystemUpdate
{
  // needed for linter
  public: EventTriggerSystem() = default;
  public: ~EventTriggerSystem() = default;

  public: void Configure(const Entity &/*_entity*/,
                         const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                         EntityComponentManager &/*_ecm*/,
                         EventManager &_eventManager) override
        {
          igndbg << "Configure" << std::endl;
          this->eventManager = &_eventManager;
        }

  public: void Update(const UpdateInfo &/*_info*/,
                      EntityComponentManager &/*_ecm*/) override
        {
          igndbg << "Update" << std::endl;
          this->eventManager->Emit<events::Pause>(false);
        }

  private: EventManager *eventManager;
};
}
}

#endif
