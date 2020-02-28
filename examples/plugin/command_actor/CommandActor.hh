/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef SYSTEM_PLUGIN_COMMANDACTOR_HH_
#define SYSTEM_PLUGIN_COMMANDACTOR_HH_

#include <chrono>
#include <ignition/gazebo/System.hh>

namespace command_actor
{

  /// \brief Example showing different ways of commanding an actor.
  class CommandActor:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: CommandActor();

    /// \brief Destructor
    public: ~CommandActor() override;

    /// Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Actor entity
    private: ignition::gazebo::Entity entity;

    // Origins to change, where the key is the number of seconds from beginning
    // of simulation, and the value is the new trajectory origin to set.
    private: std::map<int, ignition::math::Pose3d> origins;

    private: int lastOriginChange{0};
  };
}

#endif
