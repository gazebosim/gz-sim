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
#ifndef SYSTEM_PLUGIN_SAMPLESYSTEM_HH_
#define SYSTEM_PLUGIN_SAMPLESYSTEM_HH_

//! [header]
#include <gz/sim/System.hh>

namespace sample_system
{
  /// \brief Sample system that implemente the ISystemPostUpdate system
  /// plugin interface.
  class SampleSystem:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate
  {
    public: SampleSystem();

    public: ~SampleSystem() override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };

  class SampleSystem2:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
    public: SampleSystem2();

    public: ~SampleSystem2() override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    public: void Reset(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;
  };
}
//! [header]

#endif
