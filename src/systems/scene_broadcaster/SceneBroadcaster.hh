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
#ifndef GZ_SIM_SCENEBROADCASTER_SYSTEM_HH_
#define GZ_SIM_SCENEBROADCASTER_SYSTEM_HH_

#include <memory>
#include <vector>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class SceneBroadcasterPrivate;

  /** \class SceneBroadcaster SceneBroadcaster.hh \
   * gz/sim/systems/SceneBroadcaster.hh
  **/
  /// \brief System which periodically publishes a gz::msgs::Scene
  /// message with updated information.
  class SceneBroadcaster final:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate,
    public ISystemReset
  {
    /// \brief Constructor
    public: SceneBroadcaster();

    /// \brief Destructor
    public: ~SceneBroadcaster() final = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<SceneBroadcasterPrivate> dataPtr;
  };
}
}
}
}
#endif
