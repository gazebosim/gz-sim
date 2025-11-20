/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_LOCKSTEP_HH_
#define GZ_SIM_SYSTEMS_LOCKSTEP_HH_

#include <gz/msgs/serialized_map.pb.h>
#include <memory>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief ToDo.
  class LockStep : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate,
                   public ISystemUpdate,
                   public ISystemPostUpdate,
                   public ISystemReset
  {
    /// \brief Constructor
    public: LockStep() = default;

    /// \brief Destructor
    public: ~LockStep() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_element,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventManager) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    /// \brief A transport node.
    private: gz::transport::Node node;

    /// \brief Filled on demand for the state service.
    private: msgs::SerializedStepMap stepMsg;

    /// \brief True when plugin configured.
    private: bool configured = false;

    /// \brief Whether pre-update should be called.
    private: std::optional<bool> shouldCallPreUpdate = std::nullopt;

    /// \brief Whether update should be called.
    private: std::optional<bool> shouldCallUpdate = std::nullopt;

    /// \brief Whether post-update should be called.
    private: std::optional<bool> shouldCallPostUpdate = std::nullopt;
  };
}
}
}
}

#endif
