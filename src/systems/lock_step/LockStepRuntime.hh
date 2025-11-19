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
#ifndef GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_HH_
#define GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_HH_

#include <memory>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/sim/config.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemPluginPtr.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

// #include "lock_step.pb.h"
#include "RuntimeConfig.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

class LockStepRuntime {
public:
    /// \brief Create a LockStepRuntime from plugin and runtime config.
    /// Returns nullptr if
    /// - config is not valid
    /// - plugin could not be loaded
    /// - plugin does not implement any of the System interfaces
    static std::unique_ptr<LockStepRuntime> Create(
        RuntimeConfig _runtime_config);

    /// \brief Publish stats on provided stats topic in the config.
    bool PublishStats() const;

    ~LockStepRuntime();

protected:
    LockStepRuntime(SystemPluginPtr _systemPlugin, RuntimeConfig _config);

    /// \brief Called from `Create`.
    bool InitServices();

    /// \brief Callback for `Configure` service.
    bool OnConfigure(const gz::msgs::SerializedStepMap& _req,
                     gz::msgs::Boolean& _rep);

    /// \brief Callback for `PreUpdate` service.
    bool OnPreupdate(const gz::msgs::SerializedStepMap& _req,
                     gz::msgs::Boolean& _rep);

    // TODO: publish stats
    // /// \brief Stats proto
    // lock_step::RuntimeStats stats;

    /// \brief Gz transport node
    gz::transport::Node node;

    // TODO: publish stats
    // /// \brief Stats publisher
    // gz::transport::Node::Publisher publisher;

    /// \brief Last update info
    gz::msgs::WorldStatistics serverStats;

    /// \brief Plugin object. This manages the lifecycle of the instantiated
    /// class as well as the shared library.
    /// This will be null if the system wasn't loaded from a plugin.
    SystemPluginPtr systemPlugin;

    /// \brief Access the wrapped System via the ISystemConfigure interface
    /// Will be nullptr if the System doesn't implement this interface.
    ISystemConfigure *systemConfigure = nullptr;

    /// \brief Access the wrapped System via the ISystemPreUpdate interface
    /// Will be nullptr if the System doesn't implement this interface.
    ISystemPreUpdate *systemPreupdate = nullptr;

    /// \brief Access the wrapped System via the ISystemUpdate interface
    /// Will be nullptr if the System doesn't implement this interface.
    ISystemUpdate *systemUpdate = nullptr;

    /// \brief Access the wrapped System via the ISystemPostUpdate interface
    /// Will be nullptr if the System doesn't implement this interface.
    ISystemPostUpdate *systemPostupdate = nullptr;

    /// \brief Config data.
    const RuntimeConfig config;

    /// \brief ECM used in this runtime. Periodically updated by service calls
    EntityComponentManager ecm;
};

}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_HH_
