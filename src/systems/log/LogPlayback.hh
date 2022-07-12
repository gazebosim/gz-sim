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
#ifndef GZ_SIM_SYSTEMS_LOGPLAYBACK_HH_
#define GZ_SIM_SYSTEMS_LOGPLAYBACK_HH_

#include <memory>

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
  // Forward declarations.
  class LogPlaybackPrivate;

  /// \class LogPlayback LogPlayback.hh
  ///   gz/sim/systems/log/LogPlayback.hh
  /// \brief Log state playback
  class LogPlayback:
    public System,
    public ISystemConfigure,
    public ISystemReset,
    public ISystemUpdate
  {
    /// \brief Constructor
    public: explicit LogPlayback();

    /// \brief Destructor
    public: ~LogPlayback() override;

    /// Documentation inherited
    public: void Configure(const Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<LogPlaybackPrivate> dataPtr;
  };
  }
}
}
}
#endif
