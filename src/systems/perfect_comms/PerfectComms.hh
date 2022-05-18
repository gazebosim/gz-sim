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
#ifndef GZ_GAZEBO_SYSTEMS_PERFECTCOMMS_HH_
#define GZ_GAZEBO_SYSTEMS_PERFECTCOMMS_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>
#include "gz/sim/comms/ICommsModel.hh"
#include "gz/sim/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class MsgManager;

  /// \brief An example of a comms model.
  /// This model always delivers any message to its destination.
  class PerfectComms
        : public comms::ICommsModel
  {
    /// \brief Constructor.
    public: explicit PerfectComms();

    // Documentation inherited.
    public: void Load(const Entity &_entity,
                      std::shared_ptr<const sdf::Element> _sdf,
                      EntityComponentManager &_ecm,
                      EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void Step(const ignition::gazebo::UpdateInfo &_info,
                      const comms::Registry &_currentRegistry,
                      comms::Registry &_newRegistry,
                      EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
}
}

#endif
