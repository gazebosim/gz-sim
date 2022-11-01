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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */
#ifndef GZ_SIM_SYSTEMS_ACOUSTICCOMMS_HH_
#define GZ_SIM_SYSTEMS_ACOUSTICCOMMS_HH_

#include <memory>

#include <sdf/sdf.hh>
#include "gz/sim/comms/ICommsModel.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A comms model that simulates communication using acoustic
  /// devices. The model uses simple distance based acoustics model.
  ///
  /// This system can be configured with the following SDF parameters:
  ///
  /// * Optional parameters:
  ///    * <max_range>: Hard limit on range (meters). No communication will
  ///                   happen beyond this range. Default is 1000.
  ///    * <speed_of_sound>: Speed of sound in the medium (meters/sec).
  ///                         Default is 343.0
  ///
  /// Here's an example:
  ///  <plugin
  ///    filename="gz-sim-acoustic-comms-system"
  ///    name="gz::sim::systems::AcousticComms">
  ///    <max_range>6</max_range>
  ///    <speed_of_sound>1400</speed_of_sound>
  ///  </plugin>

  class AcousticComms:
    public gz::sim::comms::ICommsModel
  {
    public: explicit AcousticComms();

    // Documentation inherited.
    public: void Load(const gz::sim::Entity &_entity,
                        std::shared_ptr<const sdf::Element> _sdf,
                        gz::sim::EntityComponentManager &_ecm,
                        gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void Step(const gz::sim::UpdateInfo &_info,
                        const gz::sim::comms::Registry &_currentRegistry,
                        gz::sim::comms::Registry &_newRegistry,
                        gz::sim::EntityComponentManager &_ecm) override;

    // Impl pointer
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}

#endif
