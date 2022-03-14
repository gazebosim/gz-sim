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

#ifndef IGNITION_GAZEBO_COMMSMODEL_HH__
#define IGNITION_GAZEBO_COMMSMODEL_HH__

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/MsgManager.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Abstract interface to define how the environment should handle
  /// communication simulation. This class should be responsible for
  /// handling dropouts, decay and packet collisions.
  class ICommsModel
  {
    /// \brief This method is called when there is a timestep in the simulator
    /// override this to update your data structures as needed.
    /// \param[in] _info - Simulator information about the current timestep.
    /// \param[in] _ecm - Ignition's ECM.
    /// \param[in] _messageMgr - Use this to mark the message as arrived.
    public: virtual void Step(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::MsgManager &_messageMgr) = 0;

    /// \brief Destructor
    public: virtual ~ICommsModel() = default;
  };
}
}
}

#endif
