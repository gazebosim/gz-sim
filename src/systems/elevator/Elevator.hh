/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
 * \author Nick Lamprianidis <nlamprian@gmail.com>
 * \date January 2021
 */

#ifndef GZ_SIM_SYSTEMS_ELEVATOR_HH_
#define GZ_SIM_SYSTEMS_ELEVATOR_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
// Data forward declaration
class ElevatorPrivate;

/// \brief System that controls an elevator. It closely models the structure
/// and functionality of a real elevator. It individually controls the cabin
/// and doors of the elevator, queues commands for the elevator and stops at
/// intermediate floors if a command is received while the last one is ongoing,
/// and keeps a door open if the doorway is blocked. The model of the elevator
/// can have arbitrarily many floor levels and at arbitrary heights each.
///
/// ## Assumptions on the Elevator Model
///
/// In the default configuration, the cabin is at the ground floor and the
/// doors are closed
///
/// In the default configuration, the cabin and door joints are at zero position
///
/// There are reference floor links along the cabin joint that indicate the
/// cabin joint positions for each floor. The names of links follow the pattern
/// indicated by `<floor_link_prefix>`
///
/// There is a door in each floor and the names of the doors follow the pattern
/// indicated by `<door_joint_prefix>`
///
/// Each cabin and door joint has an associated joint position controller
/// system that listens for command to
/// `/model/{model_name}/joint/{joint_name}/0/cmd_pos`
///
/// Each door (optionally) has a lidar that, if intercepted, indicates that the
/// doorway is blocked. The lidar publishes sensor data on topic
/// `/model/{model_name}/{door_joint_name}/lidar`
///
/// ## System Parameters
///
/// - `<update_rate>`: System update rate. This element is optional and the
/// default value is 10Hz. A value of zero gets translated to the simulation
/// rate (no throttling for the system).
///
/// - `<floor_link_prefix>`: Prefix in the names of the links that function as
/// a reference for each floor level. When the elevator is requested to move
/// to a given floor level, the cabin is commanded to move to the height of
/// the corresponding floor link. The names of the links will be expected to
/// be `{prerix}i`, where \f$i=[0,N)\f$ and N is the number of floors. This
/// element is optional and the default value is `floor_`.
///
/// - `<door_joint_prefix>`: Prefix in the names of the joints that control the
/// doors of the elevator. The names of the joints will be expected to be
/// `{prerix}i`, where \f$i=[0,N)\f$ and N is the number of floors. This
/// element is optional and the default value is `door_`.
///
/// - `<cabin_joint>`: Name of the joint that controls the position of the
/// cabin. This element is optional and the default value is `lift`.
///
/// - `<cmd_topic>`: Topic to which this system will subscribe in order to
/// receive command messages. This element is optional and the default value
/// is `/model/{model_name}/cmd`.
///
/// - `<state_topic>`: Topic on which this system will publish state (current
/// floor) messages. This element is optional and the default value is
/// `/model/{model_name}/state`.
///
/// - `<state_publish_rate>`: State publication rate. This rate is bounded by
/// `<update_rate>`. This element is optional and the default value is 5Hz.
///
/// - `<open_door_wait_duration>`: Time to wait with a door open before the door
/// closes. This element is optional and the default value is 5 sec.
class GZ_SIM_VISIBLE Elevator : public System,
                                         public ISystemConfigure,
                                         public ISystemPostUpdate
{
  /// \brief Constructor
  public: Elevator();

  /// \brief Destructor
  public: ~Elevator() override = default;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override;

  /// \brief Private data pointer
  private: std::shared_ptr<ElevatorPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_ELEVATOR_HH_
