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

#include <memory>
#include <string>

#include "../ElevatorStateMachine.hh"

#include <gz/common/Console.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
/// \brief State at which the elevator is idling.
struct ElevatorStateMachineDef::IdleState : state<IdleState>
{
  /// \brief Logs the entry to the state
  public: template <typename Event, typename FSM>
  void on_enter(const Event &, FSM &)
  {
    gzmsg << "The elevator is idling" << std::endl;
  }
};

/// \brief Virtual state at which the elevator is opening or closing a door.
template <typename E>
struct ElevatorStateMachineDef::DoorState : state<DoorState<E>>
{
  /// \brief Constructor
  /// \param[in] _posEps Position tolerance when checking if a door has opened
  /// or closed
  /// \param[in] _velEps Velocity tolerance when checking if a door has opened
  /// or closed
  public: DoorState(double _posEps = 2e-2, double _velEps = 1e-2)
      : posEps(_posEps), velEps(_velEps)
  {
  }

  /// \brief Destructor
  public: virtual ~DoorState()
  {
  }

  /// \brief Sends the opening or closing command to the door joint of the
  /// target floor and configures the monitor that checks the joint state
  /// \param[in] _fsm State machine to which the state belongs
  public: template <typename Event, typename FSM>
  void on_enter(const Event &, FSM &_fsm)
  {
    const auto &data = _fsm.dataPtr;
    int32_t floorTarget = data->system->state;
    gzmsg << "The elevator is " << this->Report(data) << std::endl;

    double jointTarget = this->JointTarget(data, floorTarget);
    data->SendCmd(data->system->doorJointCmdPubs[floorTarget], jointTarget);
    this->triggerEvent = [&_fsm] { _fsm.process_event(E()); };
    data->system->SetDoorMonitor(
        floorTarget, jointTarget, this->posEps, this->velEps,
        std::bind(&DoorState::OnJointTargetReached, this));
  }

  /// \brief Gets the message that's being reported when entering the state
  /// \param[in] _data State machine data
  /// \return String with the message the state has to report
  private: virtual std::string Report(
      const std::unique_ptr<ElevatorStateMachinePrivate> &_data) const = 0;

  /// \brief Gets the joint position for opening or closing a door
  /// \param[in] _data State machine data
  /// \param[in] _floorTarget Target floor level
  /// \return Joint position
  private: virtual double JointTarget(
      const std::unique_ptr<ElevatorStateMachinePrivate> &_data,
      int32_t _floorTarget) const = 0;

  /// \brief Method that gets called when a door joint reaches its target
  private: void OnJointTargetReached()
  {
    this->triggerEvent();
  }

  /// \brief Positiion tolerance
  private: double posEps;

  /// \brief Velocity tolerance
  private: double velEps;

  /// \brief Triggers the exit event
  private: std::function<void()> triggerEvent;
};

/// \brief State at which the elevator is opening a door.
struct ElevatorStateMachineDef::OpenDoorState : DoorState<events::DoorOpen>
{
  /// \brief This state defers EnqueueNewTargetEvent events
  public: using deferred_events = type_tuple<events::EnqueueNewTarget>;

  /// \brief Constructor
  public: OpenDoorState() = default;

  // Documentation inherited
  private: virtual std::string Report(
      const std::unique_ptr<ElevatorStateMachinePrivate> &_data) const override
  {
    return "opening door " + std::to_string(_data->system->state);
  }

  // Documentation inherited
  private: virtual double JointTarget(
      const std::unique_ptr<ElevatorStateMachinePrivate> &_data,
      int32_t _floorTarget) const override
  {
    return _data->system->doorTargets[_floorTarget];
  }
};

/// \brief State at which the elevator is closing a door.
struct ElevatorStateMachineDef::CloseDoorState : DoorState<events::DoorClosed>
{
  /// \brief Constructor
  public: CloseDoorState() = default;

  // Documentation inherited
  private: virtual std::string Report(
      const std::unique_ptr<ElevatorStateMachinePrivate> &_data) const override
  {
    return "closing door " + std::to_string(_data->system->state);
  }

  // Documentation inherited
  private: virtual double JointTarget(
      const std::unique_ptr<ElevatorStateMachinePrivate> & /*_data*/,
      int32_t /*_floorTarget*/) const override
  {
    return 0.0;
  }
};

/// \brief State at which the elevator is waiting with a door open.
struct ElevatorStateMachineDef::WaitState : state<WaitState>
{
  /// \brief This state defers EnqueueNewTargetEvent events
  public: using deferred_events = type_tuple<events::EnqueueNewTarget>;

  /// \brief Starts the timer that keeps the door at the target floor level open
  /// \param[in] _fsm State machine to which the state belongs
  public: template <typename Event, typename FSM>
  void on_enter(const Event &, FSM &_fsm)
  {
    const auto &data = _fsm.dataPtr;
    gzmsg << "The elevator is waiting to close door " << data->system->state
           << std::endl;

    this->triggerEvent = [&_fsm] { _fsm.process_event(events::Timeout()); };
    data->system->StartDoorTimer(data->system->state,
                                 std::bind(&WaitState::OnTimeout, this));
  }

  /// \brief Method that gets called upon timeout
  private: void OnTimeout()
  {
    this->triggerEvent();
  }

  /// \brief Triggers the exit event
  private: std::function<void()> triggerEvent;
};

/// \brief State at which the elevator is moving the cabin to the target floor.
struct ElevatorStateMachineDef::MoveCabinState : state<MoveCabinState>
{
  /// \brief This state defers EnqueueNewTargetEvent events
  public: using deferred_events = type_tuple<events::EnqueueNewTarget>;

  /// \brief Constructor
  /// \param[in] _posEps Position tolerance when checking if the cabin has
  /// reached the target floor level
  /// \param[in] _velEps Velocity tolerance when checking if the cabin has
  /// reached the target floor level
  public: MoveCabinState(double _posEps = 15e-2, double _velEps = 1e-2)
      : posEps(_posEps), velEps(_velEps)
  {
  }

  /// \brief Sends the command that moves the cabin joint to the target floor
  /// and configures the monitor that checks the joint state
  /// \param[in] _fsm State machine to which the state belongs
  public: template <typename Event, typename FSM>
  void on_enter(const Event &, FSM &_fsm)
  {
    const auto &data = _fsm.dataPtr;
    int32_t floorTarget = data->targets.front();
    gzmsg << "The elevator is moving the cabin [ " << data->system->state
           << " -> " << floorTarget << " ]" << std::endl;

    double jointTarget = data->system->cabinTargets[floorTarget];
    data->SendCmd(data->system->cabinJointCmdPub, jointTarget);
    this->triggerEvent = [&_fsm] {
      _fsm.process_event(events::CabinAtTarget());
    };
    data->system->SetCabinMonitor(
        floorTarget, jointTarget, this->posEps, this->velEps,
        std::bind(&MoveCabinState::OnJointTargetReached, this));
  }

  /// \brief Method that gets called when the cabin joint reaches its target
  private: void OnJointTargetReached()
  {
    this->triggerEvent();
  }

  /// \brief Positiion tolerance
  private: double posEps;

  /// \brief Velocity tolerance
  private: double velEps;

  /// \brief Triggers the exit event
  private: std::function<void()> triggerEvent;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
