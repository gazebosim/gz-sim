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

#ifndef GZ_SIM_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_
#define GZ_SIM_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_

#include <memory>

#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>

#include "afsm/fsm.hpp"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
// Data forward declarations
class ElevatorCommonPrivate;
class ElevatorStateMachinePrivate;

// Event forward declarations
namespace events
{
  struct EnqueueNewTarget;
  struct NewTarget;
  struct DoorOpen;
  struct DoorClosed;
  struct Timeout;
  struct CabinAtTarget;
}  // namespace events

// Action forward declarations
namespace actions
{
  template <bool>
  struct EnqueueNewTarget;
  struct NewTarget;
  struct CabinAtTarget;
}  // namespace actions

// Guard forward declarations
namespace guards
{
  template <typename TargetState>
  struct IsInState;
  struct CabinAtTarget;
  struct NoQueuedTarget;
}  // namespace guards

/// \brief Elevator state machine frontend. Defines the transition table and
/// initial state of the state machine.
class ElevatorStateMachineDef
    : public ::afsm::def::state_machine<ElevatorStateMachineDef>
{
  // State forward declarations
  struct IdleState;
  template <typename E>
  struct DoorState;
  struct OpenDoorState;
  struct CloseDoorState;
  struct WaitState;
  struct MoveCabinState;

  /// \brief Constructor
  /// \param[in] _system Data that are common to both the system and the state
  /// machine.
  public: ElevatorStateMachineDef(
      const std::shared_ptr<ElevatorCommonPrivate> &_system);

  /// \brief Destructor
  public: ~ElevatorStateMachineDef();

  /// \brief Initial state of the state machine
  public: using initial_state = IdleState;

  /// \brief Transition transition table
  public: using internal_transitions = transition_table <
            in<events::EnqueueNewTarget, actions::EnqueueNewTarget<true>,
               guards::IsInState<IdleState> >,
            in<events::EnqueueNewTarget, actions::EnqueueNewTarget<false>,
               guards::IsInState<CloseDoorState> >
   >;

  /// \brief Transition table
  public: using transitions = transition_table<
            // +--------------------------------------------------------------+
            tr<IdleState, events::NewTarget, OpenDoorState, actions::NewTarget,
               guards::CabinAtTarget>,
            tr<IdleState, events::NewTarget, MoveCabinState, actions::NewTarget,
               not_<guards::CabinAtTarget> >,
            // +--------------------------------------------------------------+
            tr<OpenDoorState, events::DoorOpen, WaitState, none, none>,
            // +--------------------------------------------------------------+
            tr<WaitState, events::Timeout, CloseDoorState, none, none>,
            // +--------------------------------------------------------------+
            tr<CloseDoorState, events::DoorClosed, IdleState, none,
               guards::NoQueuedTarget>,
            tr<CloseDoorState, events::DoorClosed, MoveCabinState, none,
               not_<guards::NoQueuedTarget> >,
            // +--------------------------------------------------------------+
            tr<MoveCabinState, events::CabinAtTarget, OpenDoorState,
               actions::CabinAtTarget, none>
   >;

  /// \brief Public data pointer
  public: std::unique_ptr<ElevatorStateMachinePrivate> dataPtr;
};

/// \brief Elevator state machine backend
using ElevatorStateMachine = ::afsm::state_machine<ElevatorStateMachineDef>;

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#include "state_machine/ElevatorStateMachineImpl.hh"

#endif  // GZ_SIM_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_
