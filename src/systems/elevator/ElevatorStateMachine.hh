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

#ifndef IGNITION_GAZEBO_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_
#define IGNITION_GAZEBO_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_

#include <memory>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <ignition/gazebo/Entity.hh>
#include <ignition/transport/Node.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
// Data forward declarations
class ElevatorCommonPrivate;
class ElevatorStateMachinePrivate;

// Event forward declarations
class EnqueueNewTargetEvent;
class NewTargetEvent;
class DoorOpenEvent;
class DoorClosedEvent;
class TimeoutEvent;
class CabinAtTargetEvent;

// State forward declarations
class IdleState;
class OpenDoorState;
class WaitState;
class CloseDoorState;
class MoveCabinState;

// Action forward declarations
template <bool>
class EnqueueNewTargetAction;
class NewTargetAction;
class CabinAtTargetAction;

// Guard forward declarations
template <bool>
class CabinAtTargetGuard;
template <bool>
class NoTargetGuard;

/// \brief Elevator state machine frontend. Defines the transition table and
/// initial state of the state machine.
class ElevatorStateMachine_
    : public boost::msm::front::state_machine_def<ElevatorStateMachine_>
{
  /// \brief Constructor
  public: ElevatorStateMachine_(
      const std::shared_ptr<ElevatorCommonPrivate> &_system);

  /// \brief Destructor
  public: ~ElevatorStateMachine_();

  /// \brief Gives access to the private data of the state machine. Used by the
  /// states
  /// \return Pointer to the private data of the state machine
  public: const std::unique_ptr<ElevatorStateMachinePrivate> &Data() const;

  /// \brief Initial state of the state machine
  public: using initial_state = boost::mpl::vector<IdleState>;

  /// \brief Alias to make transition table cleaner
  private: template <typename S, typename E, typename T, typename A, typename G>
  using Row = boost::msm::front::Row<S, E, T, A, G>;

  /// \brief Alias to make transition table cleaner
  private: using none = boost::msm::front::none;

  /// \brief The transition_table struct
  // clang-format off
  public: struct transition_table
      : boost::mpl::vector<
            //   Start            Event                   Next             Action                          Guard
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
            Row< IdleState      , EnqueueNewTargetEvent , none           , EnqueueNewTargetAction<true>  , none                      >,
            Row< IdleState      , NewTargetEvent        , OpenDoorState  , NewTargetAction               , CabinAtTargetGuard<false> >,
            Row< IdleState      , NewTargetEvent        , MoveCabinState , NewTargetAction               , CabinAtTargetGuard<true>  >,
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
            Row< OpenDoorState  , DoorOpenEvent         , WaitState      , none                          , none                      >,
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
            Row< WaitState      , TimeoutEvent          , CloseDoorState , none                          , none                      >,
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
            Row< CloseDoorState , EnqueueNewTargetEvent , none           , EnqueueNewTargetAction<false> , none                      >,
            Row< CloseDoorState , DoorClosedEvent       , IdleState      , none                          , NoTargetGuard<false>      >,
            Row< CloseDoorState , DoorClosedEvent       , MoveCabinState , none                          , NoTargetGuard<true>       >,
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
            Row< MoveCabinState , CabinAtTargetEvent    , OpenDoorState  , CabinAtTargetAction           , none                      > >
            // +----------------+-----------------------+----------------+-------------------------------+---------------------------+
  // clang-format on
  {
  };

  /// \brief Private data pointer
  private: std::unique_ptr<ElevatorStateMachinePrivate> dataPtr;
};

/// \brief Elevator state machine backend
using ElevatorStateMachine =
    boost::msm::back::state_machine<ElevatorStateMachine_>;

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#include "state_machine/ElevatorStateMachineImpl.hh"

#endif  // IGNITION_GAZEBO_SYSTEMS_ELEVATOR_STATE_MACHINE_HH_
