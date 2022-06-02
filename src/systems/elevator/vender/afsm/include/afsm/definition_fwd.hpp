/*
 * definition_fwd.hpp
 *
 *  Created on: 1 июня 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DEFINITION_FWD_HPP_
#define AFSM_DEFINITION_FWD_HPP_

namespace afsm {
namespace def {

template < typename T, typename ... Tags >
struct state_def;
template < typename StateType, typename ... Tags >
using state = state_def< StateType, Tags... >;

template < typename T, typename ... Tags  >
struct terminal_state;

template < typename T, typename Machine, typename ... Tags >
struct pushdown;
template < typename T, typename Machine, typename ... Tags >
struct popup;

template < typename T, typename ... Tags >
struct state_machine_def;
template < typename T, typename ... Tags >
using state_machine = state_machine_def<T, Tags...>;

template < typename SourceState, typename Event, typename TargetState,
        typename Action = none, typename Guard = none >
struct transition;
template < typename Event, typename Action = none, typename Guard = none >
struct internal_transition;
template < typename ... T >
struct transition_table;

}  /* namespace def */
namespace detail {

template <typename T>
struct pushdown_state;

template <typename T>
struct popup_state;
}  /* namespace detail */
}  /* namespace afsm */

#endif /* AFSM_DEFINITION_FWD_HPP_ */
