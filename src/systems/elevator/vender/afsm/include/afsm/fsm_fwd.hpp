/*
 * fsm_fwd.hpp
 *
 *  Created on: 28 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_FSM_FWD_HPP_
#define AFSM_FSM_FWD_HPP_

namespace afsm {

struct none {};

namespace detail {
template < typename Observer >
class observer_wrapper;

}

template < typename FSM, typename T >
class state;
template < typename FSM, typename T >
class inner_state_machine;
template < typename T, typename Mutex = none, typename Observer = none,
        template<typename> class ObserverWrapper = detail::observer_wrapper >
class state_machine;
template < typename T, typename Mutex = none, typename Observer = none,
        template<typename> class ObserverWrapper = detail::observer_wrapper >
class priority_state_machine;

}  /* namespace afsm */

#endif /* AFSM_FSM_FWD_HPP_ */
