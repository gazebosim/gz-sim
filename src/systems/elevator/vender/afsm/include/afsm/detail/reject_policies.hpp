/*
 * reject_policies.hpp
 *
 *  Created on: Nov 30, 2016
 *      Author: zmij
 */

#ifndef AFSM_DETAIL_REJECT_POLICIES_HPP_
#define AFSM_DETAIL_REJECT_POLICIES_HPP_

#include <exception>
#include <pushkin/util/demangle.hpp>
#include <afsm/detail/actions.hpp>

namespace afsm {
namespace def {
namespace tags {

struct reject_throw_event {
    template < typename Event, typename FSM >
    actions::event_process_result
    reject_event(Event&& event, FSM&)
    {
        throw event;
    }
};

struct reject_throw {
    template < typename Event, typename FSM >
    actions::event_process_result
    reject_event(Event&&, FSM&)
    {
        using ::psst::util::demangle;
        throw ::std::runtime_error{
            "An instance of " + demangle<Event>() + " event was rejected"
        };
    }
};

}  /* namespace tags */
}  /* namespace def */
}  /* namespace afsm */


#endif /* AFSM_DETAIL_REJECT_POLICIES_HPP_ */
