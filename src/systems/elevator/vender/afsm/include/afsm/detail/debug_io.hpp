/*
 * debug_io.hpp
 *
 *  Created on: 2 июня 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_DEBUG_IO_HPP_
#define AFSM_DETAIL_DEBUG_IO_HPP_

#include <afsm/detail/actions.hpp>
#include <iostream>

namespace afsm {
namespace actions {

inline ::std::ostream&
operator << (::std::ostream& os, event_process_result const& val)
{
    ::std::ostream::sentry s (os);
    if (s) {
        switch (val) {
            case event_process_result::refuse:
                os << "refuse";
                break;
            case event_process_result::process:
                os << "transit";
                break;
            case event_process_result::process_in_state:
                os << "in-state";
                break;
            case event_process_result::defer:
                os << "defer";
                break;
            default:
                break;
        }
    }
    return os;
}


}  /* namespace actions */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_DEBUG_IO_HPP_ */
