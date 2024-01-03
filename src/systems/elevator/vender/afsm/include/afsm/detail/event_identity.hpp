/*
 * event_identity.hpp
 *
 *  Created on: Dec 20, 2016
 *      Author: zmij
 */

#ifndef AFSM_DETAIL_EVENT_IDENTITY_HPP_
#define AFSM_DETAIL_EVENT_IDENTITY_HPP_

#include <type_traits>
#include <set>
#include <pushkin/meta/type_tuple.hpp>

namespace afsm {
namespace detail {

struct event_base {
    struct id_type {};
};

template < typename T >
struct event : event_base {
    static constexpr id_type id{};
};

template < typename T >
constexpr event_base::id_type event<T>::id;

template < typename T >
struct event_identity {
    using event_type    = typename ::std::decay<T>::type;
    using type          = event<event_type>;
};

using event_set         = ::std::set< event_base::id_type const* >;

template < typename ... T >
event_set
make_event_set( ::psst::meta::type_tuple<T...> const& )
{
    return event_set{
        &event_identity<T>::type::id...
    };
}

}  /* namespace detail */
}  /* namespace afsm */



#endif /* AFSM_DETAIL_EVENT_IDENTITY_HPP_ */
