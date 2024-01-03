/*
 * helpers.hpp
 *
 *  Created on: 29 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_HELPERS_HPP_
#define AFSM_DETAIL_HELPERS_HPP_

#include <afsm/definition.hpp>
#include <type_traits>
#include <mutex>
#include <atomic>
#include <deque>
#include <algorithm>

namespace afsm {
namespace detail {

template <typename ... T>
struct not_a_state;

template < typename T >
struct not_a_state<T> {
    static_assert( def::traits::is_state<T>::value, "Type is not a state" );
};

template < typename T, typename FSM >
struct front_state_type
    : ::std::conditional<
        def::traits::is_state_machine< T >::value,
        inner_state_machine< T, FSM >,
        typename ::std::conditional<
            def::traits::is_state< T >::value,
            state< T, FSM >,
            not_a_state< T >
        >::type
    > {};

template < typename T, typename ... Y, typename FSM >
struct front_state_type<::psst::meta::type_tuple<T, Y...>, FSM>
    : front_state_type<
          ::psst::meta::type_tuple<Y...>,
          typename front_state_type< T, FSM >::type > {};

template < typename T, typename FSM >
struct front_state_type<::psst::meta::type_tuple<T>, FSM>
    : front_state_type<T, FSM> {};

template < typename FSM, typename T >
struct front_state_tuple;

template < typename FSM >
struct front_state_tuple< FSM, void> {
    using type = ::std::tuple<>;

    static type
    construct(FSM&)
    { return type{}; }
};

template < typename FSM, typename ... T>
struct front_state_tuple< FSM, ::psst::meta::type_tuple<T...> > {
    using type          = ::std::tuple< typename front_state_type<T, FSM>::type ... >;
    using index_tuple   = typename ::psst::meta::index_builder< sizeof ... (T) >::type;

    static type
    construct(FSM& fsm)
    { return type( typename front_state_type<T, FSM>::type{fsm}... ); }
    static type
    copy_construct(FSM& fsm, type const& rhs)
    {
        return copy_construct(fsm, rhs, index_tuple{});
    }
    static type
    move_construct(FSM& fsm, type&& rhs)
    {
        return move_construct(fsm, ::std::forward<type>(rhs), index_tuple{});
    }
private:
    template < ::std::size_t ... Indexes >
    static type
    copy_construct(FSM& fsm, type const& rhs, ::psst::meta::indexes_tuple<Indexes...> const&)
    {
        return type( typename front_state_type<T, FSM>::type{
            fsm, ::std::get< Indexes >(rhs)}...);
    }
    template < ::std::size_t ... Indexes >
    static type
    move_construct(FSM& fsm, type&& rhs, ::psst::meta::indexes_tuple<Indexes...> const&)
    {
        return type( typename front_state_type<T, FSM>::type{
            fsm, ::std::move(::std::get< Indexes >(rhs))}...);
    }
};

template < typename FSM, typename State, bool Contains >
struct substate_type_impl {
    using full_path = typename def::state_path<FSM, State>::type;
    using path      = typename ::psst::meta::pop_front<full_path>::type;
    using type      = typename front_state_type<path, FSM>::type;
    using front     = typename ::psst::meta::front<path>::type;
};

template < typename FSM, typename State, bool IsSelf >
struct substate_type_self {
    using type      = FSM;
};

template < typename FSM, typename State >
struct substate_type_self<FSM, State, false > {};

template < typename FSM, typename State >
struct substate_type_impl<FSM, State, false>
    : substate_type_self <FSM, State, ::std::is_base_of<State, FSM>::value> {};

template < typename FSM, typename State >
struct substate_type
    : substate_type_impl<FSM, State, def::contains_substate<FSM, State>::value>{};

template < typename FSM, typename StateTable >
struct stack_constructor {
    using state_table_type  = StateTable;
    using stack_item        = state_table_type;
    using type              = ::std::deque<stack_item>;
    static constexpr ::std::size_t size = state_table_type::size;

    static type
    construct(FSM& fsm)
    {
        type res;
        res.emplace_back(state_table_type{fsm});
        return res;
    }
    static type
    copy_construct(FSM& fsm, type const& rhs)
    {
        type res;
        ::std::transform(rhs.begin(), rhs.end(), ::std::back_inserter(res),
            [fsm](stack_item const& item)
            {
                return stack_item{ fsm, item };
            });
        return res;
    }
    static type
    move_construct(FSM& fsm, type&& rhs)
    {
        type res;
        ::std::transform(rhs.begin(), rhs.end(), ::std::back_inserter(res),
            [fsm](stack_item&& item)
            {
                return stack_item{fsm, ::std::move(item) };
            });
        return res;
    }
};

struct no_lock {
    no_lock(none&) {}
};

template < typename Mutex >
struct lock_guard_type {
    using type = ::std::lock_guard<Mutex>;
};

template <>
struct lock_guard_type<none> {
    using type = no_lock;
};
template <>
struct lock_guard_type<none&> {
    using type = no_lock;
};

template <typename Mutex>
struct size_type {
    using type = ::std::atomic<::std::size_t>;
};

template <>
struct size_type<none> {
    using type = ::std::size_t;
};
template <>
struct size_type<none&> {
    using type = ::std::size_t;
};

}  /* namespace detail */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_HELPERS_HPP_ */
