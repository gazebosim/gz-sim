/*
 * actions.hpp
 *
 *  Created on: 28 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_ACTIONS_HPP_
#define AFSM_DETAIL_ACTIONS_HPP_

#include <afsm/definition.hpp>
#include <functional>
#include <array>

namespace afsm {

namespace detail {

template < ::std::size_t StateIndex >
struct set_enclosing_fsm {
    static constexpr ::std::size_t state_index = StateIndex;

    template < typename FSM, typename ... T >
    static void
    set(FSM& fsm, ::std::tuple<T...>& states)
    {
        set_enclosing_fsm<StateIndex - 1>::set(fsm, states);
        ::std::get<state_index>(states).enclosing_fsm(fsm);
    }
};

template <>
struct set_enclosing_fsm<0> {
    static constexpr ::std::size_t state_index = 0;

    template < typename FSM, typename ... T >
    static void
    set(FSM& fsm, ::std::tuple<T...>& states)
    {
        ::std::get<state_index>(states).enclosing_fsm(fsm);
    }
};

}  /* namespace detail */

namespace actions {

enum class event_process_result {
    refuse,
    defer,
    process_in_state,    /**< Process with in-state transition */
    process,
};

/**
 * The event was accepted and either processed or deferred for later processing.
 * @param res
 * @return
 */
inline bool
ok(event_process_result res)
{
    return res != event_process_result::refuse;
}

/**
 * The event was processed either with or without state transition.
 * @param res
 * @return
 */
inline bool
done(event_process_result res)
{
    return res == event_process_result::process ||
            res == event_process_result::process_in_state;
}

namespace detail {

template <typename Action, typename Event, typename FSM,
        typename SourceState, typename TargetState>
struct action_long_signature {
private:
    static FSM&         fsm;
    static Event&       event;
    static SourceState& source;
    static TargetState& target;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>()(::std::move(event), fsm, source, target) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<Action>(nullptr) )::value;
};

template <typename Action, typename Event, typename FSM>
struct action_short_signature {
private:
    static FSM&         fsm;
    static Event&       event;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>()(::std::move(event), fsm) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<Action>(nullptr) )::value;
};

template <typename FSM, typename Event>
struct handles_reject {
private:
    static FSM&         fsm;
    static Event&       event;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>().reject_event(::std::move(event), fsm) ) const * );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<FSM>(nullptr) )::value;
};

template <typename Event, typename Transition>
struct handles_event
    : ::std::conditional<
        ::std::is_same< typename Transition::event_type, Event >::value,
        ::std::true_type,
        ::std::false_type
    >::type {};

template < typename Guard, typename FSM, typename State, typename Event >
struct guard_wants_event
    : ::std::integral_constant<bool,
        ::psst::meta::is_callable< Guard, FSM const&, State const&, Event const& >::value> {};

template < typename Guard, typename FSM, typename State, typename Event >
struct guard_wants_event< ::psst::meta::not_<Guard>, FSM, State, Event >
    : ::std::integral_constant<bool,
        ::psst::meta::is_callable< Guard, FSM const&, State const&, Event const& >::value> {};

template < bool WantEvent, typename FSM, typename State, typename Event, typename Guard >
struct guard_event_check {
    bool
    operator()(FSM const& fsm, State const& state, Event const&) const
    { return Guard{}(fsm, state); }
};

template < typename FSM, typename State, typename Event, typename Guard >
struct guard_event_check< true, FSM, State, Event, Guard > {
    bool
    operator()(FSM const& fsm, State const& state, Event const& event) const
    { return Guard{}(fsm, state, event); }
};

template < typename FSM, typename State, typename Event, typename Guard >
struct guard_check : ::std::conditional<
            guard_wants_event< Guard, FSM, State, Event >::value,
            guard_event_check<true, FSM, State, Event, Guard>,
            typename ::std::conditional<
                ::psst::meta::is_callable< Guard, FSM const&, State const& >::value,
                guard_event_check<false, FSM, State, Event, Guard>,
                guard_check<FSM, State, Guard, none> // TODO Replace with concept assert
            >::type
        >::type {};

template < typename FSM, typename State, typename Event, typename ... Guards >
struct guard_check< FSM, State, Event, ::psst::meta::and_<Guards...> > {
    using check_function = ::psst::meta::and_< guard_check< FSM, State, Event, Guards >... >;
    bool
    operator()(FSM const& fsm, State const& state, Event const& event) const
    {
        return check_function{}(fsm, state, event);
    }
};

template < typename FSM, typename State, typename Event, typename ... Guards >
struct guard_check< FSM, State, Event, ::psst::meta::or_<Guards...> > {
    using check_function = ::psst::meta::or_< guard_check< FSM, State, Event, Guards >... >;
    bool
    operator()(FSM const& fsm, State const& state, Event const& event) const
    {
        return check_function{}(fsm, state, event);
    }
};

template < typename FSM, typename State, typename Event >
struct guard_check< FSM, State, Event, none > {
    constexpr bool
    operator()(FSM const&, State const&, Event const&) const
    { return true; }
};

template < typename Action, typename Event, typename FSM,
    typename SourceState, typename TargetState, bool LongSignature >
struct action_invocation_impl {
    void
    operator()(Event&& event, FSM& fsm, SourceState& source, TargetState& target) const
    {
        static_assert(action_long_signature< Action, Event,
                    FSM, SourceState, TargetState >::value,
                "Action is not callable for this transition");
        Action{}(::std::forward<Event>(event), fsm, source, target);
    }
};

template < typename Action, typename Event, typename FSM,
    typename SourceState, typename TargetState >
struct action_invocation_impl<Action, Event, FSM, SourceState, TargetState, false> {
    void
    operator()(Event&& event, FSM& fsm, SourceState&, TargetState&) const
    {
        static_assert(action_short_signature< Action, Event,
                    FSM >::value,
                "Action is not callable for this transition");
        Action{}(::std::forward<Event>(event), fsm);
    }
};


template < typename Action, typename FSM,
    typename SourceState, typename TargetState >
struct action_invocation {
    template < typename Event >
    void
    operator()(Event&& event, FSM& fsm, SourceState& source, TargetState& target) const
    {
        using invocation_type = action_invocation_impl<
                                    Action, Event, FSM,
                                    SourceState, TargetState,
                                    action_long_signature<Action, Event, FSM,
                                            SourceState, TargetState>::value>;
        invocation_type{}(::std::forward<Event>(event), fsm, source, target);
    }
};

template < typename FSM,
    typename SourceState, typename TargetState >
struct action_invocation< none, FSM, SourceState, TargetState > {
    template < typename Event >
    void
    operator()(Event&&, FSM&, SourceState&, TargetState&) const
    {}
};

template < typename Action, typename Guard, typename FSM,
    typename SourceState, typename TargetState >
struct guarded_action_invocation {
    using invocation_type   = action_invocation< Action, FSM, SourceState, TargetState >;

    template < typename Event >
    event_process_result
    operator()(Event&& event, FSM& fsm, SourceState& source, TargetState& target) const
    {
        using guard_type        = guard_check< FSM, SourceState, Event, Guard >;
        if (guard_type{}(fsm, source, ::std::forward<Event>(event))) {
            invocation_type{}(::std::forward<Event>(event), fsm, source, target);
            return event_process_result::process;
        }
        return event_process_result::refuse;
    }
};

template < ::std::size_t N, typename FSM, typename State, typename Transitions >
struct nth_inner_invocation {
    static_assert(Transitions::size > N, "Transitions list is too small");
    using transition        = typename Transitions::template type<N>;
    using action_type       = typename transition::action_type;
    using guard_type        = typename transition::guard_type;
    using invocation_type   = guarded_action_invocation<
                action_type, guard_type, FSM, State, State >;
    using previous_action    = nth_inner_invocation<N - 1, FSM, State, Transitions>;

    template < typename Event >
    event_process_result
    operator()(Event&& event, FSM& fsm, State& state) const
    {
        auto res = previous_action{}(::std::forward<Event>(event), fsm, state);
        if (res == event_process_result::refuse)
            return invocation_type{}(::std::forward<Event>(event), fsm, state, state);
        return res;
    }
};

template < typename FSM, typename State, typename Transitions >
struct nth_inner_invocation<0, FSM, State, Transitions> {
    static_assert(Transitions::size > 0, "Transitions list is empty");
    using transition        = typename Transitions::template type<0>;
    using action_type       = typename transition::action_type;
    using guard_type        = typename transition::guard_type;
    using invocation_type   = guarded_action_invocation<
                action_type, guard_type, FSM, State, State >;

    template < typename Event >
    event_process_result
    operator()(Event&& event, FSM& fsm, State& state) const
    {
        return invocation_type{}(::std::forward<Event>(event), fsm, state, state);
    }
};

struct no_in_state_invocation {
    template < typename FSM, typename State, typename Event >
    event_process_result
    operator()(Event&&, FSM&, State&) const
    {
        static_assert(::std::is_same<Event, State>::value, "");
        return event_process_result::refuse;
    }
};

template < typename FSM, typename State, typename Transition >
struct unconditional_in_state_invocation {
    using action_type = typename Transition::action_type;
    using guard_type  = typename Transition::guard_type;
    using invocation_type = guarded_action_invocation<
                action_type, guard_type, FSM, State, State >;

    template < typename Event >
    event_process_result
    operator()(Event&& event, FSM& fsm, State& state) const
    {
        return invocation_type{}(::std::forward<Event>(event), fsm, state, state);
    }
};

template < typename FSM, typename State, typename Transitions >
struct conditional_in_state_invocation {
    static constexpr ::std::size_t size = Transitions::size;
    using invocation_type = nth_inner_invocation< size - 1, FSM, State, Transitions>;

    template < typename Event >
    event_process_result
    operator()(Event&& event, FSM& fsm, State& state) const
    {
        return invocation_type{}(::std::forward<Event>(event), fsm, state);
    }
};

template < bool hasActions, typename State, typename Event >
struct is_in_state_action {
    using state_type        = State;
    using event_type        = Event;
    using transitions       = typename state_type::internal_transitions;
    using event_handlers    = typename ::psst::meta::find_if<
        def::handles_event<event_type>::template type,
        typename transitions::transitions >::type;

    static constexpr bool value = event_handlers::size > 0;
};

template < typename State, typename Event >
struct is_in_state_action <false, State, Event> {
    static constexpr bool value = false;
};

template < bool hasActions, typename FSM, typename State, typename Event >
struct in_state_action_invocation {
    using fsm_type          = FSM;
    using state_type        = State;
    using event_type        = Event;

    using transitions       = typename state_type::internal_transitions;
    static_assert( !::std::is_same<transitions, void>::value,
            "State doesn't have internal transitions table" );

    using event_handlers    = typename ::psst::meta::find_if<
        def::handles_event<event_type>::template type,
        typename transitions::transitions >::type;
    static_assert( event_handlers::size > 0, "State doesn't handle event" );
    using handler_type      = typename ::std::conditional<
                event_handlers::size == 1,
                detail::unconditional_in_state_invocation<
                    fsm_type, state_type, typename event_handlers::template type<0>>,
                detail::conditional_in_state_invocation< fsm_type, state_type, event_handlers>
            >::type;

    template < typename Evt >
    event_process_result
    operator()(Evt&& event, fsm_type& fsm, state_type& state) const
    {
       auto res = handler_type{}(::std::forward<Evt>(event), fsm, state);
       if (res == event_process_result::process) {
           return event_process_result::process_in_state;
       }
       return res;
    }
};

template < typename FSM, typename State, typename Event >
struct in_state_action_invocation<false, FSM, State, Event> {
    using fsm_type          = FSM;
    using state_type        = State;
    using event_type        = Event;

    template < typename Evt >
    event_process_result
    operator()(Evt&&, fsm_type&, state_type&) const
    {
       return event_process_result::refuse;
    }
};

}  /* namespace detail */

template < typename State, typename Event >
struct is_in_state_action : detail::is_in_state_action <
        !::std::is_same<typename State::internal_transitions, void>::value &&
        ::psst::meta::contains<Event, typename State::internal_events>::value,
        State, Event >{};

template < typename FSM, typename State, typename Event >
struct in_state_action_invocation :
        detail::in_state_action_invocation<
            !::std::is_same<typename State::internal_transitions, void>::value &&
            ::psst::meta::contains<Event, typename State::internal_events>::value,
            FSM, State, Event > {
};

template < typename FSM, typename State, typename Event >
event_process_result
handle_in_state_event(Event&& event, FSM& fsm, State& state)
{
    using decayed_event = typename ::std::decay<Event>::type;
    return in_state_action_invocation< FSM, State, decayed_event >{}
        (::std::forward<Event>(event), fsm, state);
}

namespace detail {

template < ::std::size_t StateIndex >
struct process_event_handler {
    static constexpr ::std::size_t state_index = StateIndex;
    template < typename StateTuple, typename Event >
    event_process_result
    operator()(StateTuple& states, Event&& event) const
    {
        return ::std::get<state_index>(states).process_event(::std::forward<Event>(event));
    }
};

template < typename Indexes >
struct handlers_tuple;

template < ::std::size_t ... Indexes >
struct handlers_tuple < ::psst::meta::indexes_tuple<Indexes...> > {
    using type = ::std::tuple< process_event_handler<Indexes>... >;
    static constexpr ::std::size_t size = sizeof ... (Indexes);
};

template < typename T >
class inner_dispatch_table;

template < typename ... T >
class inner_dispatch_table< ::std::tuple<T...> > {
public:
    static constexpr ::std::size_t size = sizeof ... (T);
    using states_tuple      = ::std::tuple<T...>;
    using indexes_tuple     = typename ::psst::meta::index_builder< size >::type;
    using dispatch_tuple    = typename handlers_tuple<indexes_tuple>::type;
    template < typename Event >
    using invocation_table  = ::std::array<
            ::std::function< event_process_result(states_tuple&, Event&&) >, size >;
public:
    explicit
    inner_dispatch_table() {}

    template < typename Event >
    static event_process_result
    process_event(states_tuple& states, ::std::size_t current_state, Event&& event)
    {
        //using event_type = typename ::std::decay<Event>::type;
        if (current_state >= size)
            throw ::std::logic_error{ "Invalid current state index" };
        auto inv_table = state_table< Event >(indexes_tuple{});
        return inv_table[current_state](states, ::std::forward<Event>(event));
    }
private:
    template < typename Event, ::std::size_t ... Indexes >
    static invocation_table<Event> const&
    state_table( ::psst::meta::indexes_tuple< Indexes... > const& )
    {
        static invocation_table<Event> _table {{ process_event_handler<Indexes>{}... }};
        return _table;
    }
};

}  /* namespace detail */

}  /* namespace actions */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_ACTIONS_HPP_ */
