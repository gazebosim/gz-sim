/*
 * transitions.hpp
 *
 *  Created on: 29 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_TRANSITIONS_HPP_
#define AFSM_DETAIL_TRANSITIONS_HPP_

#include <afsm/detail/actions.hpp>
#include <afsm/detail/exception_safety_guarantees.hpp>
#include <afsm/detail/event_identity.hpp>

#include <deque>
#include <memory>
#include <algorithm>

#include <iostream>

namespace afsm {
namespace transitions {

namespace detail {

enum class event_handle_type {
    none,
    transition,
    internal_transition,
    inner_state,
};

template <event_handle_type V>
using handle_event = ::std::integral_constant<event_handle_type, V>;

template < typename FSM, typename Event >
struct event_handle_selector
    : ::std::conditional<
        (::psst::meta::find_if<
            def::handles_event<Event>::template type,
            typename FSM::transitions >::type::size > 0),
        handle_event< event_handle_type::transition >,
        typename ::std::conditional<
              (::psst::meta::find_if<
                      def::handles_event<Event>::template type,
                      typename FSM::internal_transitions >::type::size > 0),
              handle_event< event_handle_type::internal_transition >,
              handle_event< event_handle_type::inner_state >
        >::type
    >::type{};

template <typename SourceState, typename Event, typename Transition>
struct transits_on_event
    : ::std::conditional<
        !def::traits::is_internal_transition<Transition>::value &&
        ::std::is_same< typename Transition::source_state_type, SourceState >::value &&
        ::std::is_same< typename Transition::event_type, Event >::value,
        ::std::true_type,
        ::std::false_type
    >::type {};

template < typename State, typename FSM, typename Event >
struct has_on_exit {
private:
    static FSM&         fsm;
    static Event const& event;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>().on_exit(event, fsm) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<State>(nullptr) )::value;
};

template < typename State, typename FSM, typename Event >
struct has_on_enter {
private:
    static FSM&         fsm;
    static Event&       event;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>().on_enter(::std::move(event), fsm) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<State>(nullptr) )::value;
};

template < typename Activity, typename FSM, typename State >
struct activity_has_start {
private:
    static FSM&         fsm;
    static State&       state;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>().start(fsm, state) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<Activity>(nullptr) )::value;
};

template < typename Activity, typename FSM, typename State >
struct activity_has_stop {
private:
    static FSM&         fsm;
    static State&       state;

    template < typename U >
    static ::std::true_type
    test( decltype( ::std::declval<U>().stop(fsm, state) ) const* );

    template < typename U >
    static ::std::false_type
    test(...);
public:
    static constexpr bool value = decltype( test<Activity>(nullptr) )::value;
};

template < typename Activity, typename FSM, typename State >
struct is_valid_activity {
    static constexpr bool value = activity_has_start<Activity, FSM, State>::value
            && activity_has_stop<Activity, FSM, State>::value;
};

template < typename FSM, typename State >
struct is_valid_activity < void, FSM, State >{
    static constexpr bool value = true;
};

template < typename FSM, typename State, typename Event, bool hasExit >
struct state_exit_impl {
    void
    operator()(State& state, Event const& event, FSM& fsm) const
    {
        state.state_exit(event, fsm);
        state.on_exit(event, fsm);
    }
};

template < typename FSM, typename State, typename Event >
struct state_exit_impl< FSM, State, Event, false > {
    void
    operator()(State& state, Event const& event, FSM& fsm) const
    {
        state.state_exit(event, fsm);
    }
};

// TODO Parameter to allow/disallow empty on_exit function
template < typename FSM, typename State, typename Event >
struct state_exit : state_exit_impl< FSM, State, Event,
    has_on_exit<State, FSM, Event>::value > {};

template < typename FSM, typename State, bool hasExit >
struct state_enter_impl {
    template < typename Event >
    void
    operator()(State& state, Event&& event, FSM& fsm) const
    {
        state.on_enter(::std::forward<Event>(event), fsm);
        state.state_enter(::std::forward<Event>(event), fsm);
    }
};

template < typename FSM, typename State >
struct state_enter_impl< FSM, State, false > {
    template < typename Event >
    void
    operator()(State& state, Event&& event, FSM& fsm) const
    {
        state.state_enter(::std::forward<Event>(event), fsm);
    }
};

// TODO Parameter to allow/disallow empty enter function
template < typename FSM, typename State, typename Event >
struct state_enter : state_enter_impl<FSM, State,
        has_on_enter<State, FSM, Event>::value> {};

template < typename FSM, typename State, bool HasHistory >
struct state_clear_impl {
    bool
    operator()(FSM& fsm, State& state) const
    {
        state = State{fsm};
        return true;
    }
};

template < typename FSM, typename State >
struct state_clear_impl< FSM, State, true > {
    bool
    operator()(FSM&, State&) const
    { return false; }
};

template < typename FSM, typename State >
struct state_clear : state_clear_impl< FSM, State,
    def::traits::has_history< State >::value > {};

template < typename FSM, typename StateTable >
struct no_transition {
    template < typename Event >
    actions::event_process_result
    operator()(StateTable&, Event&&) const
    {
        return actions::event_process_result::refuse;
    }
};

template < typename FSM, typename StateTable, typename Transition >
struct single_transition;

template < typename FSM, typename StateTable,
        typename SourceState, typename Event, typename TargetState,
        typename Action, typename Guard >
struct single_transition<FSM, StateTable,
    ::psst::meta::type_tuple< def::transition<SourceState, Event, TargetState, Action, Guard> > > {

    using fsm_type          = FSM;
    using state_table       = StateTable;
    using source_state_def  = SourceState;
    using target_state_def  = TargetState;

    using source_state_type = typename afsm::detail::front_state_type<source_state_def, FSM>::type;
    using target_state_type = typename afsm::detail::front_state_type<target_state_def, FSM>::type;

    using states_def        = typename fsm_type::inner_states_def;

    using guard_type        = actions::detail::guard_check<FSM, SourceState, Event, Guard>;
    using source_exit       = state_exit<fsm_type, source_state_type, Event>;
    using target_enter      = state_enter<fsm_type, target_state_type, Event>;
    using action_type       = actions::detail::action_invocation<Action, FSM,
            SourceState, TargetState>;
    using state_clear_type  = state_clear<FSM, source_state_type>;

    using source_index = ::psst::meta::index_of<source_state_def, states_def>;
    using target_index = ::psst::meta::index_of<target_state_def, states_def>;

    static_assert(source_index::found, "Failed to find source state index");
    static_assert(target_index::found, "Failed to find target state index");

    template < typename Evt >
    actions::event_process_result
    operator()(state_table& states, Evt&& event) const
    {
        return states.template transit_state< source_state_def, target_state_def >
            ( ::std::forward<Evt>(event), guard_type{}, action_type{},
                    source_exit{}, target_enter{}, state_clear_type{});
    }
};

template < ::std::size_t N, typename FSM, typename StateTable, typename Transitions >
struct nth_transition {
    static_assert(Transitions::size > N, "Transition list is too small");
    using transition            = typename Transitions::template type<N>;
    using event_type            = typename transition::event_type;
    using previous_transition   = nth_transition<N - 1, FSM, StateTable, Transitions>;
    using transition_invocation = single_transition<FSM, StateTable, ::psst::meta::type_tuple<transition>>;

    template < typename Event >
    actions::event_process_result
    operator()(StateTable& states, Event&& event) const
    {
        auto res = previous_transition{}(states, ::std::forward<Event>(event));
        if (res == actions::event_process_result::refuse) {
            return transition_invocation{}(states, ::std::forward<Event>(event));
        }
        return res;
    }
};

template < typename FSM, typename StateTable, typename Transitions >
struct nth_transition< 0, FSM, StateTable, Transitions > {
    static_assert(Transitions::size > 0, "Transition list is too small");
    using transition            = typename Transitions::template type<0>;
    using event_type            = typename transition::event_type;
    using transition_invocation = single_transition<FSM, StateTable, ::psst::meta::type_tuple<transition>>;

    template < typename Event >
    actions::event_process_result
    operator()(StateTable& states, Event&& event) const
    {
        return transition_invocation{}(states, ::std::forward<Event>(event));
    }
};

template < typename FSM, typename StateTable, typename Transitions >
struct conditional_transition {
    static constexpr ::std::size_t size = Transitions::size;
    static_assert(Transitions::size > 0, "Transition list is too small");

    using last_transition = nth_transition<size - 1, FSM, StateTable, Transitions>;

    template < typename Event >
    actions::event_process_result
    operator()(StateTable& states, Event&& event) const
    {
        return last_transition{}(states, ::std::forward<Event>(event));
    }
};

template < typename FSM, typename StateTable, typename Transitions >
struct transition_action_selector {
    using type = typename ::std::conditional<
            Transitions::size == 0,
            no_transition<FSM, StateTable >,
            typename ::std::conditional<
                Transitions::size == 1,
                single_transition<FSM, StateTable, Transitions>,
                conditional_transition<FSM, StateTable, Transitions>
            >::type
        >::type;
};

template < typename T, ::std::size_t StateIndex >
struct common_base_cast_func {
    static constexpr ::std::size_t state_index = StateIndex;
    using type  = typename ::std::decay<T>::type;

    template < typename StateTuple >
    type&
    operator()(StateTuple& states) const
    {
        return static_cast< type& >(::std::get< state_index >(states));
    }
    template < typename StateTuple >
    type const&
    operator()(StateTuple const& states) const
    {
        return static_cast< type const& >(::std::get< state_index >(states));
    }
};

template < ::std::size_t StateIndex >
struct final_state_exit_func {
    static constexpr ::std::size_t state_index = StateIndex;

    template < typename StateTuple, typename Event, typename FSM >
    void
    operator()(StateTuple& states, Event&& event, FSM& fsm)
    {
        using final_state_type = typename ::std::tuple_element< state_index, StateTuple >::type;
        using final_exit = state_exit< FSM, final_state_type, Event >;

        auto& final_state = ::std::get<state_index>(states);
        final_exit{}(final_state, ::std::forward<Event>(event), fsm);
    }
};

template < ::std::size_t StateIndex >
struct get_current_events_func {
    static constexpr ::std::size_t state_index = StateIndex;

    template < typename StateTuple >
    ::afsm::detail::event_set
    operator()(StateTuple const& states) const
    {
        auto const& state = ::std::get<state_index>(states);
        return state.current_handled_events();
    }
};

template < ::std::size_t StateIndex >
struct get_current_deferred_events_func {
    static constexpr ::std::size_t state_index = StateIndex;

    template < typename StateTuple >
    ::afsm::detail::event_set
    operator()(StateTuple const& states) const
    {
        auto const& state = ::std::get<state_index>(states);
        return state.current_deferrable_events();
    }
};

}  /* namespace detail */

template < typename FSM, typename FSM_DEF, typename Size >
class state_transition_table {
public:
    using fsm_type                      = FSM;
    using state_machine_definition_type = FSM_DEF;
    using size_type                     = Size;

    using this_type         = state_transition_table<FSM, FSM_DEF, Size>;

    using transitions       =
            typename state_machine_definition_type::transitions;
    static_assert(!::std::is_same<transitions, void>::value,
            "Transition table is not defined for a state machine");
    using transitions_tuple =
            typename transitions::transitions;
    using initial_state     =
            typename state_machine_definition_type::initial_state;
    using inner_states_def  =
            typename def::detail::inner_states< transitions >::type;
    using inner_states_constructor =
            afsm::detail::front_state_tuple< fsm_type, inner_states_def >;
    using inner_states_tuple =
            typename inner_states_constructor::type;
    using dispatch_table    =
            actions::detail::inner_dispatch_table< inner_states_tuple >;

    static constexpr ::std::size_t initial_state_index =
            ::psst::meta::index_of<initial_state, inner_states_def>::value;
    static constexpr ::std::size_t size = inner_states_def::size;

    using state_indexes     = typename ::psst::meta::index_builder<size>::type;
    using event_set         = ::afsm::detail::event_set;

    template < typename Event >
    using transition_table_type = ::std::array<
            ::std::function< actions::event_process_result(this_type&, Event&&) >, size >;

    template < typename Event >
    using exit_table_type = ::std::array<
            ::std::function< void(inner_states_tuple&, Event&&, fsm_type&) >, size >;

    using current_events_table = ::std::array<
            ::std::function< event_set(inner_states_tuple const&) >, size >;
    using available_transtions_table = ::std::array< event_set, size >;

    template < typename CommonBase, typename StatesTuple >
    using cast_table_type = ::std::array< ::std::function<
            CommonBase&( StatesTuple& ) >, size >;
public:
    state_transition_table(fsm_type& fsm)
        : fsm_{&fsm},
          current_state_{initial_state_index},
          states_{ inner_states_constructor::construct(fsm) }
    {}

    state_transition_table(fsm_type& fsm, state_transition_table const& rhs)
        : fsm_{&fsm},
          current_state_{ (::std::size_t)rhs.current_state_ },
          states_{ inner_states_constructor::copy_construct(fsm, rhs.states_) }
      {}
    state_transition_table(fsm_type& fsm, state_transition_table&& rhs)
        : fsm_{&fsm},
          current_state_{ (::std::size_t)rhs.current_state_ },
          states_{ inner_states_constructor::move_construct(fsm, ::std::move(rhs.states_)) }
      {}

    state_transition_table(state_transition_table const&) = delete;
    state_transition_table(state_transition_table&& rhs)
        : fsm_{rhs.fsm_},
          current_state_{ (::std::size_t)rhs.current_state_ },
          states_{ ::std::move(rhs.states_) }
    {}
    state_transition_table&
    operator = (state_transition_table const&) = delete;
    state_transition_table&
    operator = (state_transition_table&&) = delete;

    void
    swap(state_transition_table& rhs)
    {
        using ::std::swap;
        swap(current_state_, rhs.current_state_);
        swap(states_, rhs.states_);
        set_fsm(*fsm_);
        rhs.set_fsm(*rhs.fsm_);
    }

    void
    set_fsm(fsm_type& fsm)
    {
        fsm_ = &fsm;
        ::afsm::detail::set_enclosing_fsm< size - 1 >::set(fsm, states_);
    }

    inner_states_tuple&
    states()
    { return states_; }
    inner_states_tuple const&
    states() const
    { return states_; }

    template < ::std::size_t N>
    typename ::std::tuple_element< N, inner_states_tuple >::type&
    get_state()
    { return ::std::get<N>(states_); }
    template < ::std::size_t N>
    typename ::std::tuple_element< N, inner_states_tuple >::type const&
    get_state() const
    { return ::std::get<N>(states_); }

    ::std::size_t
    current_state() const
    { return (::std::size_t)current_state_; }

    void
    set_current_state(::std::size_t val)
    { current_state_ = val; }

    template < typename Event >
    actions::event_process_result
    process_event(Event&& event)
    {
        // Try dispatch to inner states
        auto res = dispatch_table::process_event(states_, current_state(),
                ::std::forward<Event>(event));
        if (res == actions::event_process_result::refuse) {
            // Check if the event can cause a transition and process it
            res = process_transition_event(::std::forward<Event>(event));
        }
        if (res == actions::event_process_result::process) {
            check_default_transition();
        }
        return res;
    }

    void
    check_default_transition()
    {
        auto const& ttable = transition_table<none>( state_indexes{} );
        ttable[current_state()](*this, none{});
    }

    template < typename Event >
    void
    enter(Event&& event)
    {
        using initial_state_type = typename ::std::tuple_element<initial_state_index, inner_states_tuple>::type;
        using initial_enter = detail::state_enter< fsm_type, initial_state_type, Event >;

        auto& initial = ::std::get< initial_state_index >(states_);
        initial_enter{}(initial, ::std::forward<Event>(event), *fsm_);
        check_default_transition();
    }
    template < typename Event >
    void
    exit(Event&& event)
    {
        auto const& table = exit_table<Event>( state_indexes{} );
        table[current_state()](states_, ::std::forward<Event>(event), *fsm_);
    }

    template < typename Event >
    actions::event_process_result
    process_transition_event(Event&& event)
    {
        auto const& inv_table = transition_table<Event>( state_indexes{} );
        return inv_table[current_state()](*this, ::std::forward<Event>(event));
    }

    template < typename SourceState, typename TargetState,
        typename Event, typename Guard, typename Action,
        typename SourceExit, typename TargetEnter, typename SourceClear >
    actions::event_process_result
    transit_state(Event&& event, Guard guard, Action action, SourceExit exit,
            TargetEnter enter, SourceClear clear)
    {
        using source_index = ::psst::meta::index_of<SourceState, inner_states_def>;
        using target_index = ::psst::meta::index_of<TargetState, inner_states_def>;

        static_assert(source_index::found, "Failed to find source state index");
        static_assert(target_index::found, "Failed to find target state index");

        auto& source = ::std::get< source_index::value >(states_);
        auto& target = ::std::get< target_index::value >(states_);
        return transit_state_impl(
                ::std::forward<Event>(event), source, target,
                 guard, action, exit, enter, clear,
                 target_index::value,
                 typename def::traits::exception_safety<state_machine_definition_type>::type{});
    }
    template < typename SourceState, typename TargetState,
        typename Event, typename Guard, typename Action,
        typename SourceExit, typename TargetEnter, typename SourceClear >
    actions::event_process_result
    transit_state_impl(Event&& event, SourceState& source, TargetState& target,
            Guard guard, Action action, SourceExit exit,
            TargetEnter enter, SourceClear clear,
            ::std::size_t target_index,
            def::tags::basic_exception_safety const&)
    {
        if (guard(*fsm_, source, event)) {
            auto const& observer = root_machine(*fsm_);
            exit(source, ::std::forward<Event>(event), *fsm_);
            observer.state_exited(*fsm_, source, event);
            action(::std::forward<Event>(event), *fsm_, source, target);
            enter(target, ::std::forward<Event>(event), *fsm_);
            observer.state_entered(*fsm_, target, event);
            if (clear(*fsm_, source))
                observer.state_cleared(*fsm_, source);
            current_state_ = target_index;
            observer.state_changed(*fsm_, source, target, event);
            return actions::event_process_result::process;
        }
        return actions::event_process_result::refuse;
    }
    template < typename SourceState, typename TargetState,
        typename Event, typename Guard, typename Action,
        typename SourceExit, typename TargetEnter, typename SourceClear >
    actions::event_process_result
    transit_state_impl(Event&& event, SourceState& source, TargetState& target,
            Guard guard, Action action, SourceExit exit,
            TargetEnter enter, SourceClear clear,
            ::std::size_t target_index,
            def::tags::strong_exception_safety const&)
    {
        SourceState source_backup{source};
        TargetState target_backup{target};
        try {
            return transit_state_impl(
                    ::std::forward<Event>(event), source, target,
                     guard, action, exit, enter, clear,
                     target_index,
                     def::tags::basic_exception_safety{});
        } catch (...) {
            using ::std::swap;
            swap(source, source_backup);
            swap(target, target_backup);
            throw;
        }
    }
    template < typename SourceState, typename TargetState,
        typename Event, typename Guard, typename Action,
        typename SourceExit, typename TargetEnter, typename SourceClear >
    actions::event_process_result
    transit_state_impl(Event&& event, SourceState& source, TargetState& target,
            Guard guard, Action action, SourceExit exit,
            TargetEnter enter, SourceClear clear,
            ::std::size_t target_index,
            def::tags::nothrow_guarantee const&)
    {
        SourceState source_backup{source};
        TargetState target_backup{target};
        try {
            return transit_state_impl(
                    ::std::forward<Event>(event), source, target,
                     guard, action, exit, enter, clear,
                     target_index,
                     def::tags::basic_exception_safety{});
        } catch (...) {}
        using ::std::swap;
        swap(source, source_backup);
        swap(target, target_backup);
        return actions::event_process_result::refuse;
    }

    event_set
    current_handled_events() const
    {
        auto const& table = get_current_events_table(state_indexes{});
        auto res = table[current_state_](states_);
        auto const& available_transitions
                            = get_available_transitions_table(state_indexes{});
        auto const& trans = available_transitions[current_state_];
        res.insert( trans.begin(), trans.end());
        return res;
    }

    event_set
    current_deferrable_events() const
    {
        auto const& table = get_current_deferred_events_table(state_indexes{});
        return table[current_state_](states_);
    }

    template < typename T >
    T&
    cast_current_state()
    {
        using target_type = typename ::std::decay<T>::type;
        auto const& ct = get_cast_table<target_type, inner_states_tuple>( state_indexes{} );
        return ct[current_state_]( states_ );
    }
    template < typename T >
    T const&
    cast_current_state() const
    {
        using target_type = typename ::std::add_const< typename ::std::decay<T>::type >::type;
        auto const& ct = get_cast_table<target_type, inner_states_tuple const>( state_indexes{} );
        return ct[current_state_]( states_ );
    }
private:
    template < typename Event, ::std::size_t ... Indexes >
    static transition_table_type< Event > const&
    transition_table( ::psst::meta::indexes_tuple< Indexes... > const& )
    {
        using event_type = typename ::std::decay<Event>::type;
        using event_transitions = typename ::psst::meta::find_if<
                def::handles_event< event_type >::template type, transitions_tuple >::type;
        static transition_table_type< Event > _table {{
            typename detail::transition_action_selector< fsm_type, this_type,
                typename ::psst::meta::find_if<
                    def::originates_from<
                        typename inner_states_def::template type< Indexes >
                    >::template type,
                    event_transitions
                >::type >::type{} ...
        }};
        return _table;
    }
    template < typename Event, ::std::size_t ... Indexes >
    static exit_table_type<Event> const&
    exit_table( ::psst::meta::indexes_tuple< Indexes... > const& )
    {
        static exit_table_type<Event> _table {{
            detail::final_state_exit_func<Indexes>{} ...
        }};
        return _table;
    }
    template < ::std::size_t ... Indexes >
    static current_events_table const&
    get_current_events_table( ::psst::meta::indexes_tuple< Indexes ... > const& )
    {
        static current_events_table _table{{
            detail::get_current_events_func<Indexes>{} ...
        }};

        return _table;
    }
    template < ::std::size_t ... Indexes >
    static current_events_table const&
    get_current_deferred_events_table( ::psst::meta::indexes_tuple< Indexes ... > const& )
    {
        static current_events_table _table{{
            detail::get_current_deferred_events_func<Indexes>{} ...
        }};

        return _table;
    }
    template < ::std::size_t ... Indexes >
    static available_transtions_table const&
    get_available_transitions_table( ::psst::meta::indexes_tuple< Indexes ...> const& )
    {
        static available_transtions_table _table{{
            ::afsm::detail::make_event_set(
                typename ::psst::meta::transform<
                    def::detail::event_type,
                    typename ::psst::meta::find_if<
                        def::originates_from<
                            typename inner_states_def::template type< Indexes >
                        >:: template type,
                        transitions_tuple
                    >::type
                 > ::type {}
            ) ...
        }};

        return _table;
    }
    template < typename T, typename StateTuple, ::std::size_t ... Indexes >
    static cast_table_type<T, StateTuple> const&
    get_cast_table( ::psst::meta::indexes_tuple< Indexes... > const& )
    {
        static cast_table_type<T, StateTuple> _table {{
            detail::common_base_cast_func<T, Indexes>{}...
        }};
        return _table;
    }
private:
    fsm_type*           fsm_;
    size_type           current_state_;
    inner_states_tuple  states_;
};

template < typename FSM, typename FSM_DEF, typename Size >
class state_transition_stack {
public:
    using state_table_type  = state_transition_table<FSM, FSM_DEF, Size>;
    using this_type         = state_transition_stack<FSM, FSM_DEF, Size>;

    using fsm_type                      = typename state_table_type::fsm_type;
    using state_machine_definition_type = typename state_table_type::state_machine_definition_type;
    using size_type                     = typename state_table_type::size_type;
    using inner_states_tuple            = typename state_table_type::inner_states_tuple;
    using event_set                     = typename state_table_type::event_set;

    using stack_constructor_type        = afsm::detail::stack_constructor<FSM, state_table_type>;
public:
    state_transition_stack(fsm_type& fsm)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::construct(fsm) }
    {}
    state_transition_stack(fsm_type& fsm, state_transition_stack const& rhs)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::copy_construct(fsm, rhs.state_stack_) }
    {}
    state_transition_stack(fsm_type& fsm, state_transition_stack&& rhs)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::move_construct(fsm, ::std::move(rhs.state_stack_)) }
    {}

    state_transition_stack(state_transition_stack const&) = delete;
    state_transition_stack(state_transition_stack&&) = delete;
    state_transition_stack&
    operator = (state_transition_stack const&) = delete;
    state_transition_stack&
    operator = (state_transition_stack&&) = delete;

    void
    swap(state_transition_stack& rhs)
    {
        using ::std::swap;
        swap(state_stack_, rhs.state_stack_);
        set_fsm(*fsm_);
        rhs.set_fsm(*rhs.fsm_);
    }

    void
    set_fsm(fsm_type& fsm)
    {
        fsm_ = &fsm;
        for (auto& item : state_stack_) {
            item.set_fsm(fsm);
        }
    }

    inner_states_tuple&
    states()
    { return top().states(); }
    inner_states_tuple const&
    states() const
    { return top().states(); }

    template < ::std::size_t N >
    typename ::std::tuple_element< N, inner_states_tuple >::type&
    get_state()
    { return top().template get_state<N>(); }
    template < ::std::size_t N >
    typename ::std::tuple_element< N, inner_states_tuple >::type const&
    get_state() const
    { return top().template get_state<N>(); }

    ::std::size_t
    current_state() const
    { return top().current_state(); }

    template < typename Event >
    actions::event_process_result
    process_event(Event&& event)
    {
        return top().process_event(::std::forward<Event>(event));
    }

    template < typename Event >
    void
    enter(Event&& event)
    {
        top().enter(::std::forward<Event>(event));
    }
    template < typename Event >
    void
    exit(Event&& event)
    {
        top().exit(::std::forward<Event>(event));
    }

    template < typename Event >
    void
    push(Event&& event)
    {
        state_stack_.emplace_back( *fsm_ );
        enter(::std::forward<Event>(event));
    }

    template < typename Event >
    void
    pop(Event&& event)
    {
        if (state_stack_.size() > 1) {
            exit(::std::forward<Event>(event));
            state_stack_.pop_back();
        }
    }

    ::std::size_t
    stack_size() const
    {
        return state_stack_.size();
    }

    event_set
    current_handled_events() const
    {
        return top().current_handled_events();
    }
    event_set
    current_deferrable_events() const
    {
        return top().current_deferrable_events();
    }

    template < typename T >
    T&
    cast_current_state()
    {
        return top().template cast_current_state<T>();
    }
    template < typename T >
    T const&
    cast_current_state() const
    {
        return top().template cast_current_state<T>();
    }
private:
    using stack_type = typename stack_constructor_type::type;

    state_table_type&
    top()
    { return state_stack_.back(); }
    state_table_type const&
    top() const
    { return state_stack_.back(); }
private:
    fsm_type*           fsm_;
    stack_type          state_stack_;
};

template < typename FSM, typename FSM_DEF, typename Size, bool HasPusdowns >
struct transition_container_selector {
    using type = state_transition_table<FSM, FSM_DEF, Size>;
};

template <typename FSM, typename FSM_DEF, typename Size>
struct transition_container_selector<FSM, FSM_DEF, Size, true> {
    using type = state_transition_stack<FSM, FSM_DEF, Size>;
};

namespace detail {

template < typename FSM, typename FSM_DEF, typename Size, bool HasPushdowns >
struct transition_container {
    using transitions_tuple = typename transition_container_selector<FSM, FSM_DEF, Size, HasPushdowns>::type;

    transition_container(FSM* fsm)
        : transitions_{*fsm}
    {}
    transition_container(FSM* fsm, transition_container const& rhs)
        : transitions_{*fsm, rhs.transitions_}
    {}
    transition_container(FSM* fsm, transition_container&& rhs)
        : transitions_{*fsm, ::std::move(rhs.transitions_)}
    {}
protected:
    transitions_tuple   transitions_;
};

template < typename FSM, typename FSM_DEF, typename Size >
struct transition_container< FSM, FSM_DEF, Size, true > {
    using transitions_tuple = typename transition_container_selector<FSM, FSM_DEF, Size, true>::type;

    transition_container(FSM* fsm)
        : transitions_{*fsm}
    {}
    transition_container(FSM* fsm, transition_container const& rhs)
        : transitions_{*fsm, rhs.transitions_}
    {}
    transition_container(FSM* fsm, transition_container&& rhs)
        : transitions_{*fsm, ::std::move(rhs.transitions_)}
    {}

    ::std::size_t
    stack_size() const
    {
        return transitions_.stack_size();
    }

protected:
    template < typename T >
    friend struct ::afsm::detail::pushdown_state;
    template < typename T >
    friend struct ::afsm::detail::popup_state;
    // Push/pop ops
    template < typename Event >
    void
    pushdown(Event&& event)
    {
        transitions_.push(::std::forward<Event>(event));
    }
    template < typename Event >
    void
    popup(Event&& event)
    {
        transitions_.pop(::std::forward<Event>(event));
    }
protected:
    transitions_tuple  transitions_;
};
}  /* namespace detail */

template < typename FSM, typename FSM_DEF, typename Size >
struct transition_container
    : detail::transition_container<FSM, FSM_DEF, Size,
          def::has_pushdown_stack<FSM_DEF>::value> {
    using base_type = detail::transition_container<FSM, FSM_DEF, Size,
                            def::has_pushdown_stack<FSM_DEF>::value>;
    using transitions_tuple = typename base_type::transitions_tuple;

    transition_container( FSM* fsm ) : base_type{fsm} {}
    transition_container(FSM* fsm, transition_container const& rhs)
        : base_type{fsm, rhs} {}
    transition_container(FSM* fsm, transition_container&& rhs)
        : base_type{fsm, ::std::move(rhs)} {}
protected:
    using base_type::transitions_;
};

}  /* namespace transitions */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_TRANSITIONS_HPP_ */
