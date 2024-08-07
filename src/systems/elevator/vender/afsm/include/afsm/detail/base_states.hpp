/*
 * base_states.hpp
 *
 *  Created on: 28 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_BASE_STATES_HPP_
#define AFSM_DETAIL_BASE_STATES_HPP_

#include <afsm/definition.hpp>
#include <afsm/detail/helpers.hpp>
#include <afsm/detail/transitions.hpp>
#include <afsm/detail/orthogonal_regions.hpp>
#include <afsm/detail/event_identity.hpp>

#include <pushkin/meta/functions.hpp>

namespace afsm {
namespace detail {

template < actions::event_process_result R >
using process_type = ::std::integral_constant< actions::event_process_result, R >;

template <typename Event, typename HandledEvents,
        typename DeferredEvents = ::psst::meta::type_tuple<>>
struct event_process_selector
    : ::std::conditional<
        ::psst::meta::contains<typename ::std::decay<Event>::type, HandledEvents>::value,
        process_type<actions::event_process_result::process>,
        typename ::std::conditional<
            ::psst::meta::contains<typename ::std::decay<Event>::type, DeferredEvents>::value,
            process_type<actions::event_process_result::defer>,
            process_type<actions::event_process_result::refuse>
        >::type
    >::type{};

enum class state_containment {
    none,
    self,
    immediate,
    substate
};

template < state_containment C >
using containment_type = ::std::integral_constant<state_containment, C>;

using state_containment_none        = containment_type< state_containment::none >;
using state_containment_self        = containment_type< state_containment::self >;
using state_containment_immediate   = containment_type< state_containment::immediate >;
using state_containment_substate    = containment_type< state_containment::substate >;

template < typename StateDef, typename MachineDef, typename InnerStates >
struct state_containment_type :
        ::std::conditional<
            ::std::is_same< MachineDef, StateDef >::value,
            state_containment_self,
            typename ::std::conditional <
                ::psst::meta::contains<StateDef, InnerStates>::value,
                state_containment_immediate,
                typename ::std::conditional<
                    ::psst::meta::any_match<
                        def::contains_substate_predicate<StateDef>::template type, InnerStates >::value,
                        state_containment_substate,
                        state_containment_none
                >::type
            >::type
        >::type {};

template < typename T, bool isTerminal >
struct state_base_impl : T {
    using state_definition_type = T;
    using state_type            = state_base_impl<T, isTerminal>;
    using internal_transitions = typename state_definition_type::internal_transitions;

    static_assert(def::traits::is_state<T>::value,
            "Front state can be created only with a descendant of afsm::def::state");
    static_assert(!def::detail::has_inner_states< internal_transitions >::value,
            "Internal transition table cannot have transitions between other states");
    using handled_events =
            typename def::detail::handled_events<state_definition_type>::type;
    using internal_events =
            typename def::detail::handled_events<state_definition_type>::type;
    static_assert(def::traits::is_state_machine<state_definition_type>::value
                || !def::detail::has_default_transitions< handled_events >::value,
            "Internal transition cannot be a default transition");
    using deferred_events =
            typename ::std::conditional<
                ::std::is_same<typename T::deferred_events, void>::value,
                ::psst::meta::type_tuple<>,
                typename ::psst::meta::unique< typename T::deferred_events >::type
            >::type;

    state_base_impl() : state_definition_type{} {}
    state_base_impl(state_base_impl const&) = default;
    state_base_impl(state_base_impl&&) = default;

    state_base_impl&
    operator = (state_base_impl const&) = delete;
    state_base_impl&
    operator = (state_base_impl&&) = delete;

    void
    swap(state_base_impl& rhs) noexcept
    {
        using ::std::swap;
        swap(static_cast<T&>(*this), static_cast<T&>(rhs));
    }
    template < typename Event, typename FSM >
    void
    state_enter(Event&&, FSM&) {}
    template < typename Event, typename FSM >
    void
    state_exit(Event&&, FSM&) {}

    event_set const&
    current_handled_events() const
    { return static_handled_events(); }
    event_set const&
    current_deferrable_events() const
    { return static_deferrable_events(); }

    static event_set const&
    static_handled_events()
    {
        static event_set evts_ = make_event_set(handled_events{});
        return evts_;
    }
    static event_set const&
    internal_handled_events()
    {
        static event_set evts_ = make_event_set(typename def::detail::handled_events< internal_transitions >::type{});
        return evts_;
    }

    static event_set const&
    static_deferrable_events()
    {
        static event_set evts_ = make_event_set(deferred_events{});
        return evts_;
    }
protected:
    template< typename ... Args >
    state_base_impl(Args&& ... args)
        : state_definition_type(::std::forward<Args>(args)...) {}
};

template < typename T >
struct state_base_impl<T, true> : T {
    using state_definition_type = T;
    using state_type            = state_base_impl<T, false>;
    using internal_transitions = typename state_definition_type::internal_transitions;

    static_assert(def::traits::is_state<T>::value,
            "Front state can be created only with a descendant of afsm::def::state");
    static_assert(::std::is_same<
            typename state_definition_type::internal_transitions, void >::value,
            "Terminal state must not define transitions");
    static_assert(::std::is_same<
            typename state_definition_type::deferred_events, void >::value,
            "Terminal state must not define deferred events");

    using handled_events  = ::psst::meta::type_tuple<>;
    using internal_events = ::psst::meta::type_tuple<>;
    using deferred_events = ::psst::meta::type_tuple<>;

    state_base_impl() : state_definition_type{} {}
    state_base_impl(state_base_impl const&) = default;
    state_base_impl(state_base_impl&&) = default;

    state_base_impl&
    operator = (state_base_impl const&) = delete;
    state_base_impl&
    operator = (state_base_impl&&) = delete;

    void
    swap(state_base_impl& rhs) noexcept
    {
        using ::std::swap;
        swap(static_cast<T&>(*this), static_cast<T&>(rhs));
    }

    template < typename Event, typename FSM >
    void
    state_enter(Event&&, FSM&) {}
    template < typename Event, typename FSM >
    void
    state_exit(Event&&, FSM&) {}

    event_set const&
    current_handled_events() const
    { return static_handled_events(); }
    event_set const&
    current_deferrable_events() const
    { return static_deferrable_events(); }

    static event_set const&
    static_handled_events()
    {
        static event_set evts_{};
        return evts_;
    }
    static event_set const&
    internal_handled_events()
    {
        static event_set evts_{};
        return evts_;
    }
    static event_set const&
    static_deferrable_events()
    {
        static event_set evts_{};
        return evts_;
    }
protected:
    template< typename ... Args >
    state_base_impl(Args&& ... args)
        : state_definition_type(::std::forward<Args>(args)...) {}
};

template < typename T >
struct pushdown_state : state_base_impl<T, false> {
    using pushdown_machine_type = typename T::pushdown_machine_type;

    template < typename Event, typename FSM >
    void
    state_enter(Event&& event, FSM& fsm)
    {
        root_machine(fsm).template get_state< pushdown_machine_type >().pushdown(::std::forward<Event>(event));
    }
};

template < typename T >
struct popup_state : state_base_impl<T, true> { // TODO Check if the state is terminal by tags
    using pushdown_machine_type = typename T::pushdown_machine_type;

    template < typename Event, typename FSM >
    void
    state_enter(Event&& event, FSM& fsm)
    {
        root_machine(fsm).template get_state< pushdown_machine_type >().popup(::std::forward<Event>(event));
    }
};

template < typename T >
class state_base : public ::std::conditional<
        def::traits::is_pushdown<T>::value,
        pushdown_state<T>,
        typename ::std::conditional<
            def::traits::is_popup<T>::value,
            popup_state<T>,
            state_base_impl<T, def::traits::is_terminal_state<T>::value>
        >::type
    >::type {
public:
    using state_definition_type = T;
    using state_type            = state_base<T>;
    using internal_transitions  = typename state_definition_type::internal_transitions;
    using base_impl_type        = typename ::std::conditional<
                    def::traits::is_pushdown<T>::value,
                    pushdown_state<T>,
                    typename ::std::conditional<
                        def::traits::is_popup<T>::value,
                        popup_state<T>,
                        state_base_impl<T, def::traits::is_terminal_state<T>::value>
                    >::type
                >::type;
public:
    state_base() : base_impl_type{} {}
    state_base(state_base const&) = default;
    state_base(state_base&&) = default;

    state_base&
    operator = (state_base const&) = delete;
    state_base&
    operator = (state_base&&) = delete;

    void
    swap(state_base& rhs) noexcept
    {
        using ::std::swap;
        static_cast<base_impl_type&>(*this).swap(rhs);
    }

    template < typename StateDef >
    bool
    is_in_state() const
    {
        return ::std::is_same<state_definition_type, StateDef>::value;
    }
protected:
    template< typename ... Args >
    state_base(Args&& ... args)
        : base_impl_type(::std::forward<Args>(args)...) {}
};

template < typename T, typename Mutex, typename FrontMachine >
class state_machine_base_impl : public state_base<T>,
        public transitions::transition_container<FrontMachine, T,
                typename detail::size_type<Mutex>::type> {
public:
    using state_machine_definition_type = T;
    using state_type                    = state_base<T>;
    using machine_type                  = state_machine_base_impl<T, Mutex, FrontMachine>;
    using front_machine_type            = FrontMachine;
    static_assert(def::traits::is_state_machine<T>::value,
            "Front state machine can be created only with a descendant of afsm::def::state_machine");
    using transitions = typename state_machine_definition_type::transitions;
    using handled_events =
            typename def::detail::recursive_handled_events<state_machine_definition_type>::type;
    static_assert(def::traits::is_state<
                typename state_machine_definition_type::initial_state >::value,
            "State machine definition must specify an initial state");
    using initial_state = typename state_machine_definition_type::initial_state;
    using inner_states_def =
            typename def::detail::inner_states< transitions >::type;

    static constexpr ::std::size_t initial_state_index =
            ::psst::meta::index_of<initial_state, inner_states_def>::value;
    static constexpr ::std::size_t inner_state_count = inner_states_def::size;

    using state_indexes     = typename ::psst::meta::index_builder<inner_state_count>::type;

    using mutex_type        = Mutex;
    using size_type         = typename detail::size_type<mutex_type>::type;
    using has_pushdowns     = def::has_pushdown_stack<state_machine_definition_type>;
    using transition_container = afsm::transitions::transition_container_selector<
            front_machine_type, state_machine_definition_type, size_type, has_pushdowns::value>;
    using transition_tuple  = typename transition_container::type;
    using inner_states_tuple = typename transition_tuple::inner_states_tuple;

    using container_base  =
            afsm::transitions::transition_container<
                    front_machine_type, state_machine_definition_type, size_type>;

    template < typename State >
    using substate_type     = typename detail::substate_type<front_machine_type, State>::type;
    template < typename State >
    using contains_substate = def::contains_substate<front_machine_type, State>;
public:
    state_machine_base_impl(front_machine_type* fsm)
        : state_type{},
          container_base{fsm}
    {}

    state_machine_base_impl(front_machine_type* fsm, state_machine_base_impl const& rhs)
        : state_type{static_cast<state_type const&>(rhs)},
          container_base{fsm, rhs}
    {}
    state_machine_base_impl(front_machine_type* fsm, state_machine_base_impl&& rhs)
        : state_type{static_cast<state_type&&>(rhs)},
          container_base{fsm, ::std::move(rhs)}
    {}

    state_machine_base_impl(state_machine_base_impl const& rhs) = delete;
    state_machine_base_impl(state_machine_base_impl&& rhs) = delete;

    void
    swap(state_machine_base_impl& rhs) noexcept
    {
        using ::std::swap;
        static_cast<state_type&>(*this).swap(rhs);
        transitions_.swap(rhs.transitions_);
    }

    state_machine_base_impl&
    operator = (state_machine_base_impl const& rhs) = delete;
    state_machine_base_impl&
    operator = (state_machine_base_impl&& rhs) = delete;

    template < typename StateDef >
    bool
    is_in_state() const
    {
        return is_in_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, inner_states_def>{});
    }

    template < ::std::size_t N>
    typename ::std::tuple_element< N, inner_states_tuple >::type&
    get_state()
    { return transitions_.template get_state<N>(); }
    template < ::std::size_t N>
    typename ::std::tuple_element< N, inner_states_tuple >::type const&
    get_state() const
    { return transitions_.template get_state<N>(); }

    template < typename StateDef >
    typename ::std::enable_if<
        !::std::is_same<state_machine_definition_type, StateDef>::value &&
        contains_substate<StateDef>::value, substate_type<StateDef>>::type&
    get_state()
    {
        return get_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, inner_states_def>{});
    }
    template < typename StateDef >
    typename ::std::enable_if<
        !::std::is_same<state_machine_definition_type, StateDef>::value &&
        contains_substate<StateDef>::value, substate_type<StateDef>>::type const&
    get_state() const
    {
        return get_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, inner_states_def>{});
    }
    template < typename StateDef >
    typename ::std::enable_if<
        ::std::is_same<state_machine_definition_type, StateDef>::value, front_machine_type>::type&
    get_state()
    {
        return static_cast<front_machine_type&>(*this);
    }
    template < typename StateDef >
    typename ::std::enable_if<
        ::std::is_same<state_machine_definition_type, StateDef>::value, front_machine_type>::type const&
    get_state() const
    {
        return static_cast<front_machine_type const&>(*this);
    }

    ::std::size_t
    current_state() const
    { return transitions_.current_state(); }

    template < typename Event, typename FSM >
    void
    state_enter(Event&& event, FSM&)
    {
        transitions_.enter( ::std::forward<Event>(event) );
    }
    template < typename Event, typename FSM >
    void
    state_exit(Event&& event, FSM&)
    {
        transitions_.exit( ::std::forward<Event>(event) );
    }

    event_set
    current_handled_events() const
    {
        auto const& intl = state_type::internal_handled_events();
        auto res = transitions_.current_handled_events();
        res.insert(intl.begin(), intl.end());
        return res;
    }

    event_set
    current_deferrable_events() const
    {
        auto const& own = state_type::current_deferrable_events();
        auto res = transitions_.current_deferrable_events();
        res.insert(own.begin(), own.end());
        return res;
    }
protected:
    template<typename ... Args>
    explicit
    state_machine_base_impl(front_machine_type* fsm, Args&& ... args)
        : state_type(::std::forward<Args>(args)...),
          container_base{fsm}
    {}

    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM& enclosing_fsm, Event&& event,
        detail::process_type<actions::event_process_result::process> const&)
    {
        using event_type = typename ::std::decay<Event>::type;
        using can_handle_in_state = ::std::integral_constant<bool,
                actions::is_in_state_action<state_type, event_type>::value>;
        // Transitions and internal event dispatch
        auto res = transitions_.process_event(::std::forward<Event>(event));
        if (res == actions::event_process_result::refuse) {
        // Internal transitions
            res = process_event_impl(enclosing_fsm,
                ::std::forward<Event>(event), can_handle_in_state{});
        }
        return res;
    }
    template < typename FSM, typename Event >
    constexpr actions::event_process_result
    process_event_impl(FSM&, Event&&,
        detail::process_type<actions::event_process_result::defer> const&) const
    {
        return actions::event_process_result::defer;
    }
    template < typename FSM, typename Event >
    constexpr actions::event_process_result
    process_event_impl(FSM&, Event&&,
        detail::process_type<actions::event_process_result::refuse> const&) const
    {
        return actions::event_process_result::refuse;
    }

    //@{
    /** @name Dispatch of in-state events */
    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM& enclosing_fsm, Event&& event,
        ::std::true_type const& /* Is in-state event */)
    {
        return actions::handle_in_state_event(::std::forward<Event>(event), enclosing_fsm, *this);
    }
    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM&, Event&&,
            ::std::false_type const& /* Is not an in-state event */ )
    {
        return actions::event_process_result::refuse;
    }
    //@}
    //@{
    /** @name Get Substates */
    template < typename StateDef >
    substate_type<StateDef>&
    get_state(state_containment_immediate const&)
    {
        using index_of_state = ::psst::meta::index_of<StateDef, inner_states_def>;
        static_assert(index_of_state::found,
                "Type is not a definition of inner state");
        return transitions_.template get_state< index_of_state::value >();
    }
    template < typename StateDef >
    substate_type<StateDef> const&
    get_state(state_containment_immediate const&) const
    {
        using index_of_state = ::psst::meta::index_of<StateDef, inner_states_def>;
        static_assert(index_of_state::found,
                "Type is not a definition of inner state");
        return transitions_.template get_state< index_of_state::value >();
    }
    template < typename StateDef >
    substate_type<StateDef>&
    get_state(state_containment_substate const&)
    {
        using search = detail::substate_type<front_machine_type, StateDef>;
        return get_state< typename search::front >().template get_state<StateDef>();
    }
    template < typename StateDef >
    substate_type<StateDef> const&
    get_state(state_containment_substate const&) const
    {
        using search = detail::substate_type<front_machine_type, StateDef>;
        return get_state< typename search::front >().template get_state<StateDef>();
    }
    //@}
    //@{
    /** @name Is in state check */
    /**
     * Constant false for states not contained by this state machine
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_none const&) const
    { return false; }
    /**
     * Constant true for self
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_self const&) const
    { return true; }
    /**
     * Is in substate of an inner state
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_substate const&) const
    {
        using immediate_states =
            typename ::psst::meta::find_if<
                def::contains_substate_predicate<StateDef>::template type,
                inner_states_def >::type;
        return is_in_substate<StateDef>(immediate_states{});
    }

    template < typename StateDef, typename ... ImmediateSubStates >
    bool
    is_in_substate( ::psst::meta::type_tuple<ImmediateSubStates...> const& ) const
    {
        return ::psst::meta::any_of({
                (this->template is_in_state<ImmediateSubStates>() &&
                 this->template get_state< ImmediateSubStates >().
                            template is_in_state<StateDef>()) ...
        });
    }
    /**
     * In in an immediately inner state
     * @param Template tag switch
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_immediate const&) const
    {
        using index_of_state = ::psst::meta::index_of<StateDef, inner_states_def>;
        static_assert(index_of_state::found,
                    "Type is not a definition of inner state");
        return index_of_state::value == transitions_.current_state();
    }
    //@}
protected:
    using container_base::transitions_;
};

template < typename T, typename Mutex, typename FrontMachine >
constexpr ::std::size_t state_machine_base_impl<T, Mutex, FrontMachine>::initial_state_index;
template < typename T, typename Mutex, typename FrontMachine >
constexpr ::std::size_t state_machine_base_impl<T, Mutex, FrontMachine>::inner_state_count;

template < typename T, typename Mutex, typename FrontMachine >
struct state_machine_base_with_base : state_machine_base_impl<T, Mutex, FrontMachine> {
    using base_type = state_machine_base_impl<T, Mutex, FrontMachine>;
    using common_base = typename T::common_base;

    state_machine_base_with_base(FrontMachine* fsm)
        : base_type{fsm}
    {
    }
    state_machine_base_with_base(FrontMachine* fsm, state_machine_base_with_base const& rhs)
        : base_type{fsm, rhs}
    {
    }
    state_machine_base_with_base(FrontMachine* fsm, state_machine_base_with_base&& rhs)
        : base_type{fsm, ::std::move(rhs)}
    {
    }

    state_machine_base_with_base(state_machine_base_with_base const&) = delete;
    state_machine_base_with_base(state_machine_base_with_base&&) = delete;
    state_machine_base_with_base&
    operator = (state_machine_base_with_base const&) = delete;
    state_machine_base_with_base&
    operator = (state_machine_base_with_base&&) = delete;

    common_base&
    current_state_base()
    {
        return base_type::transitions_.template cast_current_state<common_base>();
    }
    common_base const&
    current_state_base() const
    {
        return base_type::transitions_.template cast_current_state<common_base const>();
    }
protected:
    template<typename ... Args>
    explicit
    state_machine_base_with_base(FrontMachine* fsm, Args&& ... args)
        : state_machine_base_with_base::machine_type(fsm, ::std::forward<Args>(args)...)
    {}
};

template < typename T, typename Mutex, typename FrontMachine >
class orthogonal_state_machine : public state_base<T>,
        public orthogonal::region_container<FrontMachine, T,
                typename detail::size_type<Mutex>::type> {
public:
    using state_machine_definition_type = T;
    using state_type                    = state_base<T>;
    using machine_type                  = orthogonal_state_machine<T, Mutex, FrontMachine>;
    using front_machine_type            = FrontMachine;
    static_assert(def::traits::is_state_machine<T>::value,
            "Front state machine can be created only with a descendant of afsm::def::state_machine");
    using orthogonal_regions            = typename state_machine_definition_type::orthogonal_regions;
    using handled_events                = typename def::detail::recursive_handled_events< orthogonal_regions >::type;
    using regions_def                   = typename def::detail::inner_states< orthogonal_regions >::type;

    static constexpr ::std::size_t region_count = regions_def::size;

    using mutex_type                    = Mutex;
    using size_type                     = typename detail::size_type<mutex_type>::type;

    using container_base    =
            orthogonal::region_container<front_machine_type, state_machine_definition_type, size_type>;
    using region_tuple                  = typename container_base::region_tuple;

    template < typename State >
    using substate_type     = typename detail::substate_type<front_machine_type, State>::type;
    template < typename State >
    using contains_substate = def::contains_substate<front_machine_type, State>;
public:
    orthogonal_state_machine(front_machine_type* fsm)
        : state_type{},
          container_base{fsm}
    {}
    orthogonal_state_machine(front_machine_type* fsm, orthogonal_state_machine const& rhs)
        : state_type{static_cast<state_type const&>(rhs)},
          container_base{fsm, rhs}
    {}
    orthogonal_state_machine(front_machine_type* fsm, orthogonal_state_machine&& rhs)
        : state_type{static_cast<state_type&&>(rhs)},
          container_base{fsm, ::std::move(rhs)}
    {}

    orthogonal_state_machine(orthogonal_state_machine const& rhs) = delete;
    orthogonal_state_machine(orthogonal_state_machine&& rhs) = delete;

    void
    swap(orthogonal_state_machine& rhs) noexcept
    {
        using ::std::swap;
        static_cast<state_type&>(*this).swap(rhs);
        regions_.swap(rhs.regions_);
    }

    orthogonal_state_machine&
    operator = (orthogonal_state_machine const& rhs) = delete;
    orthogonal_state_machine&
    operator = (orthogonal_state_machine&& rhs) = delete;

    template < typename StateDef >
    bool
    is_in_state() const
    {
        return is_in_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, regions_def>{});
    }

    template < ::std::size_t N>
    typename ::std::tuple_element< N, region_tuple >::type&
    get_state()
    { return regions_.template get_state<N>(); }
    template < ::std::size_t N>
    typename ::std::tuple_element< N, region_tuple >::type const&
    get_state() const
    { return regions_.template get_state<N>(); }

    template < typename StateDef >
    typename ::std::enable_if<
        !::std::is_same<state_machine_definition_type, StateDef>::value &&
        contains_substate<StateDef>::value, substate_type<StateDef>>::type&
    get_state()
    {
        return get_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, regions_def>{});
    }
    template < typename StateDef >
    typename ::std::enable_if<
        !::std::is_same<state_machine_definition_type, StateDef>::value &&
        contains_substate<StateDef>::value, substate_type<StateDef>>::type const&
    get_state() const
    {
        return get_state<StateDef>(state_containment_type<StateDef, state_machine_definition_type, regions_def>{});
    }
    template < typename StateDef >
    typename ::std::enable_if<
        ::std::is_same<state_machine_definition_type, StateDef>::value, front_machine_type>::type&
    get_state()
    {
        return static_cast<front_machine_type&>(*this);
    }
    template < typename StateDef >
    typename ::std::enable_if<
        ::std::is_same<state_machine_definition_type, StateDef>::value, front_machine_type>::type const&
    get_state() const
    {
        return static_cast<front_machine_type const&>(*this);
    }

    template < typename Event, typename FSM >
    void
    state_enter(Event&& event, FSM&)
    {
        regions_.enter(::std::forward<Event>(event));
    }
    template < typename Event, typename FSM >
    void
    state_exit(Event&& event, FSM&)
    {
        regions_.exit(::std::forward<Event>(event));
    }
    event_set
    current_handled_events() const
    {
        auto const& intl = state_type::internal_handled_events();
        event_set res = regions_.current_handled_events();
        res.insert(intl.begin(), intl.end());
        return res;
    }
    event_set
    current_deferrable_events() const
    {
        auto const& own = state_type::current_deferrable_events();
        auto res = regions_.current_deferrable_events();
        res.insert(own.begin(), own.end());
        return res;
    }
protected:
    template < typename ... Args >
    explicit
    orthogonal_state_machine(front_machine_type* fsm, Args&& ... args)
        : state_type(::std::forward<Args>(args)...),
          container_base{fsm}
    {}

    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM& enclosing_fsm, Event&& event,
            detail::process_type< actions::event_process_result::process > const&)
    {
        using event_type = typename ::std::decay<Event>::type;
        using can_handle_in_state = ::std::integral_constant<bool,
                actions::is_in_state_action<state_type, event_type>::value>;
        auto res = regions_.process_event(::std::forward<Event>(event));
        if (res == actions::event_process_result::refuse) {
        // Internal transitions
            res = process_event_impl(enclosing_fsm,
                ::std::forward<Event>(event), can_handle_in_state{});
        }
        return res;
    }
    template < typename FSM, typename Event >
    constexpr actions::event_process_result
    process_event_impl(FSM&, Event&&,
        detail::process_type<actions::event_process_result::defer> const&) const
    {
        return actions::event_process_result::defer;
    }
    template < typename FSM, typename Event >
    constexpr actions::event_process_result
    process_event_impl(FSM&, Event&&,
        detail::process_type<actions::event_process_result::refuse> const&) const
    {
        return actions::event_process_result::refuse;
    }
    //@{
    /** @name Dispatch of in-state events */
    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM& enclosing_fsm, Event&& event,
        ::std::true_type const& /* Is in-state event */)
    {
        return actions::handle_in_state_event(::std::forward<Event>(event), enclosing_fsm, *this);
    }
    template < typename FSM, typename Event >
    actions::event_process_result
    process_event_impl(FSM&, Event&&,
            ::std::false_type const& /* Is not an in-state event */ )
    {
        return actions::event_process_result::refuse;
    }
    //@}
    //@{
    /** @name Get Substates */
    template < typename StateDef >
    substate_type<StateDef>&
    get_state(state_containment_immediate const&)
    {
        using index_of_state = ::psst::meta::index_of<StateDef, regions_def>;
        static_assert(index_of_state::found,
                "Type is not a definition of inner state");
        return regions_.template get_state< index_of_state::value >();
    }
    template < typename StateDef >
    substate_type<StateDef> const&
    get_state(state_containment_immediate const&) const
    {
        using index_of_state = ::psst::meta::index_of<StateDef, regions_def>;
        static_assert(index_of_state::found,
                "Type is not a definition of inner state");
        return regions_.template get_state< index_of_state::value >();
    }
    template < typename StateDef >
    substate_type<StateDef>&
    get_state(state_containment_substate const&)
    {
        using search = detail::substate_type<front_machine_type, StateDef>;
        return get_state< typename search::front >().template get_state<StateDef>();
    }
    template < typename StateDef >
    substate_type<StateDef> const&
    get_state(state_containment_substate const&) const
    {
        using search = detail::substate_type<front_machine_type, StateDef>;
        return get_state< typename search::front >().template get_state<StateDef>();
    }
    //@}
    //@{
    /** @name Is in state checks */
    /**
     * Constant false for states not contained by this state machine
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_none const&) const
    { return false; }
    /**
     * Constant true for self
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_self const&) const
    { return true; }
    /**
     * Is in substate of an inner state
     * @param
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_substate const&) const
    {
        using immediate_states =
            typename ::psst::meta::find_if<
                def::contains_substate_predicate<StateDef>::template type,
                regions_def >::type;
        return is_in_substate<StateDef>(immediate_states{});
    }

    template < typename StateDef, typename ... ImmediateSubStates >
    bool
    is_in_substate( ::psst::meta::type_tuple<ImmediateSubStates...> const& ) const
    {
        return ::psst::meta::any_of({
                (this->template is_in_state<ImmediateSubStates>() &&
                 this->template get_state< ImmediateSubStates >().
                            template is_in_state<StateDef>()) ...
        });
    }
    /**
     * In in an immediately inner state. All immediate states are regions
     * and are always active.
     * @param Template tag switch
     * @return
     */
    template < typename StateDef >
    bool
    is_in_state(state_containment_immediate const&) const
    {
        return true;
    }
    //@}
protected:
    using container_base::regions_;
};

template < typename T, typename Mutex, typename FrontMachine >
constexpr ::std::size_t orthogonal_state_machine<T, Mutex, FrontMachine>::region_count;

template < typename T, typename Mutex, typename FrontMachine >
struct state_machine_base : ::std::conditional<
        def::traits::has_orthogonal_regions<T>::value,
        orthogonal_state_machine<T, Mutex, FrontMachine>, // TODO Check common base
        typename ::std::conditional<
            def::traits::has_common_base<T>::value,
            state_machine_base_with_base< T, Mutex, FrontMachine >,
            state_machine_base_impl< T, Mutex, FrontMachine >
        >::type
    >::type {
public:
    using state_machine_impl_type = typename ::std::conditional<
            def::traits::has_orthogonal_regions<T>::value,
            orthogonal_state_machine<T, Mutex, FrontMachine>,
            typename ::std::conditional<
                def::traits::has_common_base<T>::value,
                state_machine_base_with_base< T, Mutex, FrontMachine >,
                state_machine_base_impl< T, Mutex, FrontMachine >
            >::type
        >::type;
    state_machine_base(FrontMachine* fsm)
        : state_machine_impl_type{fsm} {}
    state_machine_base(FrontMachine* fsm, state_machine_base const& rhs)
        : state_machine_impl_type{fsm, rhs} {}
    state_machine_base(FrontMachine* fsm, state_machine_base&& rhs)
        : state_machine_impl_type{fsm, ::std::move(rhs)} {}

    state_machine_base(state_machine_base const&) = delete;
    state_machine_base(state_machine_base&&) = delete;
    state_machine_base&
    operator = (state_machine_base const&) = delete;
    state_machine_base&
    operator = (state_machine_base&&) = delete;
protected:
    template<typename ... Args>
    explicit
    state_machine_base(FrontMachine* fsm, Args&& ... args)
        : state_machine_impl_type(fsm, ::std::forward<Args>(args)...) {}
};

}  /* namespace detail */
}  /* namespace afsm */



#endif /* AFSM_DETAIL_BASE_STATES_HPP_ */
