/*
 * orthogonal_regions.hpp
 *
 *  Created on: 27 нояб. 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_ORTHOGONAL_REGIONS_HPP_
#define AFSM_DETAIL_ORTHOGONAL_REGIONS_HPP_

#include <afsm/detail/actions.hpp>
#include <afsm/detail/transitions.hpp>

namespace afsm {
namespace orthogonal {

namespace detail {

template < ::std::size_t N >
struct invoke_nth {
    using previous = invoke_nth<N - 1>;
    static constexpr ::std::size_t index = N;
    using event_handler_type = actions::detail::process_event_handler<index>;
    using event_set          = ::afsm::detail::event_set;

    template < typename Regions, typename Event, typename FSM >
    static void
    enter(Regions& regions, Event&& event, FSM& fsm)
    {
        using state_type = typename ::std::tuple_element<index, Regions>::type;
        using state_enter = transitions::detail::state_enter<FSM, state_type, Event>;

        previous::enter(regions, ::std::forward<Event>(event), fsm);
        state_enter{}(::std::get<index>(regions), ::std::forward<Event>(event), fsm);
    }

    template < typename Regions, typename Event, typename FSM >
    static void
    exit(Regions& regions, Event&& event, FSM& fsm)
    {
        using state_type = typename ::std::tuple_element<index, Regions>::type;
        using state_exit = transitions::detail::state_exit<FSM, state_type, Event>;

        // Reverse order of exit
        state_exit{}(::std::get<index>(regions), ::std::forward<Event>(event), fsm);
        previous::exit(regions, ::std::forward<Event>(event), fsm);
    }

    template < typename Regions, typename Event >
    static actions::event_process_result
    process_event(Regions& regions, Event&& event)
    {
        auto res = previous::process_event(regions, ::std::forward<Event>(event));
        return ::std::max(res, event_handler_type{}(regions, ::std::forward<Event>(event)));
    }

    template < typename Regions >
    static void
    collect_events( Regions const& regions, event_set& events )
    {
        previous::collect_events(regions, events);
        auto const& region = ::std::get<index>(regions);
        event_set evts = region.current_handled_events();
        events.insert(evts.begin(), evts.end());
    }
    template < typename Regions >
    static void
    collect_deferred_events( Regions const& regions, event_set& events )
    {
        previous::collect_deferred_events(regions, events);
        auto const& region = ::std::get<index>(regions);
        event_set evts = region.current_deferrable_events();
        events.insert(evts.begin(), evts.end());
    }
};

template <>
struct invoke_nth< 0 > {
    static constexpr ::std::size_t index = 0;
    using event_handler_type = actions::detail::process_event_handler<index>;
    using event_set          = ::afsm::detail::event_set;

    template < typename Regions, typename Event, typename FSM >
    static void
    enter(Regions& regions, Event&& event, FSM& fsm)
    {
        using state_type = typename ::std::tuple_element<index, Regions>::type;
        using state_enter = transitions::detail::state_enter<FSM, state_type, Event>;
        state_enter{}(::std::get<index>(regions), ::std::forward<Event>(event), fsm);
    }

    template < typename Regions, typename Event, typename FSM >
    static void
    exit(Regions& regions, Event&& event, FSM& fsm)
    {
        using state_type = typename ::std::tuple_element<index, Regions>::type;
        using state_exit = transitions::detail::state_exit<FSM, state_type, Event>;
        state_exit{}(::std::get<index>(regions), ::std::forward<Event>(event), fsm);
    }

    template < typename Regions, typename Event >
    static actions::event_process_result
    process_event(Regions& regions, Event&& event)
    {
        return event_handler_type{}(regions, ::std::forward<Event>(event));
    }

    template < typename Regions >
    static void
    collect_events( Regions const& regions, event_set& events )
    {
        auto const& region = ::std::get<index>(regions);
        region.current_handled_events().swap(events);
    }
    template < typename Regions >
    static void
    collect_deferred_events( Regions const& regions, event_set& events )
    {
        auto const& region = ::std::get<index>(regions);
        region.current_deferrable_events().swap(events);
    }
};

}  /* namespace detail */

template < typename FSM, typename FSM_DEF, typename Size >
class regions_table {
public:
    using fsm_type                      = FSM;
    using state_machine_definition_type = FSM_DEF;
    using size_type                     = Size;

    using this_type                     = regions_table<FSM, FSM_DEF, Size>;

    using orthogonal_regions            = typename state_machine_definition_type::orthogonal_regions;
    static_assert(!::std::is_same<orthogonal_regions, void>::value,
            "Orthogonal regions table is not defined for orthogonal state machine");

    using regions_def                   = typename def::detail::inner_states< orthogonal_regions >::type;
    using regions_constructor           = ::afsm::detail::front_state_tuple<fsm_type, regions_def>;
    using regions_tuple                 = typename regions_constructor::type;

    static constexpr ::std::size_t size = regions_def::size;

    using region_indexes                = typename ::psst::meta::index_builder<size>::type;
    using all_regions                   = detail::invoke_nth<size - 1>;
    using event_set                     = ::afsm::detail::event_set;
public:
    regions_table(fsm_type& fsm)
        : fsm_{&fsm},
          regions_{ regions_constructor::construct(fsm) }
    {}
    regions_table(fsm_type& fsm, regions_table const& rhs)
        : fsm_{&fsm},
          regions_{ regions_constructor::copy_construct(fsm, rhs.regions_) }
    {}
    regions_table(fsm_type& fsm, regions_table&& rhs)
        : fsm_{&fsm},
          regions_{ regions_constructor::move_construct(fsm, ::std::move(rhs.regions_)) }
    {}

    regions_table(regions_table const&) = delete;
    regions_table(regions_table&& rhs)
        : fsm_{rhs.fsm_},
          regions_{::std::move(rhs.regions_)}
    {
    }
    regions_table&
    operator = (regions_table const&) = delete;
    regions_table&
    operator = (regions_table&&) = delete;

    void
    swap(regions_table& rhs)
    {
        using ::std::swap;
        swap(regions_, rhs.regions_);
        set_fsm(*fsm_);
        rhs.set_fsm(*rhs.fsm_);
    }

    void
    set_fsm(fsm_type& fsm)
    {
        fsm_ = &fsm;
        ::afsm::detail::set_enclosing_fsm<size - 1>::set(fsm, regions_);
    }

    regions_tuple&
    regions()
    { return regions_; }
    regions_tuple const&
    regions() const
    { return regions_; }

    template < ::std::size_t N>
    typename ::std::tuple_element< N, regions_tuple >::type&
    get_state()
    { return ::std::get<N>(regions_); }
    template < ::std::size_t N>
    typename ::std::tuple_element< N, regions_tuple >::type const&
    get_state() const
    { return ::std::get<N>(regions_); }

    template < typename Event >
    actions::event_process_result
    process_event(Event&& event)
    {
        // Pass event to all regions
        return all_regions::process_event(regions_, ::std::forward<Event>(event));
    }
    event_set
    current_handled_events() const
    {
        event_set evts;
        all_regions::collect_events(regions_, evts);
        return evts;
    }
    event_set
    current_deferrable_events() const
    {
        event_set evts;
        all_regions::collect_deferred_events(regions_, evts);
        return evts;
    }

    template < typename Event >
    void
    enter(Event&& event)
    {
        // Call enter for all regions
        all_regions::enter(regions_, ::std::forward<Event>(event), *fsm_);
    }
    template < typename Event >
    void
    exit(Event&& event)
    {
        // Call exit for all regions
        all_regions::exit(regions_, ::std::forward<Event>(event), *fsm_);
    }
private:
    fsm_type*           fsm_;
    regions_tuple       regions_;
};

template < typename FSM, typename FSM_DEF, typename Size >
class regions_stack {
public:
    using region_table_type     = regions_table<FSM, FSM_DEF, Size>;
    using this_type             = regions_stack<FSM, FSM_DEF, Size>;

    using fsm_type                      = typename region_table_type::fsm_type;
    using state_machine_definition_type = typename region_table_type::state_machine_definition_type;
    using size_type                     = typename region_table_type::size_type;
    using regions_tuple                 = typename region_table_type::regions_tuple;

    using stack_constructor_type        = afsm::detail::stack_constructor<FSM, region_table_type>;
    using event_set                     = ::afsm::detail::event_set;
public:
    regions_stack(fsm_type& fsm)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::construct(fsm) }
    {}
    regions_stack(fsm_type& fsm, regions_stack const& rhs)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::copy_construct(fsm, rhs.state_stack_) }
    {}
    regions_stack(fsm_type& fsm, regions_stack&& rhs)
        : fsm_{&fsm},
          state_stack_{ stack_constructor_type::move_construct(fsm, ::std::move(rhs.state_stack_)) }
    {}

    regions_stack(regions_stack const&) = delete;
    regions_stack(regions_stack&&) = delete;
    regions_stack&
    operator = (regions_stack const&) = delete;
    regions_stack&
    operator = (regions_stack&&) = delete;

    void
    swap(regions_stack& rhs)
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

    regions_tuple&
    states()
    { return top().states(); }
    regions_tuple const&
    states() const
    { return top().states(); }

    template < ::std::size_t N >
    typename ::std::tuple_element< N, regions_tuple >::type&
    get_state()
    { return top().template get_state<N>(); }
    template < ::std::size_t N >
    typename ::std::tuple_element< N, regions_tuple >::type const&
    get_state() const
    { return top().template get_state<N>(); }

    template < typename Event >
    actions::event_process_result
    process_event(Event&& event)
    {
        return top().process_event(::std::forward<Event>(event));
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
private:
    using stack_type                    = typename stack_constructor_type::type;

    region_table_type&
    top()
    { return state_stack_.back(); }
    region_table_type const&
    top() const
    { return state_stack_.back(); }
private:
    fsm_type        *fsm_;
    stack_type      state_stack_;
};

template < typename FSM, typename FSM_DEF, typename Size, bool HasPushdowns >
struct region_container_selector {
    using type = regions_table<FSM, FSM_DEF, Size>;
};

template < typename FSM, typename FSM_DEF, typename Size >
struct region_container_selector<FSM, FSM_DEF, Size, true> {
    using type = regions_stack<FSM, FSM_DEF, Size>;
};

namespace detail {

template < typename FSM, typename FSM_DEF, typename Size, bool HasPushdowns >
struct region_container {
    using region_tuple = typename region_container_selector<FSM, FSM_DEF, Size, HasPushdowns>::type;

    region_container(FSM* fsm)
        : regions_{*fsm}
    {}
    region_container(FSM* fsm, region_container const& rhs)
        : regions_{*fsm, rhs.regions_}
    {}
    region_container(FSM* fsm, region_container&& rhs)
        : regions_{*fsm, ::std::move(rhs.regions_)}
    {}
protected:
    region_tuple regions_;
};

template < typename FSM, typename FSM_DEF, typename Size >
struct region_container<FSM, FSM_DEF, Size, true> {
    using region_tuple = typename region_container_selector<FSM, FSM_DEF, Size, true>::type;

    region_container(FSM* fsm)
        : regions_{*fsm}
    {}
    region_container(FSM* fsm, region_container const& rhs)
        : regions_{*fsm, rhs.regions_}
    {}
    region_container(FSM* fsm, region_container&& rhs)
        : regions_{*fsm, ::std::move(rhs.regions_)}
    {}

    ::std::size_t
    stack_size() const
    { return regions_.stack_size(); }

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
        regions_.push(::std::forward<Event>(event));
    }
    template < typename Event >
    void
    popup(Event&& event)
    {
        regions_.pop(::std::forward<Event>(event));
    }
protected:
    region_tuple regions_;
};

}  /* namespace detail */

template < typename FSM, typename FSM_DEF, typename Size >
struct region_container
    : detail::region_container<FSM, FSM_DEF, Size,
          def::has_pushdown_stack<FSM_DEF>::value> {
    using base_type = detail::region_container<FSM, FSM_DEF, Size,
            def::has_pushdown_stack<FSM_DEF>::value>;
    using region_tuple = typename base_type::region_tuple;

    region_container(FSM* fsm) : base_type{fsm} {}
    region_container(FSM* fsm, region_container const& rhs)
        : base_type{fsm, rhs} {}
    region_container(FSM* fsm, region_container&& rhs)
        : base_type{fsm, ::std::move(rhs)} {}
protected:
    using base_type::regions_;
};

}  /* namespace orthogonal */
}  /* namespace afsm */



#endif /* AFSM_DETAIL_ORTHOGONAL_REGIONS_HPP_ */
