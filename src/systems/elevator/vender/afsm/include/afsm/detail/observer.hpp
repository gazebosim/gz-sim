/*
 * observer.hpp
 *
 *  Created on: Jun 3, 2016
 *      Author: zmij
 */

#ifndef AFSM_DETAIL_OBSERVER_HPP_
#define AFSM_DETAIL_OBSERVER_HPP_

#include <afsm/fsm_fwd.hpp>
#include <afsm/detail/actions.hpp>
#include <afsm/detail/transitions.hpp>
#include <memory>

namespace afsm {
namespace detail {

struct null_observer {
    template < typename FSM, typename Event >
    void
    start_process_event(FSM const&, Event const&) const noexcept {}

    template < typename FSM, typename State, typename Event >
    void
    state_entered(FSM const&, State const&, Event const&) const noexcept{}
    template < typename FSM, typename State, typename Event >
    void
    state_exited(FSM const&, State const&, Event const&) const noexcept{}
    template < typename FSM, typename State >
    void
    state_cleared(FSM const&, State const&) const noexcept{}

    template < typename FSM, typename SourceState, typename TargetState, typename Event>
    void
    state_changed(FSM const&, SourceState const&, TargetState const&, Event const&) const noexcept {}

    template < typename FSM, typename Event >
    void
    processed_in_state(FSM const&, Event const&) const noexcept {}

    template < typename FSM, typename Event >
    void
    enqueue_event(FSM const&, Event const&) const noexcept {}

    template < typename FSM >
    void
    start_process_events_queue(FSM const&) const noexcept {}
    template < typename FSM >
    void
    end_process_events_queue(FSM const&) const noexcept {}

    template < typename FSM, typename Event >
    void
    defer_event(FSM const&, Event const&) const noexcept {}

    template < typename FSM >
    void
    start_process_deferred_queue(FSM const&, ::std::size_t /*size*/) const noexcept {}
    template < typename FSM >
    void
    end_process_deferred_queue(FSM const&, ::std::size_t /*remain*/) const noexcept {}

    template < typename FSM >
    void
    skip_processing_deferred_queue(FSM const&) const noexcept {}
    template < typename FSM >
    void
    postpone_deferred_events(FSM const&, ::std::size_t /*count*/) const noexcept {}
    template < typename FSM >
    void
    drop_deferred_event(FSM const&) const noexcept{}

    template < typename FSM, typename Event >
    void
    reject_event(FSM const&, Event const&) const noexcept {}
};

template < typename T >
class observer_wrapper {
public:
    using observer_ptr = ::std::shared_ptr<T>;
public:
    observer_wrapper()
        : observer_{} {}
    void
    set_observer(observer_ptr observer)
    {
        observer_ = observer;
    }

    template < typename ... Args >
    void
    make_observer(Args&& ... args)
    {
        observer_ = ::std::make_shared<T>(::std::forward<Args>(args)...);
    }
protected:
    template < typename FSM, typename FSM_DEF, typename Size >
    friend class transitions::state_transition_table;

    template < typename FSM, typename Event >
    void
    start_process_event(FSM const& fsm, Event const& event) const noexcept
    {
        if (observer_)
            observer_->start_process_event(fsm, event);
    }

    template < typename FSM, typename State, typename Event >
    void
    state_entered(FSM const& fsm, State const& state, Event const& event) const noexcept
    {
        if (observer_)
            observer_->state_entered(fsm, state, event);
    }
    template < typename FSM, typename State, typename Event >
    void
    state_exited(FSM const& fsm, State const& state, Event const& event) const noexcept
    {
        if (observer_)
            observer_->state_exited(fsm, state, event);
    }
    template < typename FSM, typename State >
    void
    state_cleared(FSM const& fsm, State const& state) const noexcept
    {
        if (observer_)
            observer_->state_cleared(fsm, state);
    }
    template < typename FSM, typename SourceState, typename TargetState, typename Event>
    void
    state_changed(FSM const& fsm, SourceState const& source,
            TargetState const& target, Event const& event) const noexcept
    {
        if (observer_)
            observer_->state_changed(fsm, source, target, event);
    }

    template < typename FSM, typename Event >
    void
    processed_in_state(FSM const& fsm, Event const& event) const noexcept
    {
        if (observer_)
            observer_->processed_in_state(fsm, event);
    }

    template < typename FSM, typename Event >
    void
    enqueue_event(FSM const& fsm, Event const& event) const noexcept
    {
        if (observer_)
            observer_->enqueue_event(fsm, event);
    }

    template < typename FSM >
    void
    start_process_events_queue(FSM const& fsm) const noexcept
    {
        if (observer_)
            observer_->start_process_events_queue(fsm);
    }

    template < typename FSM >
    void
    end_process_events_queue(FSM const& fsm) const noexcept
    {
        if (observer_)
            observer_->end_process_events_queue(fsm);
    }

    template < typename FSM, typename Event >
    void
    defer_event(FSM const& fsm, Event const& event) const noexcept
    {
        if (observer_)
            observer_->defer_event(fsm, event);
    }

    template < typename FSM >
    void
    start_process_deferred_queue(FSM const& fsm, ::std::size_t size) const noexcept
    {
        if (observer_)
            observer_->start_process_deferred_queue(fsm, size);
    }

    template < typename FSM >
    void
    end_process_deferred_queue(FSM const& fsm, ::std::size_t remain) const noexcept
    {
        if (observer_)
            observer_->end_process_deferred_queue(fsm, remain);
    }

    template < typename FSM >
    void
    skip_processing_deferred_queue(FSM const& fsm) const noexcept
    {
        if (observer_)
            observer_->skip_processing_deferred_queue(fsm);
    }
    template < typename FSM >
    void
    postpone_deferred_events(FSM const& fsm, ::std::size_t count) const noexcept
    {
        if (observer_)
            observer_->postpone_deferred_events(fsm, count);
    }
    template < typename FSM >
    void
    drop_deferred_event(FSM const& fsm) const noexcept
    {
        if (observer_)
            observer_->drop_deferred_event(fsm);
    }

    template < typename FSM, typename Event >
    void
    reject_event(FSM const& fsm, Event const& event) const noexcept
    {
        if (observer_)
            observer_->reject_event(fsm, event);
    }
private:
    observer_ptr    observer_;
};

template <>
struct observer_wrapper<none> : null_observer {
};

}  /* namespace detail */
}  /* namespace afsm */



#endif /* AFSM_DETAIL_OBSERVER_HPP_ */
