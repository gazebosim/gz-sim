/*
 * fsm.hpp
 *
 *  Created on: 25 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_FSM_HPP_
#define AFSM_FSM_HPP_

#include <afsm/detail/base_states.hpp>
#include <afsm/detail/observer.hpp>
#include <afsm/detail/reject_policies.hpp>
#include <afsm/detail/event_identity.hpp>
#include <deque>
#include <queue>
#include <list>

namespace afsm {

//----------------------------------------------------------------------------
//  State
//----------------------------------------------------------------------------
template < typename T, typename FSM >
class state : public detail::state_base< T > {
public:
    using enclosing_fsm_type    = FSM;
public:
    state(enclosing_fsm_type& fsm)
        : state::state_type{}, fsm_{&fsm}
    {}
    state(state const& rhs) = default;
    state(state&& rhs) = default;
    state(enclosing_fsm_type& fsm, state const& rhs)
        : state::state_type{static_cast<typename state::state_type const&>(rhs)}, fsm_{&fsm}
    {}
    state(enclosing_fsm_type& fsm, state&& rhs)
        : state::state_type{static_cast<typename state::state_type&&>(rhs)}, fsm_{&fsm}
    {}

    state&
    operator = (state const& rhs)
    {
        state{rhs}.swap(*this);
        return *this;
    }
    state&
    operator = (state&& rhs)
    {
        swap(rhs);
        return *this;
    }

    void
    swap(state& rhs) noexcept
    {
        static_cast<typename state::state_type&>(*this).swap(rhs);
    }

    enclosing_fsm_type&
    enclosing_fsm()
    { return *fsm_; }
    enclosing_fsm_type const&
    enclosing_fsm() const
    { return *fsm_; }
    void
    enclosing_fsm(enclosing_fsm_type& fsm)
    { fsm_ = &fsm; }
protected: // For tests
    template < ::std::size_t StateIndex >
    friend struct actions::detail::process_event_handler;

    template < typename Event >
    actions::event_process_result
    process_event( Event&& evt )
    {
        return process_event_impl(::std::forward<Event>(evt),
                detail::event_process_selector<
                    Event,
                    typename state::internal_events,
                    typename state::deferred_events>{} );
    }
private:
    template < typename Event >
    actions::event_process_result
    process_event_impl(Event&& evt,
        detail::process_type<actions::event_process_result::process> const&)
    {
        return actions::handle_in_state_event(::std::forward<Event>(evt), *fsm_, *this);
    }
    template < typename Event >
    constexpr actions::event_process_result
    process_event_impl(Event&&,
        detail::process_type<actions::event_process_result::defer> const&) const
    {
        return actions::event_process_result::defer;
    }
    template < typename Event >
    constexpr actions::event_process_result
    process_event_impl(Event&&,
        detail::process_type<actions::event_process_result::refuse> const&) const
    {
        return actions::event_process_result::refuse;
    }
private:
    enclosing_fsm_type*    fsm_;
};

//----------------------------------------------------------------------------
//  Inner state machine
//----------------------------------------------------------------------------
template < typename T, typename FSM >
class inner_state_machine : public detail::state_machine_base< T, none, inner_state_machine<T, FSM> > {
public:
    using enclosing_fsm_type    = FSM;
    using this_type = inner_state_machine<T, FSM>;
    using base_machine_type     = detail::state_machine_base< T, none, this_type >;
public:
    inner_state_machine(enclosing_fsm_type& fsm)
        : base_machine_type{this}, fsm_{&fsm} {}
    inner_state_machine(inner_state_machine const& rhs)
        : base_machine_type{this, rhs}, fsm_{rhs.fsm_} {}
    inner_state_machine(inner_state_machine&& rhs)
        : base_machine_type{this, ::std::move(rhs)},
          fsm_{rhs.fsm_}
    {
    }

    inner_state_machine(enclosing_fsm_type& fsm, inner_state_machine const& rhs)
        : base_machine_type{static_cast<base_machine_type const&>(rhs)}, fsm_{&fsm} {}
    inner_state_machine(enclosing_fsm_type& fsm, inner_state_machine&& rhs)
        : base_machine_type{static_cast<base_machine_type&&>(rhs)}, fsm_{&fsm} {}

    void
    swap(inner_state_machine& rhs) noexcept
    {
        using ::std::swap;
        static_cast<base_machine_type&>(*this).swap(rhs);
        swap(fsm_, rhs.fsm_);
    }
    inner_state_machine&
    operator = (inner_state_machine const& rhs)
    {
        inner_state_machine tmp{rhs};
        swap(tmp);
        return *this;
    }
    inner_state_machine&
    operator = (inner_state_machine&& rhs)
    {
        swap(rhs);
        return *this;
    }

    enclosing_fsm_type&
    enclosing_fsm()
    { return *fsm_; }
    enclosing_fsm_type const&
    enclosing_fsm() const
    { return *fsm_; }
    void
    enclosing_fsm(enclosing_fsm_type& fsm)
    { fsm_ = &fsm; }
protected: // For tests
    template < ::std::size_t StateIndex >
    friend struct actions::detail::process_event_handler;

    template < typename Event >
    actions::event_process_result
    process_event( Event&& event )
    {
        return process_event_impl(*fsm_, ::std::forward<Event>(event),
                detail::event_process_selector<
                    Event,
                    typename inner_state_machine::handled_events,
                    typename inner_state_machine::deferred_events>{} );
    }
private:
    using base_machine_type::process_event_impl;
private:
    enclosing_fsm_type*    fsm_;
};

//----------------------------------------------------------------------------
//  State machine
//----------------------------------------------------------------------------
template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
class state_machine :
        public detail::state_machine_base< T, Mutex,
            state_machine<T, Mutex, Observer, ObserverWrapper> >,
        public ObserverWrapper<Observer> {
public:
    static_assert( ::psst::meta::is_empty< typename T::deferred_events >::value,
            "Outer state machine cannot defer events" );
    using this_type         = state_machine<T, Mutex, Observer, ObserverWrapper>;
    using base_machine_type = detail::state_machine_base< T, Mutex, this_type >;
    using mutex_type        = Mutex;
    using lock_guard        = typename detail::lock_guard_type<mutex_type>::type;
    using observer_wrapper  = ObserverWrapper<Observer>;
    using event_invokation  = ::std::function< actions::event_process_result() >;
    using event_queue_item  = ::std::pair< event_invokation, detail::event_base::id_type const* >;
    using event_queue       = ::std::deque< event_queue_item >;
    using deferred_queue    = ::std::list< event_queue_item >;
public:
    state_machine()
        : base_machine_type{this},
          is_top_{},
          handled_{ base_machine_type::current_handled_events() },
          deferred_{ base_machine_type::current_deferrable_events() },
          mutex_{},
          queued_events_{},
          queue_size_{0},
          deferred_top_{},
          deferred_events_{},
          deferred_event_ids_{}
      {}
    template<typename ... Args>
    explicit
    state_machine(Args&& ... args)
        : base_machine_type(this, ::std::forward<Args>(args)...),
          is_top_{},
          handled_{ base_machine_type::current_handled_events() },
          deferred_{ base_machine_type::current_deferrable_events() },
          mutex_{},
          queued_events_{},
          queue_size_{0},
          deferred_top_{},
          deferred_events_{},
          deferred_event_ids_{}
    {}

    template < typename Event >
    actions::event_process_result
    process_event( Event&& event )
    {
        if (!queue_size_ && !is_top_.test_and_set()) {
            auto res = process_event_dispatch(::std::forward<Event>(event));
            is_top_.clear();
            // Process enqueued events
            process_event_queue();
            return res;
        } else {
            // Enqueue event
            enqueue_event(::std::forward<Event>(event));
            return actions::event_process_result::defer;
        }
    }

    detail::event_set const&
    current_handled_events() const
    { return handled_; }
    detail::event_set const&
    current_deferrable_events() const
    { return deferred_; }
    detail::event_set const&
    current_deferred_events() const
    { return deferred_event_ids_; }

    void
    clear_deferred_events()
    {
        lock_guard lock{mutex_};
        deferred_queue{}.swap(deferred_events_);
        detail::event_set{}.swap(deferred_event_ids_);
    }
private:
    template < typename Event >
    actions::event_process_result
    process_event_dispatch( Event&& event )
    {
        return process_event_impl(::std::forward<Event>(event),
                    detail::event_process_selector<
                        Event,
                        typename state_machine::handled_events>{} );
    }

    template < typename Event >
    actions::event_process_result
    process_event_impl(Event&& event,
        detail::process_type<actions::event_process_result::process> const& sel)
    {
        using actions::event_process_result;
        observer_wrapper::start_process_event(*this, ::std::forward<Event>(event));
        auto res = base_machine_type::process_event_impl(*this, ::std::forward<Event>(event), sel );
        switch (res) {
            case event_process_result::process:
                // Changed state. Process deferred events
                handled_    = base_machine_type::current_handled_events();
                deferred_   = base_machine_type::current_deferrable_events();
                process_deferred_queue();
                break;
            case event_process_result::process_in_state:
                observer_wrapper::processed_in_state(*this, ::std::forward<Event>(event));
                break;
            case event_process_result::defer:
                // Add event to deferred queue
                defer_event(::std::forward<Event>(event));
                break;
            case event_process_result::refuse:
                // The event cannot be processed in current state
                observer_wrapper::reject_event(*this, ::std::forward<Event>(event));
                return reject_event_impl(::std::forward<Event>(event),
                        ::std::integral_constant<bool,
                        actions::detail::handles_reject<T, Event>::value>{});
            default:
                break;
        }
        return res;
    }
    template < typename Event >
    actions::event_process_result
    process_event_impl(Event&&,
        detail::process_type<actions::event_process_result::refuse> const&)
    {
        static_assert( detail::event_process_selector<
                Event,
                typename state_machine::handled_events,
                typename state_machine::deferred_events>::value
            != actions::event_process_result::refuse,
            "Event type is not handled by this state machine" );
        return actions::event_process_result::refuse;
    }

    template < typename Event >
    actions::event_process_result
    reject_event_impl(Event&& event, ::std::true_type const&)
    {
        return this->T::reject_event(::std::forward<Event>(event), *this);
    }

    template < typename Event >
    actions::event_process_result
    reject_event_impl(Event&&, ::std::false_type const&)
    {
        return actions::event_process_result::refuse;
    }

    template < typename Event >
    void
    enqueue_event(Event&& event)
    {
        using evt_identity = typename detail::event_identity<Event>::type;
        {
            lock_guard lock{mutex_};
            ++queue_size_;
            observer_wrapper::enqueue_event(*this, ::std::forward<Event>(event));
            Event evt{::std::forward<Event>(event)};
            queued_events_.emplace_back([&, evt]() mutable {
                return process_event_dispatch(::std::move(evt));
            }, &evt_identity::id);
        }
        // Process enqueued events in case we've been waiting for queue
        // mutex release
        process_event_queue();
    }

    void
    lock_and_swap_queue(event_queue& queue)
    {
        lock_guard lock{mutex_};
        ::std::swap(queued_events_, queue);
        queue_size_ -= queue.size();
    }

    void
    process_event_queue()
    {
        while (queue_size_ > 0 && !is_top_.test_and_set()) {
            observer_wrapper::start_process_events_queue(*this);
            while (queue_size_ > 0) {
                event_queue postponed;
                lock_and_swap_queue(postponed);
                for (auto const& event : postponed) {
                    event.first();
                }
            }
            observer_wrapper::end_process_events_queue(*this);
            is_top_.clear();
        }
    }

    template < typename Event >
    void
    defer_event(Event&& event)
    {
        using evt_identity = typename detail::event_identity<Event>::type;

        observer_wrapper::defer_event(*this, ::std::forward<Event>(event));
        Event evt{::std::forward<Event>(event)};
        deferred_events_.emplace_back([&, evt]() mutable {
            return process_event_dispatch(::std::move(evt));
        }, &evt_identity::id);
        deferred_event_ids_.insert(&evt_identity::id);
    }
    void
    process_deferred_queue()
    {
        if (!deferred_top_.test_and_set()) {
            using actions::event_process_result;
            deferred_queue deferred;
            detail::event_set event_ids;
            if (skip_deferred_queue()) {
                observer_wrapper::skip_processing_deferred_queue(*this);
            } else {
                ::std::swap(deferred_events_, deferred);
                ::std::swap(deferred_event_ids_, event_ids);
            }
            while (!deferred.empty()) {
                observer_wrapper::start_process_deferred_queue(*this, deferred.size());
                auto res = event_process_result::refuse;
                for (auto event = deferred.begin(); event != deferred.end();) {
                    if (handled_.count(event->second)) {
                        res = event->first();
                        deferred.erase(event++);
                    } else if (deferred_.count(event->second)) {
                        // Move directly to the deferred queue
                        ::std::size_t count{0};
                        auto next = event;
                        while (next != deferred.end() && next->second == event->second) {
                            ++next;
                            ++count;
                        }
                        deferred_events_.splice(deferred_events_.end(),
                            deferred, event, next);
                        deferred_event_ids_.insert(event->second);
                        event = next;
                        observer_wrapper::postpone_deferred_events(*this, count);
                    } else {
                        deferred.erase(event++);
                        observer_wrapper::drop_deferred_event(*this);
                    }
                    if (res == event_process_result::process)
                        break;
                }
                for (auto event = deferred.begin(); event != deferred.end();) {
                    ::std::size_t count{0};
                    auto next = event;
                    while (next != deferred.end() && next->second == event->second) {
                        ++count;
                        ++next;
                    }
                    deferred_events_.splice(deferred_events_.end(),
                        deferred, event, next);
                    deferred_event_ids_.insert(event->second);
                    event = next;
                    observer_wrapper::postpone_deferred_events(*this, count);
                }
                event_ids.clear();
                observer_wrapper::end_process_deferred_queue(*this, deferred_events_.size());
                if (res == event_process_result::process) {
                    if (skip_deferred_queue()) {
                        observer_wrapper::skip_processing_deferred_queue(*this);
                    } else {
                        ::std::swap(deferred_events_, deferred);
                        ::std::swap(deferred_event_ids_, event_ids);
                    }
                }
            }
            deferred_top_.clear();
        }
    }

    bool
    skip_deferred_queue() const
    {
        detail::event_set st;
        ::std::set_intersection(handled_.begin(), handled_.end(),
            deferred_event_ids_.begin(), deferred_event_ids_.end(),
            ::std::inserter(st, st.end()));
        return st.empty();
    }
private:
    using atomic_counter    = ::std::atomic< ::std::size_t >;

    ::std::atomic_flag      is_top_;

    detail::event_set       handled_;
    detail::event_set       deferred_;

    mutex_type              mutex_;
    event_queue             queued_events_;
    atomic_counter          queue_size_;

    ::std::atomic_flag      deferred_top_;
    deferred_queue          deferred_events_;
    detail::event_set       deferred_event_ids_;
};

//----------------------------------------------------------------------------
//  Priority State machine
//----------------------------------------------------------------------------
using event_priority_type = ::std::int32_t;
template < typename T >
struct event_priority_traits : ::std::integral_constant<event_priority_type, 0>{};

template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
class priority_state_machine :
        public detail::state_machine_base< T, Mutex,
            priority_state_machine<T, Mutex, Observer, ObserverWrapper> >,
        public ObserverWrapper<Observer> {
public:
    static_assert( ::psst::meta::is_empty< typename T::deferred_events >::value,
            "Outer state machine cannot defer events" );
    using this_type         = priority_state_machine<T, Mutex, Observer, ObserverWrapper>;
    using base_machine_type = detail::state_machine_base< T, Mutex, this_type >;
    using mutex_type        = Mutex;
    using lock_guard        = typename detail::lock_guard_type<mutex_type>::type;
    using observer_wrapper  = ObserverWrapper<Observer>;
    using event_invokation  = ::std::function< actions::event_process_result() >;
    using prioritized_event = ::std::pair<event_invokation, event_priority_type>;
    struct event_comparison {
        bool
        operator()(prioritized_event const& lhs, prioritized_event const& rhs) const
        { return lhs.second < rhs.second; }
    };
    using event_queue       = ::std::priority_queue< prioritized_event, ::std::vector<prioritized_event>, event_comparison >;
public:
    priority_state_machine()
        : base_machine_type{this},
          is_top_{},
          mutex_{},
          queued_events_{},
          queue_size_{0},
          deferred_mutex_{},
          deferred_events_{}
      {}
    template<typename ... Args>
    explicit
    priority_state_machine(Args&& ... args)
        : base_machine_type(this, ::std::forward<Args>(args)...),
          is_top_{},
          mutex_{},
          queued_events_{},
          queue_size_{0},
          deferred_mutex_{},
          deferred_events_{}
    {}

    template < typename Event >
    actions::event_process_result
    process_event( Event&& event, event_priority_type priority =
                event_priority_traits< typename ::std::decay<Event>::type >::value )
    {
        if (!queue_size_ && !is_top_.test_and_set()) {
            auto res = process_event_dispatch(::std::forward<Event>(event), priority);
            is_top_.clear();
            // Process enqueued events
            process_event_queue();
            return res;
        } else {
            // Enqueue event
            enqueue_event(::std::forward<Event>(event), priority);
            return actions::event_process_result::defer;
        }
    }
private:
    template < typename Event >
    actions::event_process_result
    process_event_dispatch( Event&& event, event_priority_type priority )
    {
        return process_event_impl(::std::forward<Event>(event), priority,
                detail::event_process_selector<
                        Event,
                        typename priority_state_machine::handled_events>{} );
    }

    template < typename Event >
    actions::event_process_result
    process_event_impl(Event&& event, event_priority_type priority,
        detail::process_type<actions::event_process_result::process> const& sel)
    {
        using actions::event_process_result;
        observer_wrapper::start_process_event(*this, ::std::forward<Event>(event));
        auto res = base_machine_type::process_event_impl(*this, ::std::forward<Event>(event), sel );
        switch (res) {
            case event_process_result::process:
                // Changed state. Process deferred events
                process_deferred_queue();
                break;
            case event_process_result::process_in_state:
                observer_wrapper::processed_in_state(*this, ::std::forward<Event>(event));
                break;
            case event_process_result::defer:
                // Add event to deferred queue
                defer_event(::std::forward<Event>(event), priority);
                break;
            case event_process_result::refuse:
                // The event cannot be processed in current state
                observer_wrapper::reject_event(*this, ::std::forward<Event>(event));
                break;
            default:
                break;
        }
        return res;
    }
    template < typename Event >
    actions::event_process_result
    process_event_impl(Event&&, event_priority_type,
        detail::process_type<actions::event_process_result::refuse> const&)
    {
        static_assert( detail::event_process_selector<
                Event,
                typename priority_state_machine::handled_events,
                typename priority_state_machine::deferred_events>::value
            != actions::event_process_result::refuse,
            "Event type is not handled by this state machine" );
        return actions::event_process_result::refuse;
    }

    template < typename Event >
    void
    enqueue_event(Event&& event, event_priority_type priority)
    {
        {
            lock_guard lock{mutex_};
            ++queue_size_;
            observer_wrapper::enqueue_event(*this, ::std::forward<Event>(event));
            Event evt{::std::forward<Event>(event)};
            queued_events_.emplace([&, evt, priority]() mutable {
                return process_event_dispatch(::std::move(evt), priority);
            }, priority);
        }
        // Process enqueued events in case we've been waiting for queue
        // mutex release
        process_event_queue();
    }

    void
    lock_and_swap_queue(event_queue& queue)
    {
        lock_guard lock{mutex_};
        ::std::swap(queued_events_, queue);
        queue_size_ -= queue.size();
    }

    void
    process_event_queue()
    {
        while (queue_size_ > 0 && !is_top_.test_and_set()) {
            observer_wrapper::start_process_events_queue(*this);
            while (queue_size_ > 0) {
                event_queue postponed;
                lock_and_swap_queue(postponed);
                while (!postponed.empty()) {
                    postponed.top().first();
                    postponed.pop();
                }
            }
            observer_wrapper::end_process_events_queue(*this);
            is_top_.clear();
        }
    }

    template < typename Event >
    void
    defer_event(Event&& event, event_priority_type priority)
    {
        lock_guard lock{deferred_mutex_};
        observer_wrapper::defer_event(*this, ::std::forward<Event>(event));
        Event evt{::std::forward<Event>(event)};
        deferred_events_.emplace([&, evt, priority]() mutable {
            return process_event_dispatch(::std::move(evt), priority);
        }, priority);
    }
    void
    process_deferred_queue()
    {
        using actions::event_process_result;
        event_queue deferred;
        {
            lock_guard lock{deferred_mutex_};
            ::std::swap(deferred_events_, deferred);
        }
        while (!deferred.empty()) {
            observer_wrapper::start_process_deferred_queue(*this, deferred.size());
            auto res = event_process_result::refuse;
            while (!deferred.empty()) {
                res = deferred.top().first();
                deferred.pop();
                if (res == event_process_result::process)
                    break;
            }
            {
                lock_guard lock{deferred_mutex_};
                while (!deferred.empty()) {
                    deferred_events_.push(deferred.top());
                    deferred.pop();
                }
            }
            observer_wrapper::end_process_deferred_queue(*this, deferred_events_.size());
            if (res == event_process_result::process) {
                ::std::swap(deferred_events_, deferred);
            }
        }
    }
private:
    using atomic_counter    = ::std::atomic< ::std::size_t >;

    ::std::atomic_flag      is_top_;

    mutex_type              mutex_;
    event_queue             queued_events_;
    atomic_counter          queue_size_;

    mutex_type              deferred_mutex_;
    event_queue             deferred_events_;
};

template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
state_machine<T, Mutex, Observer, ObserverWrapper>&
root_machine(state_machine<T, Mutex, Observer, ObserverWrapper>& fsm)
{
    return fsm;
}

template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
state_machine<T, Mutex, Observer, ObserverWrapper> const&
root_machine(state_machine<T, Mutex, Observer, ObserverWrapper> const& fsm)
{
    return fsm;
}

template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
priority_state_machine<T, Mutex, Observer, ObserverWrapper>&
root_machine(priority_state_machine<T, Mutex, Observer, ObserverWrapper>& fsm)
{
    return fsm;
}

template < typename T, typename Mutex, typename Observer,
        template<typename> class ObserverWrapper >
priority_state_machine<T, Mutex, Observer, ObserverWrapper> const&
root_machine(priority_state_machine<T, Mutex, Observer, ObserverWrapper> const& fsm)
{
    return fsm;
}

template < typename T, typename FSM >
auto
root_machine(inner_state_machine<T, FSM>& fsm)
    -> decltype(root_machine(fsm.enclosing_fsm()))
{
    return root_machine(fsm.enclosing_fsm());
}

template < typename T, typename FSM >
auto
root_machine(inner_state_machine<T, FSM> const& fsm)
    -> decltype(root_machine(fsm.enclosing_fsm()))
{
    return root_machine(fsm.enclosing_fsm());
}

template < typename T, typename FSM >
auto
root_machine(state<T, FSM>& fsm)
    -> decltype(root_machine(fsm.enclosing_fsm()))
{
    return root_machine(fsm.enclosing_fsm());
}

template < typename T, typename FSM >
auto
root_machine(state<T, FSM> const& fsm)
    -> decltype(root_machine(fsm.enclosing_fsm()))
{
    return root_machine(fsm.enclosing_fsm());
}

}  /* namespace afsm */

#endif /* AFSM_FSM_HPP_ */
