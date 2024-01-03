/*
 * def_traits.hpp
 *
 *  Created on: 1 июня 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_DEF_TRAITS_HPP_
#define AFSM_DETAIL_DEF_TRAITS_HPP_

#include <afsm/definition_fwd.hpp>
#include <afsm/detail/tags.hpp>
#include <afsm/detail/exception_safety_guarantees.hpp>

namespace afsm {
namespace def {
namespace traits {

template < typename T >
struct is_transition : ::std::false_type {};

template < typename SourceState, typename Event, typename TargetState,
    typename Action, typename Guard>
struct is_transition< transition<SourceState, Event, TargetState, Action, Guard> >
    : ::std::true_type{};
template < typename Event, typename Action, typename Guard >
struct is_transition< internal_transition< Event, Action, Guard > >
    : ::std::true_type {};

template < typename T >
struct is_internal_transition : ::std::false_type {};

template < typename Event, typename Action, typename Guard >
struct is_internal_transition< internal_transition< Event, Action, Guard > >
    : ::std::true_type {};

template < typename T >
struct is_state
    : ::std::is_base_of< tags::state, T > {};

template < typename T >
struct is_terminal_state
    : ::std::is_base_of< terminal_state<T>, T > {};

template < typename T >
struct is_state_machine
    : ::std::is_base_of< tags::state_machine, T > {};

template < typename T >
struct is_pushdown
    : ::std::is_base_of< tags::pushdown_state, T > {};
template < typename T >
struct is_popup
    : ::std::is_base_of< tags::popup_state, T > {};

namespace detail {

template < typename T, typename Machine, bool IsPush >
struct pushes : ::std::false_type {};

template < typename T, typename Machine >
struct pushes<T, Machine, true>
    : ::std::integral_constant<bool,
        ::std::is_same<typename T::pushdown_machine_type, Machine>::value> {};

template < typename T, typename Machine, bool IsPop >
struct pops : ::std::false_type {};

template < typename T, typename Machine >
struct pops<T, Machine, true>
    : ::std::integral_constant<bool,
        ::std::is_same<typename T::pushdown_machine_type, Machine>::value> {};

}  /* namespace detail */

template < typename T, typename Machine >
struct pushes : detail::pushes<T, Machine, is_pushdown<T>::value> {};
template < typename T, typename Machine >
struct pops : detail::pops<T, Machine, is_popup<T>::value> {};

template < typename T >
struct has_common_base
    : ::std::is_base_of< tags::has_common_base, T > {};

template < typename T >
struct has_history
    : ::std::is_base_of< tags::has_history, T > {};

template < typename T >
struct allow_empty_transition_functions
    : ::std::is_base_of< tags::allow_empty_enter_exit, T > {};

template < typename T >
struct has_orthogonal_regions
    : ::std::integral_constant<bool, !::std::is_same< typename T::orthogonal_regions, void >::value> {};

template < typename T >
struct exception_safety {
    using type = typename ::std::conditional<
        ::std::is_base_of<tags::nothrow_guarantee, T>::value,
        tags::nothrow_guarantee,
        typename ::std::conditional<
            ::std::is_base_of<tags::strong_exception_safety, T>::value,
            tags::strong_exception_safety,
            tags::basic_exception_safety
        >::type
    >::type;
};

namespace detail {
template < typename T, bool HasCommonBase >
struct inner_states_def {
    using definition_type       = T;
    using common_base_tag       = typename definition_type::common_base_tag_type;
    using exception_guarantee   = typename exception_safety<T>::type;

    template < typename U, typename ... Tags >
    using state             = def::state_def<U, common_base_tag, exception_guarantee, Tags...>;
    template < typename U, typename ... Tags >
    using terminal_state    = def::terminal_state<U, common_base_tag, exception_guarantee, Tags...>;
    template < typename U, typename ... Tags >
    using state_machine     = def::state_machine<U, common_base_tag, exception_guarantee, Tags...>;
    template < typename U, typename M, typename ... Tags >
    using push              = def::pushdown<U, M, common_base_tag, exception_guarantee, Tags...>;
    template < typename U, typename M, typename ... Tags >
    using pop               = def::popup<U, M, common_base_tag, exception_guarantee, Tags...>;
};

template < typename T >
struct inner_states_def<T, false> {
    using definition_type   = T;
    using common_base_type  = void;
    using exception_guarantee   = typename exception_safety<T>::type;

    template < typename U, typename ... Tags >
    using state             = def::state_def<U, exception_guarantee, Tags...>;
    template < typename U, typename ... Tags >
    using terminal_state    = def::terminal_state<U, exception_guarantee, Tags...>;
    template < typename U, typename ... Tags >
    using state_machine     = def::state_machine<U, exception_guarantee, Tags...>;
    template < typename U, typename M, typename ... Tags >
    using push              = def::pushdown<U, M, exception_guarantee, Tags...>;
    template < typename U, typename M, typename ... Tags >
    using pop               = def::popup<U, M, exception_guarantee, Tags...>;
};

}  /* namespace detail */

template < typename T >
struct inner_states_definitions
        : detail::inner_states_def<T, has_common_base<T>::value> {};

}  /* namespace traits */
}  /* namespace def */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_DEF_TRAITS_HPP_ */
