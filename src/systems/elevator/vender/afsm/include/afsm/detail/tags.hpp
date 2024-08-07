/*
 * tags.hpp
 *
 *  Created on: 1 июня 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef AFSM_DETAIL_TAGS_HPP_
#define AFSM_DETAIL_TAGS_HPP_

namespace afsm {
namespace def {
namespace tags {

/**
 * Tag for state class.
 * For internal use only.
 */
struct state {};
/**
 * Tag for state machine class.
 * For internal use only.
 */
struct state_machine{};

/**
 * Tag for marking states with history.
 */
struct has_history {};
/**
 * Tag for marking states having common base class.
 * For internal use.
 */
struct has_common_base {};

/**
 * Tag for marking state having common base.
 */
template < typename T >
struct common_base : T, has_common_base {
    using common_base_type      = T;
    using common_base_tag_type  = common_base<common_base_type>;
};

template <>
struct common_base<void> {
    using common_base_type      = void;
    using common_base_tag_type  = common_base<common_base_type>;
};

/**
 * Tag for marking state having common virtual base.
 */
template < typename T >
struct virtual_common_base : virtual T, has_common_base {
    using common_base_type      = T;
    using common_base_tag_type  = virtual_common_base<common_base_type>;
};

template <>
struct virtual_common_base<void> {
    using common_base_type      = void;
    using common_base_tag_type  = virtual_common_base<common_base_type>;
};

//@{
/** @name Push/pop tags */
struct pushdown_state {};
struct popup_state {};

template < typename T >
struct pushdown : pushdown_state {
    using pushdown_machine_type = T;
};

template < typename T >
struct popup : popup_state {
    using pushdown_machine_type = T;
};
//@}

struct allow_empty_enter_exit {};
struct mandatory_empty_enter_exit {};

}  /* namespace tags */
}  /* namespace def */
}  /* namespace afsm */

#endif /* AFSM_DETAIL_TAGS_HPP_ */
