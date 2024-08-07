/*
 * integer_sequence.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: zmij
 */

#ifndef PUSHKIN_META_INTEGER_SEQUENCE_HPP_
#define PUSHKIN_META_INTEGER_SEQUENCE_HPP_

#include <type_traits>
#include <utility>

//#include <pushkin/meta/algorithm.hpp>

namespace psst {
namespace meta {

/**
 * Metafunction to join sequences
 */
template < typename ... T >
struct join;
template < typename ... T >
using join_t = typename join<T...>::type;

template < typename T >
struct join < T > {
    using type = T;
};

template < typename T, typename U, typename ... V >
struct join< T, U, V... >
    : join< join_t< T, U >, V... > {};

template < typename T, T ... A, T... B >
struct join< ::std::integer_sequence<T, A...>, ::std::integer_sequence<T, B...> > {
    using type = ::std::integer_sequence<T, A..., B...>;
};

template < typename T, T ... A, T B >
struct join< ::std::integer_sequence<T, A...>, ::std::integral_constant<T, B > > {
    using type = ::std::integer_sequence<T, A..., B>;
};

template < typename T, T A, T ...B >
struct join< ::std::integral_constant<T, A>, ::std::integer_sequence<T, B...> > {
    using type = ::std::integer_sequence<T, A, B...>;
};

namespace detail {

template < typename T, T Min, T Max, T ... Values >
struct unwrap_range_impl {
    static_assert(Min < Max, "Start of range must be less than ending");
    using type = typename unwrap_range_impl<T, Min, Max - 1, Max, Values ... >::type;
};

template < typename T, T V, T ... Values >
struct unwrap_range_impl<T, V, V, Values...> {
    using type = ::std::integer_sequence< T, V, Values... >;
};

template < typename T, T Min, T Max, T ... Values >
struct reverse_unwrap_range_impl {
    static_assert(Min < Max, "Start of range must be less than ending");
    using type = typename reverse_unwrap_range_impl<T, Min + 1, Max, Min, Values ... >::type;
};

template < typename T, T V, T ... Values >
struct reverse_unwrap_range_impl<T, V, V, Values...> {
    using type = ::std::integer_sequence< T, V, Values... >;
};

const ::std::size_t template_depth = 128;

template < typename T, T Min, T Max >
struct unwrap_range;
template < typename T, T Min, T Max >
struct reverse_unwrap_range;

template < typename T, T Min, T Max >
struct split_range_helper {
    using type = typename join<
            typename unwrap_range_impl<T, Min, Min + template_depth - 1>::type,
            typename unwrap_range<T, Min + template_depth, Max>::type
    >::type;
};

template < typename T, T Min, T Max >
struct unwrap_range :
    ::std::conditional< (Max - Min < template_depth),
         unwrap_range_impl<T, Min, Max>,
         split_range_helper<T, Min, Max>
    >::type {};

template < typename T, T Min, T Max >
struct reverse_split_range_helper {
    using type = typename join<
            typename reverse_unwrap_range<T, Min + template_depth, Max>::type,
            typename reverse_unwrap_range_impl<T, Min, Min + template_depth - 1>::type
    >::type;
};
template < typename T, T Min, T Max >
struct reverse_unwrap_range :
    ::std::conditional< (Max - Min < template_depth),
         reverse_unwrap_range_impl<T, Min, Max>,
         reverse_split_range_helper<T, Min, Max>
    >::type {};

template < typename T, T Min, T Max, bool Reverse >
struct make_integer_sequence_impl : unwrap_range<T, Min, Max> {};

template < typename T, T Min, T Max >
struct make_integer_sequence_impl<T, Min, Max, true> : reverse_unwrap_range<T, Max, Min> {};

} /* namespace detail */


/**
 * Metafunction to make integer sequences in range [Min, Max]
 * If Min > Max, the sequence will be reversed.
 */
template < typename T, T Min, T Max >
using make_integer_sequence =
    typename detail::make_integer_sequence_impl<T, Min, Max, (Min > Max )>::type;

template <::std::size_t Min, ::std::size_t Max>
using make_index_sequence = make_integer_sequence<::std::size_t, Min, Max>;

namespace detail {

template < ::std::size_t Head, template <typename> class Predicate, typename ... T >
struct find_index_if_impl;

template < ::std::size_t Head, template <typename> class Predicate, typename T, typename ...Y >
struct find_index_if_impl<Head, Predicate, T, Y...> :
    ::std::conditional<
            Predicate<T>::value,
            typename join< ::std::integral_constant<::std::size_t, Head>,
                    typename find_index_if_impl<Head + 1, Predicate, Y... >::type >::type,
            typename find_index_if_impl<Head + 1, Predicate, Y...>::type
        > {};

template < ::std::size_t Head, template <typename> class Predicate, typename T >
struct find_index_if_impl<Head, Predicate, T > :
    ::std::conditional<
            Predicate<T>::value,
            ::std::index_sequence<Head>,
            ::std::index_sequence<>
        > {};

} /* namespace detail */

/**
 * Metafunction to build index sequence of types matching predicate
 */
template < template <typename> class Predicate, typename ... T >
struct find_index_if : detail::find_index_if_impl<0, Predicate, T...> {};

template <template <typename> class Predicate>
struct find_index_if<Predicate> {
    using type = ::std::index_sequence<>;
};

template <typename T, T A, T B>
struct max : std::integral_constant<T, (A > B ? A : B)> {};
template <typename T, T A, T B>
struct min : std::integral_constant<T, (A < B ? A : B)> {};


} /* namespace meta */
} /* namespace psst */

#endif /* PUSHKIN_META_INTEGER_SEQUENCE_HPP_ */
