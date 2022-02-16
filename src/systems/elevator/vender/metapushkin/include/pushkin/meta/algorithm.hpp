/*
 * algorithm.hpp
 *
 *  Created on: 28 мая 2016 г.
 *      Author: sergey.fedorov
 */

/**
 * @page Metaprogramming algorithms
 *
 */
#ifndef PUSHKIN_META_ALGORITHM_HPP_
#define PUSHKIN_META_ALGORITHM_HPP_

#include <type_traits>
#include <limits>

#include <pushkin/meta/type_tuple.hpp>
#include <pushkin/meta/integer_sequence.hpp>

namespace psst {
namespace meta {

//@{
/**
 * Metafunction to determine if typename T is contained in type
 * variadic pack Y ...
 */
template < typename T, typename ... Y >
struct contains;
template < typename T, typename V, typename ... Y >
struct contains< T, V, Y ... > : contains < T, Y ...> {};
template < typename T, typename ... Y >
struct contains< T, T, Y ... > : ::std::true_type {};
template < typename T >
struct contains< T, T > : ::std::true_type {};
template < typename T, typename Y >
struct contains< T, Y > : ::std::false_type {};
template < typename T >
struct contains<T> : ::std::false_type {};
template < typename T, typename ... Y >
struct contains< T, type_tuple<Y...> > : contains<T, Y...> {};

template < typename T, typename ... Y >
using contains_t = typename contains<T, Y...>::type;
template < typename T, typename ...Y >
constexpr bool contains_v = contains_t<T, Y...>::value;
//@}

//@{
/**
 * Metafunction to determine if a variadic pack is empty
 */
template < typename ... T >
struct is_empty : ::std::false_type {};
template <>
struct is_empty<> : ::std::true_type {};
template <>
struct is_empty<void> : ::std::true_type {};
template < typename ... T >
struct is_empty< type_tuple<T...> >
    : ::std::conditional<
        (sizeof ... (T) > 0),
        ::std::false_type,
        ::std::true_type
    >::type {};
template <typename T>
struct is_empty<::std::integer_sequence<T>> : ::std::true_type {};
template <typename T, T ... Values>
struct is_empty<::std::integer_sequence<T, Values...>> : ::std::false_type {};

template < typename ... T >
using is_empty_t = typename is_empty<T...>::type;
template < typename ... T >
constexpr bool is_empty_v = is_empty_t<T...>::value;
//@}

//@{
/**
 * Metafunction to get the fist type from a variadic pack of types
 * or the first value from an integer_sequence.
 */
template < typename ... T >
struct front;
template < typename ... T>
using front_t = typename front<T...>::type;

template <>
struct front<> {
    using type = void;
};

template < typename T, typename ... Y >
struct front<T, Y...> {
    using type = T;
};

template < typename ... T >
struct front< type_tuple<T...> > : front<T...> {};

template < typename T, T V, T... Values>
struct front<::std::integer_sequence<T, V, Values...>> : std::integral_constant<T, V> {};
template < typename T >
struct front<::std::integer_sequence<T>> {
    using type = void;
};
//@}

//@{
/**
 * Metafunction to get the last type from a variadic pack of types
 * or the last value from an integer_sequnce
 */
template < typename ... T >
struct back;
template < typename ... T >
using back_t = typename back<T...>::type;

template <>
struct back<> {
    using type = void;
};

template < typename T >
struct back<T> {
    using type = T;
};

template < typename T, typename ... Y >
struct back<T, Y...> : back <Y...> {};

template < typename ... T >
struct back<type_tuple<T...>> : back<T...> {};

template < typename T, T V >
struct back<::std::integer_sequence<T, V>> : std::integral_constant<T, V> {};
template < typename T, T V, T... Values>
struct back<::std::integer_sequence<T, V, Values...>> : back<::std::integer_sequence<T, Values...>> {};
template < typename T >
struct back<::std::integer_sequence<T>> {
    using type = void;
};
//@}

//@{
/**
 * Metafunction to find an index of T in variadic pack Y...
 */
template < typename T, typename ... Y >
struct index_of;

namespace detail {

template < typename T, ::std::size_t N, typename ... Y >
struct index_of_impl;

template < typename T, ::std::size_t N, typename V, typename ... Y >
struct index_of_impl< T, N, V, Y ... > : index_of_impl<T, N + 1, Y...> {};
template < typename T, ::std::size_t N, typename ... Y >
struct index_of_impl< T, N, T, Y ... > {
    static constexpr ::std::size_t value    = N;
    static constexpr bool found             = true;
};

template < typename T, ::std::size_t N >
struct index_of_impl< T, N, T > {
    static constexpr ::std::size_t value    = N;
    static constexpr bool found             = true;
};

template < typename T, ::std::size_t N, typename Y>
struct index_of_impl<T, N, Y> {
    static constexpr ::std::size_t value    = ::std::numeric_limits<::std::size_t>::max();
    static constexpr bool found             = false;
};

template < typename T, ::std::size_t N>
struct index_of_impl<T, N> {
    static constexpr ::std::size_t value    = ::std::numeric_limits<::std::size_t>::max();
    static constexpr bool found             = false;
};

}  /* namespace detail */

template < typename T, typename ... Y >
struct index_of : detail::index_of_impl<T, 0, Y...> {};
template < typename T, typename ... Y >
struct index_of< T, type_tuple<Y...> > : detail::index_of_impl<T, 0, Y...> {};

template < typename T, typename ... Y >
using index_of_t = typename index_of<T, Y...>::type;
//@}

//@{
/**
 * Metafunction to combine two or more type tuples
 */
template < typename ... T >
struct combine;

template < typename ... T >
using combine_t = typename combine<T...>::type;

template <>
struct combine<> {
    using type = type_tuple<>;
};

template < typename T >
struct combine < T > {
    using type = type_tuple<T>;
};

template < typename ... T >
struct combine < type_tuple<T...> > {
    using type = type_tuple<T...>;
};

template < typename ... T, typename U, typename ... Y >
struct combine< type_tuple<T...>, U, Y...>
    : combine< type_tuple<T..., U>, Y...> {};

template < typename T, typename ... U, typename ... Y >
struct combine< T, type_tuple<U...>, Y... > :
    combine<type_tuple<T, U...>, Y...>{};


template < typename ... T, typename ... U, typename ... Y >
struct combine< type_tuple<T...>, type_tuple<U...>, Y... > {
    using type = combine_t< type_tuple<T..., U... >, Y...>;
};
//@}

//@{
/**
 * Metafunction to get a part of variadic type pack
 */
template < ::std::size_t Fist, ::std::size_t Last, typename ... T >
struct slice;

namespace detail {

template <typename IndexTuple, typename T>
struct slice_impl;

template <::std::size_t ... Indexes, typename ... T>
struct slice_impl< ::std::index_sequence<Indexes...>, type_tuple<T...> > {
    using index_tuple = ::std::index_sequence<Indexes...>;
    static_assert(front_t<index_tuple>::value < sizeof...(T), "Start index of slice is out of range");
    static_assert(back_t<index_tuple>::value < sizeof...(T), "End index of slice is out of range");
    using types = type_tuple<T...>;
    using type = type_tuple< typename types::template type<Indexes>... >;
};

template <typename ... T>
struct slice_impl< ::std::index_sequence<>, type_tuple<T...> > {
    using type = type_tuple<>;
};

}  // namespace detail

template < ::std::size_t Fist, ::std::size_t Last, typename ... T >
struct slice : detail::slice_impl< make_index_sequence<Fist, Last>, type_tuple<T...> > {};
template < ::std::size_t Fist, ::std::size_t Last, typename ... T >
using slice_t = typename slice<Fist, Last, T...>::type;

template < ::std::size_t Fist, ::std::size_t Last, typename ... T >
struct slice<Fist, Last, type_tuple<T...>> : slice<Fist, Last, T...> {};
//@}

//@{
/**
 * Metafunction to reverse a variadic type pack or a type tuple
 */
template < typename ... T >
struct reverse {
    using type = slice_t< sizeof ... (T) - 1, 0, T...>;
};
template < typename ... T >
using reverse_t = typename reverse<T...>::type;

template <>
struct reverse<> {
    using type = type_tuple<>;
};

template < typename ... T >
struct reverse<type_tuple<T...>> : reverse<T...> {};
//@}

//@{
/**
 * Metafunction to push a type to the back of type tuple
 */
template < typename T, typename Y >
struct push_back;
template < typename T, typename Y >
using push_back_t = typename push_back<T, Y>::type;

template < typename ... T, typename Y >
struct push_back<type_tuple<T...>, Y > {
    using type = type_tuple<T..., Y>;
};
//@}

//@{
/**
 * Metafunction to push a type to the from of type tuple
 */
template < typename T, typename Y >
struct push_front;
template < typename T, typename Y >
using push_front_t = typename push_front<T, Y>::type;

template < typename ... T, typename Y >
struct push_front<type_tuple<T...>, Y > {
    using type = type_tuple<Y, T...>;
};
//@}

//@{
/**
 * Metafunction to remove a type from the front of a variadic type pack
 * or a type tuple.
 */
template < typename ... T >
struct pop_front;
template < typename ... T >
using pop_front_t = typename pop_front<T...>::type;

template < typename T, typename ... Y >
struct pop_front<T, Y...> {
    using type = type_tuple<Y...>;
};

template <>
struct pop_front<> {
    using type = type_tuple<>;
};

template < typename ... T >
struct pop_front< type_tuple<T...> > : pop_front<T...> {};
//@}

//@{
/**
 * Metafunction to remove a type from the back of a variadic type pack
 * or a type tuple.
 */
template < typename ... T >
struct pop_back;
template < typename ... T >
using pop_back_t = typename pop_back<T...>::type;

template <typename ... T>
struct pop_back {
    using type = slice_t<0, sizeof...(T) - 2, T...>;
};

template < typename T, typename Y >
struct pop_back<T, Y> {
    using type = type_tuple<T>;
};
template < typename T >
struct pop_back<T> {
    using type = type_tuple<>;
};
template <>
struct pop_back<> {
    using type = type_tuple<>;
};

template <typename ... T>
struct pop_back<type_tuple<T...>> : pop_back<T...> {};
//@}

//@{
/**
 * Metafunction to remove a type from a variadic type pack or type tuple
 */
template < typename T, typename ... Y>
struct remove_type;
template < typename T, typename ... Y>
using remove_type_t = typename remove_type<T, Y...>::type;

template < typename T, typename V, typename ... Y >
struct remove_type<T, V, Y ...> {
    using type = push_front_t<remove_type_t<T, Y...>, V >;
};

template < typename T, typename ... Y >
struct remove_type<T, T, Y...> {
    using type = remove_type_t<T, Y...>;
};

template < typename T >
struct remove_type <T> {
    using type = type_tuple<>;
};

template < typename T, typename ... Y >
struct remove_type< type_tuple<Y...>, T > {
    using type = remove_type_t<T, Y...>;
};
//@}

//@{
/**
 * Metafunciton to do nothing with a variadic type pack
 */
template < typename T, typename ... Y >
struct noop {
    using type = type_tuple<Y...>;
};
//@}

//@{
/**
 * Metafunction to insert a type to a variadic type pack
 * or type tuple only if the pack or tuple doesn't contain it
 */
template < typename T, typename ... Y >
struct insert_type : ::std::conditional_t<
        contains_v<T, Y...>,
        noop<T, Y...>,
        ::std::conditional_t<
            ::std::is_same< T, void >::value,
            noop<T, Y...>,
            push_front<type_tuple<Y...>, T>
        >
    > {};
template < typename T, typename ... Y >
using insert_type_t = typename insert_type<T, Y...>::type;

template < typename T, typename ... Y >
struct insert_type< type_tuple<Y...>, T > {
    using type = insert_type_t<T, Y...>;
};
//@}

//@{
/**
 * Metafunction to build a type tuple containing only
 * unique types from a variadic type pack or type tuple
 */
template < typename ... T >
struct unique;
template < typename ... T>
using unique_t = typename unique<T...>::type;

template < typename T, typename ... Y >
struct unique<T, Y...> {
    using type = insert_type_t<unique_t<Y...>, T>;
};

template <>
struct unique<> {
    using type = type_tuple<>;
};

template < typename ... T >
struct unique< type_tuple<T...> > : unique<T...> {};
template < typename ... T, typename ... Y >
struct unique< type_tuple<T...>, type_tuple<Y...> >
    : unique<T..., Y...> {};

template < typename ... T, typename ... Y >
struct unique< unique<T...>, unique<Y...> >
    : unique<T..., Y...> {};
//@}

template < typename T, typename ... Y >
struct contains< T, unique<Y...> > : contains<T, Y...> {};

//@{
/**
 * Check if all types in a variadic type pack or type tuple
 * match a predicate.
 */
template < template <typename> class Predicate, typename ... T >
struct all_match;

template < template <typename> class Predicate, typename T, typename ... Y >
struct all_match< Predicate, T, Y... >
    : ::std::conditional<
        Predicate<T>::value,
        all_match<Predicate, Y...>,
        ::std::false_type
    >::type {};

template < template <typename> class Predicate, typename T >
struct all_match< Predicate, T >
    : ::std::conditional<
        Predicate<T>::value,
        ::std::true_type,
        ::std::false_type
    >::type {};

template < template <typename> class Predicate >
struct all_match< Predicate >
    : ::std::false_type {};

template < template <typename> class Predicate, typename ... T >
struct all_match< Predicate, type_tuple<T...> >
    : all_match<Predicate, T...> {};
//@}

//@{
/**
 * Check if any type in a variadic type pack or type tuple
 * matches a predicate.
 */
template < template <typename> class Predicate, typename ... T>
struct any_match;

template < template <typename> class Predicate, typename T, typename ... Y >
struct any_match< Predicate, T, Y... >
    : ::std::conditional<
        Predicate<T>::value,
        ::std::true_type,
        any_match<Predicate, Y ...>
    >::type {};

template < template <typename> class Predicate, typename T >
struct any_match< Predicate, T >
    : std::conditional<
        Predicate<T>::value,
        ::std::true_type,
        ::std::false_type
    >::type {};

template < template <typename> class Predicate >
struct any_match< Predicate > : ::std::false_type {};

template < template <typename> class Predicate, typename ... T >
struct any_match< Predicate, type_tuple<T...> >
    : any_match< Predicate, T... >{};
//@}

//@{
/**
 * Metafunction to find types satisfying a predicate
 * in a variadic type pack or type tuple.
 */
template < template <typename> class Predicate, typename ... T >
struct find_if;
template < template <typename> class Predicate, typename ... T >
using find_if_t = typename find_if<Predicate, T...>::type;

template < template <typename> class Predicate >
struct find_if<Predicate> {
    using type = type_tuple<>;
};

template < template <typename> class Predicate, typename T, typename ... Y>
struct find_if< Predicate, T, Y... >
    : ::std::conditional_t<
        Predicate<T>::value,
        combine< T, find_if_t<Predicate, Y...>>,
        find_if<Predicate, Y...>
    > {};

template < template <typename> class Predicate, typename T >
struct find_if< Predicate, T >
    : ::std::conditional_t<
        Predicate<T>::value,
        combine<T>,
        combine<>
    > {};

template < template <typename> class Predicate, typename ... T>
struct find_if< Predicate, type_tuple<T...> > : find_if< Predicate, T... > {};
//@}

//@{
/**
 * Metafunction to remove types matching a predicate.
 */
template < template <typename> class Predicate, typename ... T >
struct remove_if;
template < template <typename> class Predicate, typename ... T >
using remove_if_t = typename remove_if<Predicate, T...>::type;

template < template <typename> class Predicate >
struct remove_if< Predicate > {
    using type = type_tuple<>;
};

template < template <typename> class Predicate, typename T, typename ... Y >
struct remove_if<Predicate, T, Y...>
    : ::std::conditional_t<
        Predicate<T>::value,
        remove_if<Predicate, Y...>,
        combine< T, remove_if_t<Predicate, Y...> >
    > {};

template < template <typename> class Predicate, typename T >
struct remove_if<Predicate, T>
    : ::std::conditional_t<
        Predicate<T>::value,
        combine<>,
        combine<T>
    > {};

template < template <typename> class Predicate, typename ... T>
struct remove_if< Predicate, type_tuple<T...> > : remove_if< Predicate, T... > {};
//@}

//@{
/**
 * Metafunction to apply a predicate to a variadic type pack or
 * type tuple.
 */
template < template<typename> class Predicate, typename ... T >
struct transform {
    using type = type_tuple< typename Predicate<T>::type ... >;
};

template < template<typename> class Predicate, typename ... T >
struct transform < Predicate, type_tuple<T...> >
    : transform< Predicate, T... > {};

template < template<typename> class Predicate >
struct transform<Predicate> {
    using type = type_tuple<>;
};
//@}

//@{
/**
 * Metafunction to invert a predicate.
 *
 * Can be used in metafunction 'binding' as follows:
 * @code
 * template <typename T>
 * using not_fundamental = invert<std::is_fundamental, T>;
 * @endcode
 */
template < template<typename> class Predicate, typename T >
struct invert : ::std::integral_constant<bool, !Predicate<T>::value> {};
//@}

//@{
/**
 * Metafunction to compare two types using a Compare predicate.
 * If the Compare predicate is false for both ways of comparison,
 * meaning the types are evaluated equal, the order of types is not changed.
 */
template <template<typename, typename> class Compare, typename T, typename Y>
struct stable_order {
    using forward = type_tuple<T, Y>;
    using reverse = type_tuple<Y, T>;
    static constexpr bool original_order = Compare<T, Y>::value || !Compare<Y, T>::value;
    using type = ::std::conditional_t<original_order, forward, reverse>;
    using first = typename type::template type<0>;
    using second = typename type::template type<1>;
};
//@}

//@{
/**
 * Metafunction to merge sorted type tuples in a stable sorted order
 */
template <template<typename, typename> class Compare, typename ... T>
struct merge_sorted;
template <template<typename, typename> class Compare, typename ... T>
using merge_sorted_t = typename merge_sorted<Compare, T...>::type;

template <template<typename, typename> class Compare>
struct merge_sorted<Compare> {
    using type = type_tuple<>;
};

template <template<typename, typename> class Compare, typename ... T>
struct merge_sorted<Compare, type_tuple<T...>> {
    using type = type_tuple<T...>;
};

template <template<typename, typename> class Compare, typename ... T>
struct merge_sorted<Compare, type_tuple<T...>, type_tuple<>> {
    using type = type_tuple<T...>;
};

template <template<typename, typename> class Compare, typename ... T>
struct merge_sorted<Compare, type_tuple<>, type_tuple<T...>> {
    using type = type_tuple<T...>;
};

template <template<typename, typename> class Compare, typename T1, typename ...TN, typename Y1, typename ... YN>
struct merge_sorted<Compare, type_tuple<T1, TN...>, type_tuple<Y1, YN...>>
    : ::std::conditional_t<
        stable_order<Compare, T1, Y1>::original_order,
        combine< type_tuple<T1>, merge_sorted_t<Compare, type_tuple<TN...>, type_tuple<Y1, YN...>> >,
        combine< type_tuple<Y1>, merge_sorted_t<Compare, type_tuple<T1, TN...>, type_tuple<YN...>> >
    > {};
//@}

//@{
/**
 * Metafunction to stable sort a list of types using the Compare predicate.
 */
template < template<typename, typename> class Compare, typename ... T >
struct stable_sort;
template < template<typename, typename> class Compare, typename ... T >
using stable_sort_t = typename stable_sort<Compare, T...>::type;

template < template<typename, typename> class Compare, typename ... T >
struct stable_sort {
    static constexpr std::size_t tuple_size = sizeof...(T);
    using first_half = slice_t<0, tuple_size/2, T...>;
    using second_half = slice_t<tuple_size/2 + 1, tuple_size - 1, T...>;
    using type = merge_sorted_t<Compare,
            stable_sort_t<Compare, first_half>, stable_sort_t<Compare,second_half>>;
};

template < template<typename, typename> class Compare, typename ... T >
struct stable_sort<Compare, type_tuple<T...>> : stable_sort<Compare, T...> {};

template < template<typename, typename> class Compare, typename T, typename Y >
struct stable_sort<Compare, T, Y> : stable_order<Compare, T, Y> {};

template < template<typename, typename> class Compare, typename T >
struct stable_sort<Compare, T> {
    using type = type_tuple<T>;
};

template < template<typename, typename> class Compare >
struct stable_sort<Compare> {
    using type = type_tuple<>;
};
//@}

}  /* namespace meta */
}  /* namespace pus */

#endif /* PUSHKIN_META_ALGORITHM_HPP_ */
