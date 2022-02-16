/*
 * char_sequence.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: zmij
 */

#ifndef PUSHKIN_META_CHAR_SEQUENCE_HPP_
#define PUSHKIN_META_CHAR_SEQUENCE_HPP_

#include <pushkin/meta/integer_sequence.hpp>

namespace psst {
namespace meta {

template < char C >
using char_constant = ::std::integral_constant<char, C>;

namespace detail {

template < char C, char ... Chars >
struct first_char {
    using type = ::std::integral_constant<char, C>;
};

template < char C, char ... Chars >
struct last_char : last_char< Chars ... > {};
template < char C >
struct last_char<C> {
    using type = ::std::integral_constant<char, C>;
};

constexpr ::std::size_t
strlen(char const* str)
{
    ::std::size_t sz = 0;
    for (; *str != 0; ++sz, ++str);
    return sz;
}

constexpr int
charcmp(char lhs, char rhs)
{
    return (lhs < rhs) ? - 1 : (lhs > rhs ? 1 : 0);
}

constexpr int
strcmp(char const* lhs, char const* rhs)
{
    int cmp = 0;
    for (; cmp == 0 && *lhs != 0 && *rhs != 0; ++lhs, ++rhs) {
        cmp = charcmp(*lhs, *rhs);
    }
    if (cmp == 0 && (*lhs != 0 || *rhs != 0)) {
        return *lhs != 0 ? 1 : -1;
    }
    return cmp;
}

constexpr char
min(char a, char b)
{ return a < b ? a : b; }

constexpr char
max(char a, char b)
{ return a > b ? a : b; }

template < char Min, char Max, char ... Chars >
struct unwrap_char_range;

} /* namespace detail */

//----------------------------------------------------------------------------
template < char ... Chars >
struct char_sequence_literal {
    using type = char_sequence_literal<Chars...>;
    static constexpr char value[]{ Chars..., 0 };
    static constexpr ::std::size_t size = sizeof ... (Chars);

    static constexpr bool
    eq(char const* str)
    {
        return detail::strcmp(value, str) == 0;
    }
    static constexpr bool
    ne(char const* str)
    {
        return detail::strcmp(value, str) != 0;
    }
    static constexpr bool
    lt(char const* str)
    {
        return detail::strcmp(value, str) < 0;
    }
    static constexpr bool
    gt(char const* str)
    {
        return detail::strcmp(value, str) > 0;
    }
    static constexpr bool
    le(char const* str)
    {
        return detail::strcmp(value, str) <= 0;
    }
    static constexpr bool
    ge(char const* str)
    {
        return detail::strcmp(value, str) >= 0;
    }

    /**
     * Check if a string starts with this char literal
     * @param str
     * @return
     */
    template < typename InputIterator >
    static constexpr bool
    starts_with(InputIterator str)
    {
        if (size == 0)
            return true;
        char const* lhs = value;
        for (; *lhs != 0 && *str != 0; ++lhs, ++str) {
            if (*lhs != *str)
                return false;
        }

        return *lhs == 0;
    }

    static constexpr char const*
    static_begin()
    { return value; }
    static constexpr char const*
    static_end()
    { return value + size; }
};

template < char ... Chars >
constexpr char char_sequence_literal<Chars...>::value[];

template < typename T >
struct make_char_literal;

//----------------------------------------------------------------------------
/**
 * A template structure representing a character sequence
 */
template < char ... Chars >
struct char_sequence {
    using size          = ::std::integral_constant<::std::size_t, sizeof ... (Chars)>;
    using first_char    = typename detail::first_char<Chars...>::type;
    using last_char     = typename detail::last_char<Chars...>::type;
};

template <>
struct char_sequence<> {
    using size          = ::std::integral_constant<::std::size_t, 0>;
};


template < char ... Chars >
struct make_char_literal<char_sequence<Chars...>> {
    using type = char_sequence_literal<Chars...>;
};

//----------------------------------------------------------------------------
/**
 * A range of chars
 */
template < char Min, char Max >
struct char_range {
    static_assert(Min <= Max, "Minimum char must be less or equal to maximum");
    using size          = ::std::integral_constant<::std::size_t, Max - Min + 1>;
    using first_char    = ::std::integral_constant<char, Min>;
    using last_char     = ::std::integral_constant<char, Max>;
};

namespace chars {
//----------------------------------------------------------------------------
//@{
template < typename T, typename U >
struct includes;

template < char MinA, char MaxA, char MinB, char MaxB >
struct includes< char_range< MinA, MaxA >, char_range< MinB, MaxB > > {
    using type = ::std::integral_constant< bool, ( MinA <= MinB && MaxB <= MaxA ) >;
};
//@}

//----------------------------------------------------------------------------
//@{
template < typename T, typename U >
struct overlaps;

template < char MinA, char MaxA, char MinB, char MaxB >
struct overlaps <char_range< MinA, MaxA >, char_range< MinB, MaxB > > {
    using type = ::std::integral_constant< bool, !( MaxA < MinB || MaxB < MinA ) >;
};
//@}

//----------------------------------------------------------------------------
//@{
template < typename T, char>
struct contains;

template < char F, char ... O, char C >
struct contains< char_sequence<F, O...>, C > : contains< char_sequence<O...>, C > {};

template < char ... O, char C >
struct contains< char_sequence<C, O...>, C > : ::std::true_type {};

template < char C >
struct contains< char_sequence<>, C > : ::std::false_type {};

template < char Min, char Max, char C >
struct contains< char_range<Min, Max>, C > :
    ::std::integral_constant<bool, (Min <= C && C <= Max)> {};
//@}

} /* namespace chars */

namespace detail {

template < char Min, char Max, char ... Chars >
struct unwrap_char_range {
    static_assert(Min < Max, "Starting char of the range must be less than ending");
    using type = typename unwrap_char_range< Min, Max - 1, Max, Chars... >::type;
};

template < char C, char ... Chars >
struct unwrap_char_range< C, C, Chars ... > {
    using type = char_sequence<C, Chars...>;
};

#if __cplusplus >= 201402L
/**
 * Convert a pointer to char to a char_sequence, compile time.
 */
template < char const* Str, typename T >
struct make_char_sequence_impl;

template < char const* Str, ::std::size_t ... Indexes >
struct make_char_sequence_impl<Str, ::std::integer_sequence< ::std::size_t, Indexes... >> {
    using type = char_sequence< Str[Indexes]... >;
};

/**
 * Build a table for sorting chars
 */
template < typename IndexTuple, template < char > class Predicate,
        typename V, V ifTrue, V ifFalse = V{} >
struct build_sort_table;

template < ::std::size_t ... Indexes, template < char > class Predicate,
    typename V, V ifTrue, V ifFalse >
struct build_sort_table<::std::integer_sequence< ::std::size_t, Indexes... >,
        Predicate, V, ifTrue, ifFalse > {

    using size = ::std::integral_constant<::std::size_t, sizeof ... (Indexes)>;
    using type = ::std::integer_sequence< V,
            (Predicate< (char)Indexes >::value ? ifTrue : ifFalse) ... >;
};

template < typename T, typename Index >
struct make_char_sort_table;

template < char ... Chars, typename Index >
struct make_char_sort_table<char_sequence<Chars...>, Index> {
    using charset       = char_sequence<Chars...>;
    template < char C >
    using predicate     = chars::contains<charset, C>;
    using type          = typename build_sort_table<Index,
                            predicate, bool, true, false>::type;
};

template < unsigned char N, typename T, char ... Chars >
struct sort_helper;

template < unsigned char N, bool V, bool ... Vals, char ... Chars >
struct sort_helper<N, ::std::integer_sequence<bool, V, Vals...>, Chars... > {
    using type = typename ::std::conditional<V,
            sort_helper<N - 1, ::std::integer_sequence<bool, Vals...>, char(N), Chars...>,
            sort_helper<N - 1, ::std::integer_sequence<bool, Vals...>, Chars...>
        >::type::type;
};

template < unsigned char N, bool V, char ... Chars >
struct sort_helper< N, ::std::integer_sequence<bool, V >, Chars ... > {
    using type = typename ::std::conditional<V,
            char_sequence< char(N), Chars... >,
            char_sequence< Chars... >
        >::type;
};

#endif /* __cplusplus >= 201402L */

} /* namespace detail */

#if __cplusplus >= 201402L
template < typename T >
struct unique_sort;

/**
 * Uniquely sort a sequence of chars
 */
template < char ... Chars >
struct unique_sort<char_sequence<Chars...>> {
    using original_seq      = char_sequence<Chars...>;
    using low_index         = typename detail::reverse_unwrap_range<::std::size_t, 0, 127>::type;
    using high_index        = typename detail::reverse_unwrap_range<::std::size_t, 128, 255>::type;
    using low_table         = typename detail::make_char_sort_table<original_seq, low_index>::type;
    using high_table        = typename detail::make_char_sort_table<original_seq, high_index>::type;
    using low_sorted        = typename detail::sort_helper< 127, low_table >::type;
    using high_sorted       = typename detail::sort_helper< 127, high_table >::type;

    using type              = typename join< low_sorted, high_sorted >::type;
};
#endif /* __cplusplus >= 201402L */

/**
 * Specialization to join two character sequences
 */
template < char ... A, char ... B >
struct join< char_sequence<A...>, char_sequence<B...> > {
    using type = char_sequence<A..., B...>;
};

template < char ... A, char C >
struct join< char_sequence<A...>, ::std::integral_constant< char, C > > {
    using type = char_sequence< A..., C >;
};

template < char ... A, char C >
struct join< ::std::integral_constant< char, C >, char_sequence<A...> > {
    using type = char_sequence< A..., C >;
};

//@{
/** @name Specializations to join adjacent character ranges */
template < char A, char B, char C >
struct join< char_range<A, B>, char_range<B, C> > {
    using type = char_range< A, C >;
};

template < char A, char B, char C >
struct join< char_range<A, B>, char_range<B + 1, C> > {
    using type = char_range< A, C >;
};

template < char A, char B, char C >
struct join< char_range<B, C>, char_range<A, B> > {
    using type = char_range< A, C >;
};

template < char A, char B, char C >
struct join< char_range<B + 1, C>, char_range<A, B> > {
    using type = char_range< A, C >;
};

template < char A, char B >
struct join< char_range<A, B>, ::std::integral_constant<char, A> > {
    using type = char_range<A, B>;
};
template < char A, char B >
struct join< char_range<A, B>, ::std::integral_constant<char, B> > {
    using type = char_range<A, B>;
};

template < char A, char B >
struct join< char_range<A, B>, ::std::integral_constant<char, A-1> > {
    using type = char_range<A-1, B>;
};
template < char A, char B >
struct join< char_range<A, B>, ::std::integral_constant<char, B+1> > {
    using type = char_range<A, B+1>;
};

//@}

#if __cplusplus >= 201402L
namespace detail {

template < char A, char B, char C, char D, bool >
struct overlapping_range_join {
    using type = char_range< min(A, C), max(B, D) >;
};

template < char A, char B, char C, char D >
struct overlapping_range_join< A, B, C, D, false > {
    using type = typename unique_sort<
            typename join<
                typename unwrap_char_range< A, B >::type,
                typename unwrap_char_range< C, D >::type
            >::type
        >::type;
};

template < char A, char B, char C, bool >
struct including_range_join {
    using type = char_range< A, B >;
};

template < char A, char B, char C >
struct including_range_join<A, B, C, false> {
    using type = typename unique_sort<
                typename join<
                    typename unwrap_char_range<A, B>::type,
                    ::std::integral_constant<char, C>
                >::type
            >::type;
};

}  /* namespace detail */

template < char A, char B, char C, char D >
struct join< char_range<A, B>, char_range<C, D> >
    : detail::overlapping_range_join< A, B, C, D,
            chars::overlaps< char_range<A, B>,
                char_range<C, D>>::type::value > {};

template < char A, char B, char C >
struct join< char_range<A, B>, ::std::integral_constant<char, C> >
    : detail::including_range_join<
          A, B, C,
          chars::contains<char_range<A, B>, C>::value> {};

template < char A, char B, char C >
struct join< ::std::integral_constant<char, C>, char_range<A, B> >
    : detail::including_range_join<
          A, B, C,
          chars::contains<char_range<A, B>, C>::value> {};

//----------------------------------------------------------------------------
template < char A, char B >
struct join< char_range<A, B>, char_sequence<> > {
    using type = char_range<A, B>;
};

template < char A, char B, char ... Chars >
struct join< char_range<A, B>, char_sequence<Chars...> > {
    using type = typename unique_sort<
            typename join<
                typename detail::unwrap_char_range< A, B >::type,
                char_sequence< Chars... >
            >::type
        >::type;
};

template < char A, char B >
struct join< char_sequence<>, char_range<A, B> > {
    using type = char_range<A, B>;
};

template < char ... Chars, char A, char B >
struct join< char_sequence<Chars...>, char_range<A, B> > {
    using type = typename unique_sort<
            typename join<
                typename detail::unwrap_char_range< A, B >::type,
                char_sequence< Chars... >
            >::type
        >::type;
};

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
template < char const* Str >
using make_char_sequence = typename detail::make_char_sequence_impl<Str,
                                ::std::make_index_sequence<detail::strlen(Str)>>::type;

template < char A, char B >
struct make_char_literal<char_range<A, B>> {
    using type = typename make_char_literal<
            typename detail::unwrap_char_range< A, B >::type
        >::type;
};

//template < char const* Str >
namespace detail {

/**
 * Make a char sequence literal from a pointer to char, compile-time
 */
template < char const* Str, typename T >
struct make_char_literal_impl;

template < char const* Str, ::std::size_t ... Indexes >
struct make_char_literal_impl<Str, ::std::integer_sequence< ::std::size_t, Indexes... >> {
    using type = char_sequence_literal< Str[Indexes]... >;
};

} /* namespace detail */


template < char const* Str >
using make_char_literal_s = typename detail::make_char_literal_impl<Str,
        ::std::make_index_sequence<detail::strlen(Str)>>::type;

#endif /* __cplusplus >= 201402L */

} /* namespace meta */
} /* namespace psst */

#ifdef __METASHELL
#include <metashell/scalar.hpp>
#endif

#endif /* PUSHKIN_META_CHAR_SEQUENCE_HPP_ */
