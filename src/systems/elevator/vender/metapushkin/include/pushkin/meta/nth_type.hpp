/*
 * nth_type.hpp
 *
 *  Created on: 28 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef PUSHKIN_META_NTH_TYPE_HPP_
#define PUSHKIN_META_NTH_TYPE_HPP_

#include <cstdint>

namespace psst {
namespace meta {

template < ::std::size_t N, typename ... T >
struct nth_type;

template < ::std::size_t N, typename T, typename ... Y >
struct nth_type< N, T, Y ... > : nth_type < N - 1, Y ... > {
    static_assert(N <= sizeof ...(Y), "Index type is out of range");
};

template < typename T, typename ... Y >
struct nth_type < 0, T, Y ... > {
    using type = T;
};

}  /* namespace meta */
}  /* namespace pus */


#endif /* PUSHKIN_META_NTH_TYPE_HPP_ */
