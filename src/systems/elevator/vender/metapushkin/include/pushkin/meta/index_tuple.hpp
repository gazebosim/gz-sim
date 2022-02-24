/*
 * index_tuple.hpp
 *
 *  Created on: 29 мая 2016 г.
 *      Author: sergey.fedorov
 */

#ifndef PUSHKIN_META_INDEX_TUPLE_HPP_
#define PUSHKIN_META_INDEX_TUPLE_HPP_

#include <cstdint>

namespace psst {
namespace meta {

template < ::std::size_t ... Indexes >
struct indexes_tuple {
    static constexpr ::std::size_t size = sizeof ... (Indexes);
};

template < ::std::size_t num, typename tp = indexes_tuple <> >
struct index_builder;

template < ::std::size_t num, ::std::size_t ... Indexes >
struct index_builder< num, indexes_tuple< Indexes ... > >
    : index_builder< num - 1, indexes_tuple< Indexes ..., sizeof ... (Indexes) > > {
};

template <::std::size_t ... Indexes >
struct index_builder< 0, indexes_tuple< Indexes ... > > {
    typedef indexes_tuple < Indexes ... > type;
    static constexpr ::std::size_t size = sizeof ... (Indexes);
};

}  /* namespace meta */
}  /* namespace pus */

#endif /* PUSHKIN_META_INDEX_TUPLE_HPP_ */
