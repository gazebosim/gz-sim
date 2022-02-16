/*
 * functions.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: zmij
 */

#ifndef PUSHKIN_META_FUNCTIONS_HPP_
#define PUSHKIN_META_FUNCTIONS_HPP_

namespace psst {
namespace meta {

inline bool
any_of(::std::initializer_list<bool> args)
{
    for (auto v : args) {
        if (v)
            return true;
    }
    return false;
}

inline bool
all_of(::std::initializer_list<bool> args)
{
    for (auto v : args) {
        if (!v)
            return false;
    }
    return true;
}

}  /* namespace meta */
}  /* namespace psst */



#endif /* PUSHKIN_META_FUNCTIONS_HPP_ */
