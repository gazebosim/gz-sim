/*
 * demangle.hpp
 *
 *  Created on: Jun 23, 2016
 *      Author: zmij
 */

#ifndef PUSHKIN_UTIL_DEMANGLE_HPP_
#define PUSHKIN_UTIL_DEMANGLE_HPP_

#include <string>
#include <iosfwd>

/*
 * Test for non-portable GNU extension compatibility.
 */
#ifdef __GNUC__
#define HAS_GNU_EXTENSIONS_
#endif

 /*
  * cxxabi.h is a non-portable GNU extension.  Only include it if we're compiling against a
  * GNU library.
  */
#ifdef HAS_GNU_EXTENSIONS_
#include <cxxabi.h>
#endif

namespace psst {
namespace util {

/**
 * Type name demangle function
 * Usage:
 * @code
 * ::std::cout << demangle< ::std::iostream >() << "\n"
 * @endcode
 * @return Demangled type name
 */
template < typename T >
::std::string
demangle()
{
    int status {0};

    /*
     * Demangle the type name if using non-portable GNU extensions.
     */
    #ifdef HAS_GNU_EXTENSIONS_
    char* ret = abi::__cxa_demangle( typeid(T).name(), nullptr, nullptr, &status );
    ::std::string res{ret};
    if(ret) free(ret);
    #else
    ::std::string res{ typeid(T).name() };
    #endif
    return res;
}

/**
 * Type name demangle function, io manip interface. Doesn't create a string.
 * Usage:
 * @code
 * ::std::cout << demangle< ::std::iostream > << "\n"
 * @endcode
 * @param os
 */
template < typename T >
void
demangle(::std::iostream& os)
{
    ::std::ostream::sentry s(os);
    if (s) {
        int status {0};

        /*
         * Demangle the type name if using non-portable GNU extensions.
         */
        #ifdef HAS_GNU_EXTENSIONS_
        char* ret = abi::__cxa_demangle( typeid(T).name(), nullptr, nullptr, &status );
        if (ret) {
            os << ret;
            free(ret);
        }
        #else
        os << typeid(T).name();
        #endif
    }
}

}  /* namespace util */
}  /* namespace psst */


#endif /* PUSHKIN_UTIL_DEMANGLE_HPP_ */
