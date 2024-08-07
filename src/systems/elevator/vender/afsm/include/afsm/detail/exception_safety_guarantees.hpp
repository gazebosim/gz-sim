/*
 * exception_safety_guarantees.hpp
 *
 *  Created on: Nov 30, 2016
 *      Author: zmij
 */

#ifndef AFSM_DETAIL_EXCEPTION_SAFETY_GUARANTEES_HPP_
#define AFSM_DETAIL_EXCEPTION_SAFETY_GUARANTEES_HPP_


namespace afsm {
namespace def {
namespace tags {

/**
 * Default exception safety guarantee
 * On an exception caused by
 *  * a guard check nothing happens
 *  * state exit action - only data modified by exit action before exception
 *  * transition action - previous and data modified by the transition action
 *  * state enter action - previous and data modified by the enter action
 *  * source state constructor - all of the above
 */
struct basic_exception_safety {};
/**
 * Strong exception safety guarantee
 * If an exception is thrown in the course of transition, nothing will be modified.
 *
 * Requires non-throwing swap operations
 * The exception will be thrown.
 */
struct strong_exception_safety {};
/**
 * Same as the strong exception safety, but the exception will be consumed
 * The event will be rejected and further behavior is ruled by event rejection
 * policies.
 */
struct nothrow_guarantee {};

}  /* namespace tags */
}  /* namespace def */
}  /* namespace afsm */


#endif /* AFSM_DETAIL_EXCEPTION_SAFETY_GUARANTEES_HPP_ */
