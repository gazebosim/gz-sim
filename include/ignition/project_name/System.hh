/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef IGNITION_<PROJECT-NAME>_SYSTEM_HH_
#define IGNITION_<PROJECT-NAME>_SYSTEM_HH_

#if defined(__GNUC__)
#define IGNITION_<PROJECT-NAME>_DEPRECATED(version) __attribute__((deprecated))
#define IGNITION_<PROJECT-NAME>_FORCEINLINE __attribute__((always_inline))
#elif defined(_WIN32)
#define IGNITION_<PROJECT-NAME>_DEPRECATED(version) ()
#define IGNITION_<PROJECT-NAME>_FORCEINLINE __forceinline
#else
#define IGNITION_<PROJECT-NAME>_DEPRECATED(version) ()
#define IGNITION_<PROJECT-NAME>_FORCEINLINE
#endif

/// \def IGNITION_<PROJECT-NAME>_VISIBLE
/// Use to represent "symbol visible" if supported

/// \def IGNITION_<PROJECT-NAME>_HIDDEN
/// Use to represent "symbol hidden" if supported
#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define IGNITION_<PROJECT-NAME>_VISIBLE __attribute__ ((dllexport))
    #else
      #define IGNITION_<PROJECT-NAME>_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define IGNITION_<PROJECT-NAME>_VISIBLE __attribute__ ((dllimport))
    #else
      #define IGNITION_<PROJECT-NAME>_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define IGNITION_<PROJECT-NAME>_HIDDEN
#else
  #if __GNUC__ >= 4
    #define IGNITION_<PROJECT-NAME>_VISIBLE __attribute__ ((visibility ("default")))
    #define IGNITION_<PROJECT-NAME>_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define IGNITION_<PROJECT-NAME>_VISIBLE
    #define IGNITION_<PROJECT-NAME>_HIDDEN
  #endif
#endif

#endif
