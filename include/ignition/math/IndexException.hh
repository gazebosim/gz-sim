/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_INDEX_EXCEPTION_HH_
#define IGNITION_MATH_INDEX_EXCEPTION_HH_

#include <stdexcept>
#include <ignition/math/Helpers.hh>

// Ignore warning C4275. It is okay to ignore this according to microsoft:
// https://msdn.microsoft.com/en-us/library/3tdb471s.aspx
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4275)
#endif

namespace ignition
{
  namespace math
  {
    /// \class IndexException IndexException.hh ignition/math/IndexException.hh
    /// \brief Exception that is thrown when an out-of-bounds index is
    /// encountered.
    class IGNITION_VISIBLE IndexException : public std::runtime_error
    {
      public: IndexException();
    };
  }
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
