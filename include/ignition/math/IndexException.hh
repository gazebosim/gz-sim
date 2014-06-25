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
#ifndef _IGNTION_INDEX_EXCEPTION_HH_
#define _IGNTION_INDEX_EXCEPTION_HH_

#include <stdexcept>
#include <ignition/math/Helpers.hh>

namespace ignition
{
  namespace math
  {
    /// \brief Exception that is thrown when an out-of-bounds index is
    /// encountered.
    class IndexException : public std::runtime_error
    {
      public: IndexException() : std::runtime_error("Invalid index")
              {}
    };
  }
}
#endif
