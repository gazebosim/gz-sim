/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_SIM_CONSTANTS_HH_
#define GZ_SIM_CONSTANTS_HH_

#include "gz/sim/config.hh"
#include <string_view>

namespace gz::sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  namespace  config_action {
    constexpr std::string_view kPluginAttribute{"gz:config_action"};
    constexpr std::string_view kPrependReplace{"prepend_replace"};
    constexpr std::string_view kPrepend{"prepend"};
    constexpr std::string_view kAppendReplace{"append_replace"};
    constexpr std::string_view kAppend{"append"};
  }
}
}  // namespace gz::sim

#endif