/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_PLAYBACK_SCRUBBER_DURATION_LITERAL_HH_
#define GZ_SIM_GUI_PLAYBACK_SCRUBBER_DURATION_LITERAL_HH_

#include <cstdint>
#include <limits>
#include <regex>
#include <string>

namespace gz::sim::detail
{
/// \brief Parse a compact duration literal such as `200s` or `2h 19m 27s`.
/// \param[in] _input Duration literal to parse.
/// \param[out] _seconds Parsed duration in whole seconds.
/// \return True when the whole input is a valid duration literal.
inline bool ParseDurationLiteral(const std::string &_input, int64_t &_seconds)
{
  static const std::regex durationPart(R"(([0-9]+)\s*([dhms]))");
  std::sregex_iterator it(_input.begin(), _input.end(), durationPart);
  const std::sregex_iterator end;

  int64_t totalSeconds = 0;
  std::size_t consumed = 0;
  bool validDuration = false;
  for (; it != end; ++it)
  {
    const auto &match = *it;
    const std::size_t pos = static_cast<std::size_t>(match.position());
    if (_input.substr(consumed, pos - consumed).find_first_not_of(" \t") !=
        std::string::npos)
    {
      return false;
    }

    int64_t value = 0;
    try
    {
      value = std::stoll(match[1].str());
    }
    catch (const std::exception &)
    {
      return false;
    }

    int64_t multiplier = 1;
    switch (match[2].str()[0])
    {
      case 'd':
        multiplier = 24 * 60 * 60;
        break;
      case 'h':
        multiplier = 60 * 60;
        break;
      case 'm':
        multiplier = 60;
        break;
      case 's':
        break;
    }

    if (value > (std::numeric_limits<int64_t>::max() - totalSeconds) /
        multiplier)
    {
      return false;
    }

    totalSeconds += value * multiplier;
    consumed = pos + static_cast<std::size_t>(match.length());
    validDuration = true;
  }

  if (!validDuration ||
      _input.substr(consumed).find_first_not_of(" \t") != std::string::npos)
  {
    return false;
  }

  _seconds = totalSeconds;
  return true;
}
}

#endif
