/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gz/math/Helpers.hh"

#include <iomanip>
#include <regex>
#include <sstream>

#include <gz/utils/NeverDestroyed.hh>

namespace gz
{
  namespace math
  {
    inline namespace GZ_MATH_VERSION_NAMESPACE
    {
    /////////////////////////////////////////////
    int parseInt(const std::string &_input)
    {
      // Return NAN_I if it is empty
      if (_input.empty())
      {
        return NAN_I;
      }
      // Return 0 if it is all spaces
      else if (_input.find_first_not_of(' ') == std::string::npos)
      {
        return 0;
      }

      // Otherwise try standard library
      try
      {
        return std::stoi(_input);
      }
      // if that fails, return NAN_I
      catch(...)
      {
        return NAN_I;
      }
    }

    /////////////////////////////////////////////
    double parseFloat(const std::string &_input)
    {
      // Return NAN_D if it is empty
      if (_input.empty())
      {
        return NAN_D;
      }
      // Return 0 if it is all spaces
      else if (_input.find_first_not_of(' ') == std::string::npos)
      {
        return 0;
      }

      // Otherwise try standard library
      try
      {
        return std::stod(_input);
      }
      // if that fails, return NAN_D
      catch(...)
      {
        return NAN_D;
      }
    }

    /////////////////////////////////////////////
    std::pair<int64_t, int64_t> timePointToSecNsec(
        const std::chrono::steady_clock::time_point &_time)
    {
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _time.time_since_epoch());
      auto now_s = std::chrono::duration_cast<std::chrono::seconds>(
        _time.time_since_epoch());
      int64_t seconds = now_s.count();
      int64_t nanoseconds = std::chrono::duration_cast
        <std::chrono::nanoseconds>(now_ns - now_s).count();
      return {seconds, nanoseconds};
    }

    /////////////////////////////////////////////
    std::chrono::steady_clock::time_point secNsecToTimePoint(
        const uint64_t &_sec, const uint64_t &_nanosec)
    {
      auto duration = std::chrono::seconds(_sec) + std::chrono::nanoseconds(
        _nanosec);
      std::chrono::steady_clock::time_point result;
      using std::chrono::duration_cast;
      result += duration_cast<std::chrono::steady_clock::duration>(duration);
      return result;
    }

    /////////////////////////////////////////////
    std::chrono::steady_clock::duration secNsecToDuration(
        const uint64_t &_sec, const uint64_t &_nanosec)
    {
      return std::chrono::seconds(_sec) + std::chrono::nanoseconds(
        _nanosec);
    }

    /////////////////////////////////////////////
    std::pair<int64_t, int64_t> durationToSecNsec(
        const std::chrono::steady_clock::duration &_dur)
    {
      auto s = std::chrono::duration_cast<std::chrono::seconds>(_dur);
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_dur-s);
      return {s.count(), ns.count()};
    }

    /////////////////////////////////////////////
    std::string timePointToString(
        const std::chrono::steady_clock::time_point &_point)
    {
      auto duration = _point - secNsecToTimePoint(0, 0);
      auto cleanDuration = breakDownDurations<days,
                                              std::chrono::hours,
                                              std::chrono::minutes,
                                              std::chrono::seconds,
                                              std::chrono::milliseconds>(
                                                duration);
      std::ostringstream output_string;
      output_string << std::setw(2) << std::setfill('0')
                    << std::get<0>(cleanDuration).count() << " "
                    << std::setw(2) << std::setfill('0')
                    << std::get<1>(cleanDuration).count() << ":"
                    << std::setw(2) << std::setfill('0')
                    << std::get<2>(cleanDuration).count() << ":"
                    << std::setfill('0') << std::setw(6)
                    << std::fixed << std::setprecision(3)
                    << std::get<3>(cleanDuration).count() +
                       std::get<4>(cleanDuration).count()/1000.0;
      return output_string.str();
    }

    /////////////////////////////////////////////
    std::string durationToString(
        const std::chrono::steady_clock::duration &_duration)
    {
      auto cleanDuration = breakDownDurations<days,
                                              std::chrono::hours,
                                              std::chrono::minutes,
                                              std::chrono::seconds,
                                              std::chrono::milliseconds>(
                                                _duration);
      std::ostringstream outputString;
      outputString << std::setw(2) << std::setfill('0')
                    << std::get<0>(cleanDuration).count() << " "
                    << std::setw(2) << std::setfill('0')
                    << std::get<1>(cleanDuration).count() << ":"
                    << std::setw(2) << std::setfill('0')
                    << std::get<2>(cleanDuration).count() << ":"
                    << std::setfill('0') << std::setw(6)
                    << std::fixed << std::setprecision(3)
                    << std::get<3>(cleanDuration).count() +
                       std::get<4>(cleanDuration).count()/1000.0;
      return outputString.str();
    }

    /////////////////////////////////////////////
    bool splitTimeBasedOnTimeRegex(
        const std::string &_timeString,
        uint64_t & numberDays, uint64_t & numberHours,
        uint64_t & numberMinutes, uint64_t & numberSeconds,
        uint64_t & numberMilliseconds)
    {
      // The following regex takes a time string in the general format of
      // "dd hh:mm:ss.nnn" where n is milliseconds, if just one number is
      // provided, it is assumed to be seconds.
      static const gz::utils::NeverDestroyed<std::regex> timeRegex {
          "^([0-9]+ ){0,1}"                       // day:
                                                  // Any positive integer

          "(?:([1-9]:|[0-1][0-9]:|2[0-3]:){0,1}"  // hour:
                                                  // 1 - 9:
                                                  // 01 - 19:
                                                  // 20 - 23:

          "([0-9]:|[0-5][0-9]:)){0,1}"            // minute:
                                                  // 0 - 9:
                                                  // 00 - 59:

          "(?:([0-9]|[0-5][0-9]){0,1}"            // second:
                                                  // 0 - 9
                                                  // 00 - 59

          "(\\.[0-9]{1,3}){0,1})$"};              // millisecond:
                                                  // .0 - .9
                                                  // .00 - .99
                                                  // .000 - 0.999

      std::smatch matches;

      // `matches` should always be a size of 6 as there are 6 matching
      // groups in the regex.
      // 1. The whole regex
      // 2. The days
      // 3. The hours
      // 4. The minutes
      // 5. The seconds
      // 6. The milliseconds
      // We can also index them as such below.
      // Note that the space will remain in the day match, the colon
      // will remain in the hour and minute matches, and the period will
      // remain in the millisecond match
      if (!std::regex_search(_timeString, matches, timeRegex.Access()) ||
          matches.size() != 6)
        return false;

      std::string dayString = matches[1];
      std::string hourString = matches[2];
      std::string minuteString = matches[3];
      std::string secondString = matches[4];
      std::string millisecondString = matches[5];

      // Days are the only unbounded number, so check first to see if stoi
      // runs successfully
      if (!dayString.empty())
      {
        // Erase the space
        dayString.erase(dayString.length() - 1);
        try
        {
          numberDays = std::stoi(dayString);
        }
        catch (const std::out_of_range &)
        {
          return false;
        }
      }

      if (!hourString.empty())
      {
        // Erase the colon
        hourString.erase(hourString.length() - 1);
        numberHours = std::stoi(hourString);
      }

      if (!minuteString.empty())
      {
        // Erase the colon
        minuteString.erase(minuteString.length() - 1);
        numberMinutes = std::stoi(minuteString);
      }

      if (!secondString.empty())
      {
        numberSeconds = std::stoi(secondString);
      }

      if (!millisecondString.empty())
      {
        // Erase the period
        millisecondString.erase(0, 1);

        // Multiplier because "4" = 400 ms, "04" = 40 ms, and "004" = 4 ms
        numberMilliseconds = std::stoi(millisecondString) *
          static_cast<uint64_t>(1000 / pow(10, millisecondString.length()));
      }
      return true;
    }

    /////////////////////////////////////////////
    std::chrono::steady_clock::duration stringToDuration(
        const std::string &_timeString)
    {
      using namespace std::chrono_literals;
      std::chrono::steady_clock::duration duration{
        std::chrono::steady_clock::duration::zero()};

      if (_timeString.empty())
        return duration;

      uint64_t numberDays = 0;
      uint64_t numberHours = 0;
      uint64_t numberMinutes = 0;
      uint64_t numberSeconds = 0;
      uint64_t numberMilliseconds = 0;

      if (!splitTimeBasedOnTimeRegex(_timeString, numberDays, numberHours,
                                     numberMinutes, numberSeconds,
                                     numberMilliseconds))
      {
        return duration;
      }

      // TODO(anyone): Replace below day conversion with std::chrono::days.
      /// This will exist in C++-20
      duration = std::chrono::steady_clock::duration::zero();
      auto delta = std::chrono::milliseconds(numberMilliseconds) +
        std::chrono::seconds(numberSeconds) +
        std::chrono::minutes(numberMinutes) +
        std::chrono::hours(numberHours) +
        std::chrono::hours(24 * numberDays);
      duration += delta;

      return duration;
    }

    /////////////////////////////////////////////
    std::chrono::steady_clock::time_point stringToTimePoint(
        const std::string &_timeString)
    {
      using namespace std::chrono_literals;
      std::chrono::steady_clock::time_point timePoint{-1s};

      if (_timeString.empty())
        return timePoint;

      uint64_t numberDays = 0;
      uint64_t numberHours = 0;
      uint64_t numberMinutes = 0;
      uint64_t numberSeconds = 0;
      uint64_t numberMilliseconds = 0;

      if (!splitTimeBasedOnTimeRegex(_timeString, numberDays, numberHours,
                                     numberMinutes, numberSeconds,
                                     numberMilliseconds))
      {
        return timePoint;
      }

      // TODO(anyone): Replace below day conversion with std::chrono::days.
      /// This will exist in C++-20
      timePoint = math::secNsecToTimePoint(0, 0);
      auto duration = std::chrono::milliseconds(numberMilliseconds) +
        std::chrono::seconds(numberSeconds) +
        std::chrono::minutes(numberMinutes) +
        std::chrono::hours(numberHours) +
        std::chrono::hours(24 * numberDays);
      timePoint += duration;

      return timePoint;
    }

    /////////////////////////////////////////////
    PairOutput Pair(const PairInput _a, const PairInput _b)
    {
      // Store in 64bit local variable so that we don't overflow.
      uint64_t a = _a;
      uint64_t b = _b;

      // Szudzik's function
      return _a >= _b ?
              static_cast<PairOutput>(a * a + a + b) :
              static_cast<PairOutput>(a + b * b);
    }

    /////////////////////////////////////////////
    std::tuple<PairInput, PairInput> Unpair(const PairOutput _key)
    {
      // Accurate 64-bit integer sqrt
      //  From https://stackoverflow.com/a/18501209
      //
      // \todo: Remove this ifdef and use the "0x1p-20" version when c++17
      // is used. All platforms should then support the "p" literal.
      //
      // Note that the of Hexadecimal floats is failing in some Linux g++
      // compilers from 6.x/7.x series. The native support for C++ is defined
      // in C++17 See https://bugzilla.redhat.com/show_bug.cgi?id=1321986
#if (__cplusplus < 201703L)
      uint64_t sqrt = static_cast<uint64_t>(std::sqrt(_key) -9.53674e-07);
#else
      uint64_t sqrt = static_cast<uint64_t>(std::sqrt(_key) - 0x1p-20);
#endif
      if (2 * sqrt < _key - sqrt * sqrt)
        sqrt++;

      uint64_t sq = sqrt * sqrt;

      return ((_key - sq) >= sqrt) ?
        std::make_tuple(static_cast<PairInput>(sqrt),
                        static_cast<PairInput>(_key - sq - sqrt)) :
        std::make_tuple(static_cast<PairInput>(_key - sq),
                        static_cast<PairInput>(sqrt));
    }
    }
  }
}
