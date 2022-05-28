/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module stopwatch
%{
#include <chrono>
#include <memory>
#include "gz/math/Stopwatch.hh"
#include <gz/math/Export.hh>
#include <gz/math/config.hh>
%}

%include "typemaps.i"
%typemap(out) (std::chrono::steady_clock::time_point) %{
  $result = SWIG_From_long(std::chrono::duration_cast<std::chrono::milliseconds>(
    (&result)->time_since_epoch()).count());
%}

%typemap(out) (std::chrono::steady_clock::duration) %{
  $result = SWIG_From_long((&result)->count());
%}

namespace gz
{
  namespace math
  {
    class Stopwatch
    {
      public: Stopwatch();

      public: virtual ~Stopwatch();

      public: bool Start(const bool _reset = false);

      public: std::chrono::steady_clock::time_point StartTime() const;

      public: bool Stop();

      public: std::chrono::steady_clock::time_point StopTime() const;

      public: bool Running() const;

      public: void Reset();

      public: std::chrono::steady_clock::duration ElapsedRunTime() const;

      public: std::chrono::steady_clock::duration ElapsedStopTime() const;

      public: bool operator==(const Stopwatch &_watch) const;

      public: bool operator!=(const Stopwatch &_watch) const;
    };
  }
}
