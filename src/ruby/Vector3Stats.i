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

%module vector3stats
%{
#include <gz/math/Helpers.hh>
#include <gz/math/SignalStats.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector3Stats.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    class Vector3Stats
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Vector3Stats();
      public: ~Vector3Stats();
      public: void InsertData(const Vector3<double> &_data);
      public: bool InsertStatistic(const std::string &_name);
      public: bool InsertStatistics(const std::string &_names);
      public: void Reset();
      public: const SignalStats &X() const;
      public: const SignalStats &Y() const;
      public: const SignalStats &Z() const;
      public: const SignalStats &Mag() const;
      public: SignalStats &X();
      public: SignalStats &Y();
      public: SignalStats &Z();
      public: SignalStats &Mag();
    };
  }
}
