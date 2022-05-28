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

%module signalStats

%{
#include <gz/math/SignalStats.hh>
%}

%include "std_string.i"
%include "std_map.i"
%template(map_string_double) std::map<std::string, double>;

namespace gz
{
  namespace math
  {
    class SignalMaximum
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalMean
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalMinimum
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalRootMeanSquare
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalMaxAbsoluteValue
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalVariance
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: virtual void Reset();
      public: virtual size_t Count() const;
      public: virtual double Value() const;
      public: virtual std::string ShortName() const;
      public: virtual void InsertData(const double _data);
    };

    class SignalStats
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: SignalStats();
      public: ~SignalStats();
      public: SignalStats(const SignalStats &_ss);
      public: size_t Count() const;
      public: std::map<std::string, double> Map() const;
      public: void InsertData(const double _data);
      public: bool InsertStatistic(const std::string &_name);
      public: bool InsertStatistics(const std::string &_names);
      public: void Reset();
    };

  }
}
