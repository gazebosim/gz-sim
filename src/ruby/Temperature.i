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

%module temperature
%{
#include <sstream>
#include <gz/math/Temperature.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    class Temperature
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Temperature();
      public: Temperature(const double _temp);
      public: Temperature(const Temperature &_temp);
      public: virtual ~Temperature();
      public: static double KelvinToCelsius(const double _temp);
      public: static double KelvinToFahrenheit(const double _temp);
      public: static double CelsiusToFahrenheit(const double _temp);
      public: static double CelsiusToKelvin(const double _temp);
      public: static double FahrenheitToCelsius(const double _temp);
      public: static double FahrenheitToKelvin(const double _temp);
      public: void SetKelvin(const double _temp);
      public: void SetCelsius(const double _temp);
      public: void SetFahrenheit(const double _temp);
      public: double Kelvin() const;
      public: double Celsius() const;
      public: double Fahrenheit() const;
      public: double operator()() const;
      public: Temperature operator+(const double _temp);
      public: Temperature operator+(const Temperature &_temp);
      public: const Temperature &operator+=(const double _temp);
      public: const Temperature &operator+=(const Temperature &_temp);
      public: Temperature operator-(const double _temp);
      public: Temperature operator-(const Temperature &_temp);
      public: const Temperature &operator-=(const double _temp);
      public: const Temperature &operator-=(const Temperature &_temp);
      public: Temperature operator*(const double _temp);
      public: Temperature operator*(const Temperature &_temp);
      public: const Temperature &operator*=(const double _temp);
      public: const Temperature &operator*=(const Temperature &_temp);
      public: Temperature operator/(const double _temp);
      public: Temperature operator/(const Temperature &_temp);
      public: const Temperature &operator/=(const double _temp);
      public: const Temperature &operator/=(const Temperature &_temp);
      public: bool operator==(const Temperature &_temp) const;
      public: bool operator==(const double _temp) const;
      public: bool operator!=(const Temperature &_temp) const;
      public: bool operator!=(const double _temp) const;
      public: bool operator<(const Temperature &_temp) const;
      public: bool operator<(const double _temp) const;
      public: bool operator<=(const Temperature &_temp) const;
      public: bool operator<=(const double _temp) const;
      public: bool operator>(const Temperature &_temp) const;
      public: bool operator>(const double _temp) const;
      public: bool operator>=(const Temperature &_temp) const;
      public: bool operator>=(const double _temp) const;
    };

    %extend Temperature
    {
      std::string __str__() const {
        std::ostringstream out;
        out << *$self;
        return out.str();
      }
    }
  }
}
