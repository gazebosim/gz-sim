/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

%module angle
%{
#include <ignition/math/Angle.hh>
%}

namespace ignition
{
  namespace math
  {
    class Angle
    {
      public: static const Angle Zero;
      public: static const Angle Pi;
      public: static const Angle HalfPi;
      public: static const Angle TwoPi;
      public: Angle();
      public: Angle(double _radian);
      public: Angle(const Angle &_angle);
      public: virtual ~Angle();
      public: void Radian(double _radian);
      public: void Degree(double _degree);
      public: double Radian() const;
      public: double Degree() const;
      public: void Normalize();
      public: inline double operator*() const;
      public: Angle operator-(const Angle &_angle) const;
      public: Angle operator+(const Angle &_angle) const;
      public: Angle operator*(const Angle &_angle) const;
      public: Angle operator/(const Angle &_angle) const;
      public: bool operator==(const Angle &_angle) const;
      public: bool operator<(const Angle &_angle) const;
      public: bool operator<=(const Angle &_angle) const;
      public: bool operator>(const Angle &_angle) const;
      public: bool operator>=(const Angle &_angle) const;
    };
  }
}
