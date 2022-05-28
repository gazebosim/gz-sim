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

%module line3
%{
#include <sstream>
#include <gz/math/Line3.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    template<typename T>
    class Line3
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Line3() = default;
      public: Line3(const Line3<T> &_line);
      public: Line3(const math::Vector3<T> &_ptA, const math::Vector3<T> &_ptB);
      public: Line3(const double _x1, const double _y1,
          const double _x2, const double _y2);
      public: Line3(const double _x1, const double _y1,
          const double _z1, const double _x2,
          const double _y2, const double _z2);
      public: void Set(const math::Vector3<T> &_ptA,
                      const math::Vector3<T> &_ptB);
      public: void SetA(const math::Vector3<T> &_ptA);
      public: void SetB(const math::Vector3<T> &_ptB);
      public: void Set(const double _x1, const double _y1,
          const double _x2, const double _y2,
          const double _z = 0);
      public: void Set(const double _x1, const double _y1,
          const double _z1, const double _x2,
          const double _y2, const double _z2);
      public: math::Vector3<T> Direction() const;
      public: T Length() const;
      public: bool Distance(const Line3<T> &_line, Line3<T> &_result,
                              const double _epsilon = 1e-6) const;
      public: bool Intersect(const Line3<T> &_line,
                              double _epsilon = 1e-6) const;
      public: bool Coplanar(const Line3<T> &_line,
                              const double _epsilon = 1e-6) const;
      public: bool Parallel(const Line3<T> &_line,
                              const double _epsilon = 1e-6) const;
      public: bool Intersect(const Line3<T> &_line, math::Vector3<T> &_pt,
                              double _epsilon = 1e-6) const;
      public: bool Within(const math::Vector3<T> &_pt,
                          double _epsilon = 1e-6) const;
      public: bool operator==(const Line3<T> &_line) const;
      public: bool operator!=(const Line3<T> &_line) const;
    };

    %extend Line3
    {
      gz::math::Vector3<T> __getitem__(const unsigned int i) const
      {
        return (*$self)[i];
      }
    }

    %extend Line3
    {
      std::string __str__() const {
        std::ostringstream out;
        out << *$self;
        return out.str();
      }
    }

    %template(Line3i) Line3<int>;
    %template(Line3d) Line3<double>;
    %template(Line3f) Line3<float>;
    }
}
