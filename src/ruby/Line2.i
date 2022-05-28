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

%module line2
%{
#include <sstream>
#include <gz/math/Line2.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector2.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    template<typename T>
    class Line2
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Line2(const math::Vector2<T> &_ptA, const math::Vector2<T> &_ptB);
      public: Line2(double _x1, double _y1, double _x2, double _y2);
      public: void Set(const math::Vector2<T> &_ptA,
                       const math::Vector2<T> &_ptB);
      public: void Set(double _x1, double _y1, double _x2, double _y2);
      public: double CrossProduct(const Line2<T> &_line) const;
      public: double CrossProduct(const Vector2<T> &_pt) const;
      public: bool Collinear(const math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const;
      public: bool Parallel(const math::Line2<T> &_line,
                            double _epsilon = 1e-6) const;
      public: bool Collinear(const math::Line2<T> &_line,
                             double _epsilon = 1e-6) const;
      public: bool OnSegment(const math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const;
      public: bool Within(const math::Vector2<T> &_pt,
                          double _epsilon = 1e-6) const;
      public: bool Intersect(const Line2<T> &_line,
                             double _epsilon = 1e-6) const;
      public: bool Intersect(const Line2<T> &_line, math::Vector2<T> &_pt,
                             double _epsilon = 1e-6) const;
      public: T Length() const;
      public: double Slope() const;
      public: bool operator==(const Line2<T> &_line) const;
      public: bool operator!=(const Line2<T> &_line) const;
    };

    %extend Line2
    {
      gz::math::Vector2<T> __getitem__(unsigned int i) const
      {
        return (*$self)[i];
      }
    }

    %extend Line2
    {
      std::string __str__() const {
        std::ostringstream out;
        out << *$self;
        return out.str();
      }
    }

    %template(Line2i) Line2<int>;
    %template(Line2d) Line2<double>;
    %template(Line2f) Line2<float>;
  }
}
