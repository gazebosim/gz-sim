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

%module triangle
%{
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Line2.hh>
#include <set>
#include <gz/math/Triangle.hh>
#include <gz/math/Vector2.hh>
%}

namespace gz
{
namespace math
{
    template<typename T>
    class Triangle
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Triangle() = default;
      public: Triangle(const math::Vector2<T> &_pt1,
                       const math::Vector2<T> &_pt2,
                       const math::Vector2<T> &_pt3);
      public: void Set(const unsigned int _index, const math::Vector2<T> &_pt);
      public: void Set(const math::Vector2<T> &_pt1,
                       const math::Vector2<T> &_pt2,
                       const math::Vector2<T> &_pt3);
      public: bool Valid() const;
      public: Line2<T> Side(const unsigned int _index) const;
      public: bool Contains(const Line2<T> &_line) const;
      public: bool Contains(const math::Vector2<T> &_pt) const;
      public: bool Intersects(const Line2<T> &_line,
                              math::Vector2<T> &_ipt1,
                              math::Vector2<T> &_ipt2) const;
      public: T Perimeter() const;
      public: double Area() const;
    };

    %extend Triangle
    {
      gz::math::Vector2<T> __getitem__(const unsigned int i) const
      {
        return (*$self)[i];
      }
    }

    %template(Trianglei) Triangle<int>;
    %template(Triangled) Triangle<double>;
    %template(Trianglef) Triangle<float>;
}
}
