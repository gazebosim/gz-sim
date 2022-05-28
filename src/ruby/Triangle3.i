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

%module triangle3
%{
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Line3.hh>
#include <set>
#include <gz/math/Triangle3.hh>
#include <gz/math/Vector3.hh>
%}

namespace gz
{
namespace math
{
    template<typename T>
    class Triangle3
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Triangle3() = default;
      public: Triangle3(const gz::math::Vector3<T> &_pt1,
                        const gz::math::Vector3<T> &_pt2,
                        const gz::math::Vector3<T> &_pt3);
      public: void Set(const unsigned int _index, const math::Vector3<T> &_pt);
      public: void Set(const gz::math::Vector3<T> &_pt1,
                       const gz::math::Vector3<T> &_pt2,
                       const gz::math::Vector3<T> &_pt3);
      public: bool Valid() const;
      public: Line3<T> Side(const unsigned int _index) const;
      public: bool Contains(const Line3<T> &_line) const;
      public: bool Contains(const gz::math::Vector3<T> &_pt) const;
      public: gz::math::Vector3d Normal() const;
      public: bool Intersects(const Line3<T> &_line,
                              gz::math::Vector3<T> &_ipt1) const;
      public: T Perimeter() const;
      public: double Area() const;
    };

    %extend Triangle3
    {
      gz::math::Vector3<T> __getitem__(const unsigned int i) const
      {
        return (*$self)[i];
      }
    }

    %template(Triangle3i) Triangle3<int>;
    %template(Triangle3d) Triangle3<double>;
    %template(Triangle3f) Triangle3<float>;
}
}
