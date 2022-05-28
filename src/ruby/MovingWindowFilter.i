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
%module movingwindowfilter
%{
#include "gz/math/MovingWindowFilter.hh"
#include "gz/math/Vector3.hh"
%}

namespace gz
{
namespace math
{
    template< typename T>
    class MovingWindowFilter
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: MovingWindowFilter();
        public: virtual ~MovingWindowFilter();
        public: void Update(const T _val);
        public: void SetWindowSize(const unsigned int _n);
        public: unsigned int WindowSize() const;
        public: bool WindowFilled() const;
        public: T Value() const;
    };

    %template(MovingWindowFilteri) MovingWindowFilter<int>;
    %template(MovingWindowFilterd) MovingWindowFilter<double>;
    %template(MovingWindowFilterv3) MovingWindowFilter<Vector3<double>>;
}
}
