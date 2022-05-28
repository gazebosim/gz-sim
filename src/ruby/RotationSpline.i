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

%module rotationspline
%{
#include <gz/math/RotationSpline.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/config.hh>
%}

namespace gz
{
namespace math
{
    class RotationSpline
    {
        %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
        public: RotationSpline();
        public: ~RotationSpline();
        public: void AddPoint(const Quaternion<double> &_p);
        public: const Quaternion<double> &Point(const unsigned int _index) const;
        public: unsigned int PointCount() const;
        public: void Clear();
        public: bool UpdatePoint(const unsigned int _index,
                                 const Quaternion<double> &_value);
        public: Quaternion<double> Interpolate(double _t,
                                        const bool _useShortestPath = true);
        public: Quaternion<double> Interpolate(const unsigned int _fromIndex,
                    const double _t, const bool _useShortestPath = true);
        public: void AutoCalculate(bool _autoCalc);
        public: void RecalcTangents();
    };
}
}
