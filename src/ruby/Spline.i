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

%module spline
%{
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Spline.hh>
#include <gz/math/Vector3.hh>
%}

namespace gz
{
  namespace math
  {
    class ControlPoint;

    class Spline
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Spline();
      public: ~Spline();
      public: void Tension(double _t);
      public: double Tension() const;
      public: double ArcLength() const;
      public: double ArcLength(const double _t) const;
      public: double ArcLength(const unsigned int _index,
                               const double _t) const;
      public: void AddPoint(const Vector3<double> &_p);
      public: void AddPoint(const Vector3<double> &_p, const Vector3<double> &_t);
      private: void AddPoint(const ControlPoint &_cp, const bool _fixed);
      public: Vector3<double> Point(const unsigned int _index) const;
      public: Vector3<double> Tangent(const unsigned int _index) const;
      public: Vector3<double> MthDerivative(const unsigned int _index,
                                     const unsigned int _mth) const;
      public: size_t PointCount() const;
      public: void Clear();
      public: bool UpdatePoint(const unsigned int _index,
                               const Vector3<double> &_p);
      public: bool UpdatePoint(const unsigned int _index,
                               const Vector3<double> &_p,
                               const Vector3<double> &_t);
      private: bool UpdatePoint(const unsigned int _index,
                                const ControlPoint &_cp,
                                const bool _fixed);
      public: Vector3<double> Interpolate(const double _t) const;
      public: Vector3<double> Interpolate(const unsigned int _fromIndex,
                                   const double _t) const;
      public: Vector3<double> InterpolateTangent(const double _t) const;
      public: Vector3<double> InterpolateTangent(const unsigned int _fromIndex,
                                          const double _t) const;
      public: Vector3<double> InterpolateMthDerivative(const unsigned int _mth,
                                                const double _1) const;
      public: Vector3<double> InterpolateMthDerivative(const unsigned int _fromIndex,
                                                const unsigned int _mth,
                                                const double _s) const;
      public: void AutoCalculate(bool _autoCalc);
      public: void RecalcTangents();
    };
  }
}
