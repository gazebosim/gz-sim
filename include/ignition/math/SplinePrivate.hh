/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_SPLINE_PRIVATE_HH_
#define IGNITION_MATH_SPLINE_PRIVATE_HH_

#include <vector>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix4.hh>

namespace ignition
{
  namespace math
  {
    class SplinePrivate
    {
      /// \brief when true, the tangents are recalculated when the control
      /// point change
      public: bool autoCalc;

      /// \brief control points
      public: std::vector<Vector3d> points;

      /// \brief tangents
      public: std::vector<Vector3d> tangents;

      /// Matrix of coefficients
      public: Matrix4d coeffs;

      /// Tension of 0 = Catmull-Rom spline, otherwise a Cardinal spline
      public: double tension;
    };
  }
}

#endif
