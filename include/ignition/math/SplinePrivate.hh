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
#ifndef IGNITION_MATH_SPLINEPRIVATE_HH_
#define IGNITION_MATH_SPLINEPRIVATE_HH_

#include <algorithm>
#include <vector>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <ignition/math/Matrix4.hh>

namespace ignition
{
  namespace math
  {
    class ControlPoint
    {
      /// \brief Constructor that takes the M derivatives that
      /// define the control point
      /// \param[in] _initList with the M derivatives
      public: explicit ControlPoint(const std::vector<Vector3d> &_initList)
          : derivatives(_initList.begin(), _initList.end())
      {
      }

      /// \brief Matches all mth derivatives defined in \p _other
      /// to this.
      /// \remarks Higher order derivatives in this and not defined
      /// in \p _other are kept.
      /// \param[in] _other control point to be matches
      /// \return this control point
      public: inline void Match(const ControlPoint &_other)
      {
        std::copy(_other.derivatives.begin(),
                  _other.derivatives.end(),
                  this->derivatives.begin());
      }

      /// \brief Checks for control point equality.
      /// \param[in] _other control point to compare against
      /// \return whether this and \p _other can be seen as equal
      public: inline bool operator==(const ControlPoint &_other)
      {
        return (this->derivatives == _other.derivatives);
      }

      /// \brief Returns the mth derivative of
      /// this control point.
      /// \remarks Higher derivatives than those defined
      /// default to [0.0, 0.0, 0.0].
      /// \param[in] _mth derivative order
      /// \return The mth derivative.
      public: inline Vector3d MthDerivative(const unsigned int _mth) const
      {
        if (_mth >= this->derivatives.size())
          return Vector3d(0.0, 0.0, 0.0);
        return this->derivatives[_mth];
      }

      /// \brief Returns a mutable reference to the mth derivative of
      /// this control point.
      /// \remarks Higher derivatives than those defined
      /// default to [0.0, 0.0, 0.0].
      /// \param[in] _mth derivative order
      /// \return The mth derivative.
      public: inline Vector3d& MthDerivative(const unsigned int _mth)
      {
        if (_mth >= this->derivatives.size())
          this->derivatives.insert(this->derivatives.end(),
                                   _mth - this->derivatives.size() + 1,
                                   Vector3d(0.0, 0.0, 0.0));
        return this->derivatives[_mth];
      }

      /// \brief control point M derivatives (0 to M-1)
      private: std::vector<Vector3d> derivatives;
    };

    class CurveInterpolator
    {
      /// \brief Interpolates the mth derivative based on a parametric value.
      /// \param[in] _mth order of curve derivative to interpolate
      /// \param[in] _t parameter value (range 0 to 1)
      /// \return The interpolated mth derivative, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: virtual Vector3d InterpolateMthDerivative(
          const unsigned int _mth, const double _t) const = 0;

      /// \brief Returns this curve arc length
      public: virtual double ArcLength() const = 0;
    };

    class IntervalCubicSpline : public CurveInterpolator
    {
      /// \brief Dummy constructor.
      public: IntervalCubicSpline();

      /// \brief Constructor that takes the control points for
      /// interpolation.
      /// \param[in] _startPoint start control point
      /// \param[in] _endPoint end control point
      public: explicit IntervalCubicSpline(const ControlPoint &_startPoint,
                                           const ControlPoint &_endPoint);

      /// \brief Sets both control points for this interpolator
      /// \param[in] _startPoint start control point
      /// \param[in] _endPoint end control point
      public: void SetPoints(const ControlPoint &_startPoint,
                             const ControlPoint &_endPoint);

      /// \brief Returns the start control point for this interpolator
      /// \return the start control point
      public: inline const ControlPoint& StartPoint() const
      {
        return this->startPoint;
      };

      /// \brief Returns the end control point for this interpolator
      /// \return the end control point
      public: inline const ControlPoint& EndPoint() const
      {
        return this->endPoint;
      };

      /// \brief Interpolates the mth derivative based on a parametric value.
      /// \param[in] _mth order of curve derivative to interpolate
      /// \param[in] _t arc length parameter value (range 0 to 1)
      /// \return The interpolated mth derivative, or
      /// [INF, INF, INF] on error. Use Vector3d::IsFinite() to
      /// check for an error.
      public: virtual Vector3d InterpolateMthDerivative(
          const unsigned int _mth, const double _t) const override;

      /// \brief Returns this spline arc length
      public: inline virtual double ArcLength() const override
      {
        return this->arcLength;
      }

      /// \internal
      /// \brief Computes this spline arc length
      /// \return arc length
      private: double ComputeArcLength() const;

      /// \brief start control point for the curve
      private: ControlPoint startPoint;

      /// \brief end control point for the curve
      private: ControlPoint endPoint;

      /// \brief Bernstein-Hermite polynomial coefficients
      /// for interpolation
      private: Matrix4d coeffs;

      /// \brief spline arc length
      private: double arcLength;
    };

    class InverseArcLengthInterpolator
    {
      /// \brief Dummy constructor
      public: InverseArcLengthInterpolator();

      /// \brief Constructor that takes the \p _spline to scale for
      /// \param[in] _spline to scale the inverse arc length function
      public: explicit InverseArcLengthInterpolator(
          const CurveInterpolator &_interpolator);

      /// \brief Rescales this inverse function to match the given
      /// \p _spline arc length
      /// \param[in] _spline to scale the inverse arc length function
      public: void Rescale(const CurveInterpolator &_interpolator);

      /// \brief Interpolates the \p _mth derivative of the inverse arc length
      /// function based on the arc length parametric value \p _s.
      /// \param[in] _mth order of curve derivative to interpolate
      /// \param[in] _s arc length parameter value (range 0 to length)
      /// \return The interpolated mth derivative of the inverse function
      public: double InterpolateMthDerivative(const unsigned int _mth,
                                              const double _s) const;

      /// \brief full arc length for rescaling
      private: double arcLength;
    };

    class SplinePrivate
    {
      /// \brief when true, the tangents are recalculated when the control
      /// point change
      public: bool autoCalc;

      /// Tension of 0 = Catmull-Rom spline, otherwise a Cardinal spline
      public: double tension;

      /// \brief fixings for control points
      public: std::vector<bool> fixings;

      /// \brief control points
      public: std::vector<ControlPoint> points;

      // \brief interpolated arcs
      public: std::vector<IntervalCubicSpline> segments;

      // \brief segments arc length parameterizations
      public: std::vector<InverseArcLengthInterpolator> params;

      // \brief segments arc length cumulative distribution
      public: std::vector<double> cumulativeArcLengths;

      // \brief full spline arc length
      public: double arcLength;
    };
  }
}

#endif
