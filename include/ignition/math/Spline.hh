/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
// Note: Originally cribbed from Ogre3d. Modified to implement Cardinal
// spline and catmull-rom spline
#ifndef IGNITION_MATH_SPLINE_HH_
#define IGNITION_MATH_SPLINE_HH_

#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declare private data classes
    class ControlPoint;
    class SplinePrivate;

    /// \class Spline Spline.hh ignition/math/Spline.hh
    /// \brief Splines
    class IGNITION_VISIBLE Spline
    {
      /// \brief constructor
      public: Spline();

      /// \brief destructor
      public: ~Spline();

      /// \brief Set the tension parameter. A value of 0 = Catmull-Rom
      /// spline.
      /// \param[in] _t Tension value between 0.0 and 1.0
      public: void Tension(double _t);

      /// \brief Get the tension value
      /// \return The value of the tension, which is between 0.0 and 1.0
      public: double Tension() const;

      /// \brief Gets the spline arc length
      public: double ArcLength() const;

      /// \brief Gets the spline arc length
      /// \param[in] _t optional parameter value (range 0 to 1)
      /// \return The value of arc length up to \p _t
      public: double ArcLength(const double _t) const;

      /// \brief Gets a spline segment arc length
      /// \param[in] _index of the spline segment
      /// \param[in] _t parameter value (range 0 to 1)
      /// \return The value of given segment arc length up to
      /// \p _t or INF on error.
      public: double ArcLength(const unsigned int _index,
                               const double _t) const;

      /// \brief  Adds a single control point to the end of the spline.
      /// \param[in] _pt control point value to add
      public: void AddPoint(const Vector3d &_pt);

      /// \brief  Adds a single control point to the end of the spline
      ///         with fixed tangent.
      /// \param[in] _p control point value to add
      /// \param[in] _t tangent at \p _p
      public: void AddPoint(const Vector3d &_p, const Vector3d &_t);

      /// \brief  Adds a single control point to the end of the spline.
      /// \param[in] _cp control point to add
      /// \param[in] _fixed whether this control point should not be
      ///                   subject to tangent recomputation
      private: void AddPoint(const ControlPoint &_cp, const bool _fixed);

      /// \brief Gets the value for one of the control points
      /// of the spline.
      /// \param[in] _index the control point index
      /// \return the control point value, or [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d Point(const unsigned int _index) const;

      /// \brief Gets the tangent value for one of the control points
      /// of the spline.
      /// \param[in] _index the control point index
      /// \return the control point tangent, or [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d Tangent(const unsigned int _index) const;

      /// \brief Gets the mth derivative for one of the control points
      /// of the spline.
      /// \param[in] _index the control point index
      /// \param[in] _mth derivative order
      /// \return the control point mth derivative, or [INF, INF, INF]
      ///  on error. Use Vector3d::IsFinite() to check for an error.
      public: Vector3d MthDerivative(const unsigned int _index,
                                     const unsigned int _mth) const;

      /// \brief  Gets the number of control points in the spline.
      /// \return the count
      public: size_t PointCount() const;

      /// \brief  Clears all the points in the spline.
      public: void Clear();

      /// \brief Updates a single control point value in the spline,
      ///        keeping its tangent.
      /// \remarks an error to the error stream is printed when the index is
      /// out of bounds
      /// \param[in] _index the control point index
      /// \param[in] _p the new control point value
      /// \return True on success.
      public: bool UpdatePoint(const unsigned int _index,
                               const Vector3d &_p);

      /// \brief Updates a single control point in the spline, along with its
      ///        tangent.
      /// \remarks An error to the error stream is printed when the index is
      /// out of bounds
      /// \param[in] _index the control point index
      /// \param[in] _p the new control point value
      /// \param[in] _t the new control point tangent
      /// \return True on success.
      public: bool UpdatePoint(const unsigned int _index,
                               const Vector3d &_p,
                               const Vector3d &_t);

      /// \brief Updates a single control point in the spline.
      /// \remarks an error to the error stream is printed when the index is
      /// out of bounds
      /// \param[in] _index the control point index
      /// \param[in] _cp the new control point
      /// \param[in] _fixed whether the new control point should not be
      /// subject to tangent recomputation
      /// \return True on success.
      private: bool UpdatePoint(const unsigned int _index,
                                const ControlPoint &_cp,
                                const bool _fixed);

      /// \brief Returns an interpolated point based on an
      ///        arc-length parametric value, normalized over
      ///        the whole series.
      /// \param[in] _t parameter (range 0 to 1)
      /// \return The interpolated point, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d Interpolate(const double _t) const;

      /// \brief Interpolates a point at a single segment of the spline
      ///        based on an arc-length parametric value, normalized
      ///        over such segment.
      /// \param[in] _fromIndex The point index to treat as t = 0.
      ///        fromIndex + 1 is deemed to be t = 1
      /// \param[in] _t parameter (range 0 to 1)
      /// \return The interpolated point, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d Interpolate(const unsigned int _fromIndex,
                                   const double _t) const;

      /// \brief Returns an interpolated tangent based on an
      ///        arc-length parametric value, normalized over
      ///        the whole series.
      /// \param[in] _t parameter (range 0 to 1)
      /// \return The interpolated point, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinte() to check for an error.
      public: Vector3d InterpolateTangent(const double _t) const;

      /// \brief Interpolates the tangent at a single segment of
      ///        the spline based on an arc length parametric value,
      ///        normalized over such segment.
      /// \param[in] _fromIndex The point index to treat as t = 0.
      ///        fromIndex + 1 is deemed to be t = 1
      /// \param[in] _t parameter (range 0 to 1)
      /// \return The interpolated point, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinte() to check for an error.
      public: Vector3d InterpolateTangent(const unsigned int _fromIndex,
                                          const double _t) const;

      /// \brief Returns an interpolated mth derivative based on an
      ///        arc length parametric value over the whole series.
      /// \param[in] _mth order of curve derivative to interpolate
      /// \param[in] _s arc length parameter value (range 0 to spline length)
      /// \return The interpolated mth derivative, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d InterpolateMthDerivative(const unsigned int _mth,
                                                const double _s) const;

      /// \brief Interpolates the mth derivative at a single segment of
      ///        the spline based on an arc length parametric value.
      /// \param[in] _fromIndex point index to treat as t = 0, fromIndex + 1
      ///                       is deemed to be t = 1
      /// \param[in] _mth order of curve derivative to interpolate
      /// \param[in] _s arc length parameter value (range 0 to segment length)
      /// \return The interpolated mth derivative, or
      /// [INF, INF, INF] on error. Use
      /// Vector3d::IsFinite() to check for an error.
      public: Vector3d InterpolateMthDerivative(const unsigned int _fromIndex,
                                                const unsigned int _mth,
                                                const double _s) const;

      /// \brief Tells the spline whether it should automatically
      ///        calculate tangents on demand as points are added.
      /// \remarks The spline calculates tangents at each point
      ///          automatically based on the input points. Normally it
      ///          does this every time a point changes. However, if you
      ///          have a lot of points to add in one go, you probably
      ///          don't want to incur this overhead and would prefer to
      ///          defer the calculation until you are finished setting all
      ///          the points. You can do this by calling this method with a
      ///          parameter of 'false'. Just remember to manually call the
      ///          recalcTangents method when you are done.
      /// \param[in] _autoCalc If true, tangents are calculated for you whenever
      ///        a point changes. If false, you must call RecalcTangents to
      ///        recalculate them when it best suits.
      public: void AutoCalculate(bool _autoCalc);

      /// \brief Recalculates the tangents associated with this spline.
      /// \remarks If you tell the spline not to update on demand by
      ///          calling setAutoCalculate(false) then you must call this
      ///          after completing your updates to the spline points.
      public: void RecalcTangents();

      /// \brief Rebuilds this spline's segments.
      private: void Rebuild();

      /// \brief Maps \p _t parameter value over the whole spline
      /// to the right segment (starting at point \p _index) with
      /// the proper parameter value fraction \p _fraction.
      /// \param[in] _t parameter value over the whole spline (range 0 to 1)
      /// \param[out] _index point index at which the segment starts
      /// \param[out] _fraction parameter value fraction for the given segment
      private: void MapToSegment(const double _t,
                                 unsigned int &_index,
                                 double &_fraction) const;

      /// \internal
      /// \brief Private data pointer
      private: SplinePrivate *dataPtr;
    };
  }
}
#endif
