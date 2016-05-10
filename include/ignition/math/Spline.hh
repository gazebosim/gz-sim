/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <ignition/math/IndexException.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declare private data class
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

      /// \brief  Adds a control point to the end of the spline.
      /// \param[in] _pt point to add
      public: void AddPoint(const Vector3d &_pt);

      /// \brief Gets the detail of one of the control points of the spline.
      /// \param[in] _index the control point index
      /// \return the control point, or [0,0,0] and a message on the error
      /// stream
      /// \throws IndexException if _index >= PointCount()
      public: Vector3d Point(unsigned int _index) const;

      /// \brief  Gets the number of control points in the spline.
      /// \return the count
      public: size_t PointCount() const;

      /// \brief Get the tangent value for a point
      /// \param[in] _index the control point index
      /// \throws IndexException if _index >= PointCount()
      public: Vector3d Tangent(unsigned int _index) const;

      /// \brief  Clears all the points in the spline.
      public: void Clear();

      /// \brief Updates a single point in the spline.
      /// \remarks an error to the error stream is printed when the index is
      /// out of bounds
      /// \param[in] _index the control point index
      /// \param[in] _value the new position
      /// \throws IndexException if _index >= PointCount()
      public: void UpdatePoint(unsigned int _index, const Vector3d &_value);

      /// \brief Returns an interpolated point based on a parametric value
      ///        over the whole series.
      /// \param[in] _t parameter (range 0 to 1)
      public: Vector3d Interpolate(double _t) const;

      /// \brief Interpolates a single segment of the spline given a
      ///        parametric value.
      /// \param[in] _fromIndex The point index to treat as t = 0.
      ///        fromIndex + 1 is deemed to be t = 1
      /// \param[in] _t Parametric value
      /// \throws IndexException if _fromIndex >= PointCount()
      public: Vector3d Interpolate(unsigned int _fromIndex, double _t) const;

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
      ///        a point changes. If false, you must call reclacTangents to
      ///        recalculate them when it best suits.
      public: void AutoCalculate(bool _autoCalc);

      /// \brief Recalculates the tangents associated with this spline.
      /// \remarks If you tell the spline not to update on demand by
      ///          calling setAutoCalculate(false) then you must call this
      ///          after completing your updates to the spline points.
      public: void RecalcTangents();

      /// \internal
      /// \brief Private data pointer
      private: SplinePrivate *dataPtr;
    };
  }
}
#endif
