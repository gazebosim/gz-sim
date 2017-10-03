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

#include "SplinePrivate.hh"
#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector4.hh"
#include "ignition/math/Spline.hh"

using namespace ignition;
using namespace math;

///////////////////////////////////////////////////////////
Spline::Spline()
    : dataPtr(new SplinePrivate())
{
  // Set up matrix
  this->dataPtr->autoCalc = true;
  this->dataPtr->tension = 0.0;
  this->dataPtr->arcLength = INF_D;
}

///////////////////////////////////////////////////////////
Spline::~Spline()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

///////////////////////////////////////////////////////////
void Spline::Tension(double _t)
{
  this->dataPtr->tension = _t;
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
double Spline::Tension() const
{
  return this->dataPtr->tension;
}

///////////////////////////////////////////////////////////
double Spline::ArcLength() const
{
  return this->dataPtr->arcLength;
}

///////////////////////////////////////////////////////////
double Spline::ArcLength(const double _t) const
{
  unsigned int fromIndex; double tFraction;
  if (!this->MapToSegment(_t, fromIndex, tFraction))
    return INF_D;
  return (this->dataPtr->cumulativeArcLengths[fromIndex] +
          this->ArcLength(fromIndex, tFraction));
}

///////////////////////////////////////////////////////////
double Spline::ArcLength(const unsigned int _index,
                         const double _t) const
{
  if (_index >= this->dataPtr->segments.size())
    return INF_D;
  return this->dataPtr->segments[_index].ArcLength(_t);
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p)
{
  this->AddPoint(
      ControlPoint({_p, Vector3d(INF_D, INF_D, INF_D)}), false);
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p, const Vector3d &_t)
{
  this->AddPoint(
      ControlPoint({_p, _t}), true);
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const ControlPoint &_cp, const bool _fixed)
{
  this->dataPtr->points.push_back(_cp);
  this->dataPtr->fixings.push_back(_fixed);
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
  else
    this->Rebuild();
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateMthDerivative(const unsigned int _mth,
                                          const double _t) const
{
  unsigned int fromIndex; double tFraction;
  this->MapToSegment(_t, fromIndex, tFraction);
  return this->InterpolateMthDerivative(fromIndex, _mth, tFraction);
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateMthDerivative(const unsigned int _fromIndex,
                                          const unsigned int _mth,
                                          const double _t) const
{
  // Bounds check
  if (_fromIndex >= this->dataPtr->points.size())
    return Vector3d(INF_D, INF_D, INF_D);

  if (_fromIndex == this->dataPtr->segments.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->dataPtr->points[_fromIndex].MthDerivative(_mth);
  }

  // Interpolate derivative
  return this->dataPtr->segments[_fromIndex].InterpolateMthDerivative(_mth, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(const double _t) const
{
  return this->InterpolateMthDerivative(0, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(const unsigned int _fromIndex,
                             const double _t) const
{
  return this->InterpolateMthDerivative(_fromIndex, 0, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateTangent(const double _t) const
{
  return this->InterpolateMthDerivative(1, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateTangent(const unsigned int _fromIndex,
                                    const double _t) const
{
  return this->InterpolateMthDerivative(_fromIndex, 1, _t);
}

///////////////////////////////////////////////////////////
void Spline::RecalcTangents()
{
  // Catmull-Rom approach
  //
  // tangent[i] = 0.5 * (point[i+1] - point[i-1])
  //
  // Assume endpoint tangents are parallel with line with neighbour

  size_t i, numPoints;
  bool isClosed;

  numPoints = this->dataPtr->points.size();
  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  // Closed or open?
  if (this->dataPtr->points[0].MthDerivative(0) ==
      this->dataPtr->points[numPoints-1].MthDerivative(0))
    isClosed = true;
  else
    isClosed = false;

  double t = 1.0 - this->dataPtr->tension;

  for (i = 0; i < numPoints; ++i)
  {
    if (!this->dataPtr->fixings[i])
    {
      if (i == 0)
      {
        // Special case start
        if (isClosed)
        {
          // Use this->dataPtr->points-2 since this->dataPtr->points-1
          // is the last point and == [0]
          this->dataPtr->points[i].MthDerivative(1) =
              ((this->dataPtr->points[1].MthDerivative(0) -
                this->dataPtr->points[numPoints-2].MthDerivative(0)) * 0.5) * t;
        }
        else
        {
          this->dataPtr->points[i].MthDerivative(1) =
              ((this->dataPtr->points[1].MthDerivative(0) -
                this->dataPtr->points[0].MthDerivative(0)) * 0.5) * t;
        }
      }
      else if (i == numPoints-1)
      {
        // Special case end
        if (isClosed)
        {
          // Use same tangent as already calculated for [0]
          this->dataPtr->points[i].MthDerivative(1) =
              this->dataPtr->points[0].MthDerivative(1);
        }
        else
        {
          this->dataPtr->points[i].MthDerivative(1) =
              ((this->dataPtr->points[i].MthDerivative(0) -
                this->dataPtr->points[i-1].MthDerivative(0)) * 0.5) * t;
        }
      }
      else
      {
        this->dataPtr->points[i].MthDerivative(1) =
            ((this->dataPtr->points[i+1].MthDerivative(0) -
              this->dataPtr->points[i-1].MthDerivative(0)) * 0.5) * t;
      }
    }
  }
  this->Rebuild();
}

///////////////////////////////////////////////////////////
bool Spline::MapToSegment(const double _t,
                          unsigned int &_index,
                          double &_fraction) const
{
  _index = 0;
  _fraction = 0.0;

  // Check corner cases
  if (this->dataPtr->segments.empty())
    return false;

  if (equal(_t, 0.0))
    return true;

  if (equal(_t, 1.0))
  {
    _index = static_cast<unsigned int>(this->dataPtr->segments.size()-1);
    _fraction = 1.0;
    return true;
  }

  // Assume linear relationship between t and arclength
  double tArc = _t * this->dataPtr->arcLength;

  // Get segment index where t would lie
  auto it = std::lower_bound(this->dataPtr->cumulativeArcLengths.begin(),
                             this->dataPtr->cumulativeArcLengths.end(),
                             tArc);

  if (it != this->dataPtr->cumulativeArcLengths.begin())
    _index = static_cast<unsigned int>(
        (it - this->dataPtr->cumulativeArcLengths.begin() - 1));

  // Get fraction of t, but renormalized to the segment
  _fraction = (tArc - this->dataPtr->cumulativeArcLengths[_index])
              / this->dataPtr->segments[_index].ArcLength();
  return true;
}

///////////////////////////////////////////////////////////
void Spline::Rebuild()
{
  size_t numPoints = this->dataPtr->points.size();

  if (numPoints < 2) {
    // Can't do anything yet
    return;
  }

  size_t numSegments = numPoints - 1;
  this->dataPtr->segments.resize(numSegments);
  this->dataPtr->cumulativeArcLengths.resize(numSegments);
  for (size_t i = 0 ; i < numSegments ; ++i)
  {
    this->dataPtr->segments[i].SetPoints(this->dataPtr->points[i],
                                         this->dataPtr->points[i+1]);

    if (i > 0) {
      this->dataPtr->cumulativeArcLengths[i] =
          (this->dataPtr->segments[i-1].ArcLength()
           + this->dataPtr->cumulativeArcLengths[i-1]);
    }
    else
    {
      this->dataPtr->cumulativeArcLengths[i] = 0.0;
    }
  }
  this->dataPtr->arcLength = (this->dataPtr->cumulativeArcLengths.back()
                              + this->dataPtr->segments.back().ArcLength());
}

///////////////////////////////////////////////////////////
Vector3d Spline::Point(const unsigned int _index) const
{
  return this->MthDerivative(_index, 0);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Tangent(const unsigned int _index) const
{
  return this->MthDerivative(_index, 1);
}

///////////////////////////////////////////////////////////
Vector3d Spline::MthDerivative(const unsigned int _index,
                               const unsigned int _mth) const
{
  if (_index >= this->dataPtr->points.size())
    return Vector3d(INF_D, INF_D, INF_D);
  return this->dataPtr->points[_index].MthDerivative(_mth);
}

///////////////////////////////////////////////////////////
size_t Spline::PointCount() const
{
  return this->dataPtr->points.size();
}

///////////////////////////////////////////////////////////
void Spline::Clear()
{
  this->dataPtr->points.clear();
  this->dataPtr->segments.clear();
  this->dataPtr->fixings.clear();
}

///////////////////////////////////////////////////////////
bool Spline::UpdatePoint(const unsigned int _index,
                         const Vector3d &_point)
{
  return this->UpdatePoint(_index, ControlPoint({_point}), false);
}

///////////////////////////////////////////////////////////
bool Spline::UpdatePoint(const unsigned int _index,
                         const Vector3d &_point,
                         const Vector3d &_tangent)
{
  return this->UpdatePoint(_index, ControlPoint({_point, _tangent}), true);
}

///////////////////////////////////////////////////////////
bool Spline::UpdatePoint(const unsigned int _index,
                         const ControlPoint &_point,
                         const bool _fixed)
{
  if (_index >= this->dataPtr->points.size())
    return false;

  this->dataPtr->points[_index].Match(_point);
  this->dataPtr->fixings[_index] = _fixed;

  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
  else
    this->Rebuild();
  return true;
}

///////////////////////////////////////////////////////////
void Spline::AutoCalculate(bool _autoCalc)
{
  this->dataPtr->autoCalc = _autoCalc;
}
