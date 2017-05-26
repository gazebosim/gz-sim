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

#include <algorithm>

#include "ignition/math/SplinePrivate.hh"
#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector4.hh"
#include "ignition/math/Spline.hh"

using namespace ignition;
using namespace math;

///////////////////////////////////////////////////////////
void ComputeLoopCriticalPoints(const ControlPoint &_startPoint,
                               const ControlPoint &_endPoint,
                               ControlPoint &_criticalStartPoint,
                               ControlPoint &_criticalEndPoint)
{
  // Bezier polynomial basis.
  const Matrix4d bmatrix(-1.0, 3.0, -3.0, 1.0,
                         3.0, -6.0, 3.0, 0.0,
                         -3.0, 3.0, 0.0, 0.0,
                         1.0, 0.0, 0.0, 0.0);
  // Hermite basis matrix
  const Matrix4d hmatrix(2.0, -2.0,  1.0,  1.0,
                        -3.0,  3.0, -2.0, -1.0,
                         0.0,  0.0,  1.0,  0.0,
                         1.0,  0.0,  0.0,  0.0);
  
  // Get Hermite control points
  const Vector3d &hpoint0 = _startPoint.MthDerivative(0);
  const Vector3d &hpoint1 = _endPoint.MthDerivative(0);
  const Vector3d &htan0 = _startPoint.MthDerivative(1);
  const Vector3d &htan1 = _endPoint.MthDerivative(1);

  // Build Hermite control matrix
  Matrix4d hcmatrix(hpoint0.X(), hpoint0.Y(), hpoint0.Z(), 1.0,
                    hpoint1.X(), hpoint1.Y(), hpoint1.Z(), 1.0,
                    htan0.X(),   htan0.Y(),   htan0.Z(),   1.0,
                    htan1.X(),   htan1.Y(),   htan1.Z(),   1.0);

  // Compute Bezier control matrix
  Matrix4d bcmatrix = bmatrix.Inverse() * hmatrix * hcmatrix;

  // Get Bezier control points
  Vector3d bpoint0(bcmatrix(0, 0),
                   bcmatrix(0, 1),
                   bcmatrix(0, 2));
  Vector3d bpoint1(bcmatrix(1, 0),
                   bcmatrix(1, 1),
                   bcmatrix(1, 2));
  Vector3d bpoint2(bcmatrix(2, 0),
                   bcmatrix(2, 1),
                   bcmatrix(2, 2));
  Vector3d bpoint3(bcmatrix(3, 0),
                   bcmatrix(3, 1),
                   bcmatrix(3, 2));

  // Compute distance vectors and useful cross-products
  const Vector3d a = bpoint3 - bpoint0;
  const Vector3d b = bpoint1 - bpoint0;
  const Vector3d c = bpoint2 - bpoint3;

  const Vector3d axc = a.Cross(c);
  const Vector3d bxc = b.Cross(c);
  const Vector3d axb = a.Cross(b);

  // Following uses a large scalar as a replacement of infinity
  // in order for the matrix products to yield meaningful values.
  
  if (bxc != Vector3d::Zero) // Non collinear nor parallel tangents
  {
    if (equal(a.Dot(bxc), 0.0)) // Coplanar points
    {
      if (axc != Vector3d::Zero) 
      {
        // The second control point tangent is not collinear
        // with the line that passes through both control points,
        // so intersection with the first control point tangent
        // projection is not at the latter origin.
        
        // If scale factor is less than 1, the first control
        // point tangent extends beyond the intersection, and
        // thus loops are likely to happen (this IS NOT a necessary
        // condition, but a sufficient one).
        bpoint1 = b * (axc.Dot(bxc) / bxc.SquaredLength()) + bpoint0;
      }
      else
      {
        bpoint1 = b * MAX_D * 0.5 + bpoint0;
      }
      
      if (axb != Vector3d::Zero)
      {
        // The first control point tangent is not collinear
        // with the line that passes through both control points,
        // so intersection with the second control point tangent
        // projection is not at the latter origin.
        
        // If scale factor is less than 1, the second control
        // point tangent extends beyond the intersection, and
        // thus loops are likely to happen (this IS NOT a necessary
        // condition, but a sufficient one).
        bpoint2 = c * (axb.Dot(bxc) / bxc.SquaredLength()) + bpoint3;      
      }
      else
      {
        bpoint2 = c * MAX_D * 0.5 + bpoint3;      
      }       
    }
    else
    {
      // TODO: handle non coplanar cases.
      bpoint1 = bpoint0;
      bpoint2 = bpoint3;
    }
  }
  else
  {
    if (axb == Vector3d::Zero)
    {
      // All collinear points case. If inner
      // control points go past each other
      // loops will ensue.
      double k = a.Length() / (c.Length() + b.Length());
      bpoint1 = bpoint0 + k * b; bpoint2 = bpoint3 + k * c;
    } else {
      // Parallel tangents case.
      bpoint1 = b * MAX_D * 0.5 + bpoint0;
      bpoint2 = c * MAX_D * 0.5 + bpoint3;
    }
  }

  // Build Bezier critical control matrix
  bcmatrix.Set(bpoint0.X(), bpoint0.Y(), bpoint0.Z(), 1.0,
               bpoint1.X(), bpoint1.Y(), bpoint1.Z(), 1.0,
               bpoint2.X(), bpoint2.Y(), bpoint2.Z(), 1.0,
               bpoint3.X(), bpoint3.Y(), bpoint3.Z(), 1.0);

  // Compute Hermite critical control matrix
  hcmatrix = hmatrix.Inverse() * bmatrix * bcmatrix;

  // Recover critical hermite control points
  Vector3d &chpoint0 = _criticalStartPoint.MthDerivative(0);
  Vector3d &chpoint1 = _criticalEndPoint.MthDerivative(0);
  Vector3d &chtan0 = _criticalStartPoint.MthDerivative(1);
  Vector3d &chtan1 = _criticalEndPoint.MthDerivative(1);

  chpoint0.Set(hcmatrix(0, 0),
               hcmatrix(0, 1),
               hcmatrix(0, 2));
  chpoint1.Set(hcmatrix(1, 0),
               hcmatrix(1, 1),
               hcmatrix(1, 2));
  chtan0.Set(hcmatrix(2, 0),
             hcmatrix(2, 1),
             hcmatrix(2, 2));
  chtan1.Set(hcmatrix(3, 0),
             hcmatrix(3, 1),
             hcmatrix(3, 2));      
}

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
bool Spline::HasLoop() const
{
  // Check if any segments shows loop
  return std::any_of(
      this->dataPtr->segments.begin(),
      this->dataPtr->segments.end(),
      [](const IntervalCubicSpline &segment) {
        ControlPoint criticalStartPoint, criticalEndPoint;

        ComputeLoopCriticalPoints(
            segment.StartPoint(), segment.EndPoint(),
            criticalStartPoint, criticalEndPoint);

        const Vector3d &startTangent = segment.StartPoint().MthDerivative(1);
        const Vector3d &endTangent = segment.EndPoint().MthDerivative(1);
        const Vector3d &critStartTangent = criticalStartPoint.MthDerivative(1);
        const Vector3d &critEndTangent = criticalEndPoint.MthDerivative(1);
        return (startTangent.Length() > critStartTangent.Length() ||
                endTangent.Length() > critEndTangent.Length());
      });
}

///////////////////////////////////////////////////////////
void Spline::EnsureNoLoop()
{
  size_t numPoints = this->dataPtr->points.size();

  if (numPoints < 2)
  {
    // Nothing to be done here.
    return;
  }

  if (std::any_of(this->dataPtr->fixings.begin(),
                  this->dataPtr->fixings.end(),
                  [](const bool &x) { return x; }))
  {
    // Reset all fixings and recalc tangents.
    std::fill(this->dataPtr->fixings.begin(),
              this->dataPtr->fixings.end(),
              false);

    this->RecalcTangents();
  }

  ControlPoint criticalStartPoint, criticalEndPoint;
  for(size_t i = 0 ; i < this->dataPtr->points.size() - 1 ; ++i)
  {
    ControlPoint &startPoint = this->dataPtr->points[i];
    ControlPoint &endPoint = this->dataPtr->points[i+1];

    ComputeLoopCriticalPoints(startPoint, endPoint,
                              criticalStartPoint,
                              criticalEndPoint);

    Vector3d &startTangent = startPoint.MthDerivative(1);
    Vector3d &endTangent = endPoint.MthDerivative(1);
    const Vector3d &criticalStartTangent = criticalStartPoint.MthDerivative(1);
    const Vector3d &criticalEndTangent = criticalEndPoint.MthDerivative(1);

    if (startTangent.Length() > criticalStartTangent.Length())
    {
      startTangent = criticalStartTangent;
    }

    if (endTangent.Length() > criticalEndTangent.Length())
    {
      endTangent = criticalEndTangent;
    }
  }

  bool isClosed;
  if (this->dataPtr->points[0].MthDerivative(0) ==
      this->dataPtr->points[numPoints-1].MthDerivative(0))
    isClosed = true;
  else
    isClosed = false;

  double noLoopTension = 0.0;
  Vector3d pointNeighboursChord;
  for(size_t i = 0 ; i < numPoints; ++i)
  {
    if (i == 0)
    {
      // Special case start
      if (isClosed)
      {
        // Use this->dataPtr->points-2 since this->dataPtr->points-1
        // is the last point and == [0]
        pointNeighboursChord = (
            this->dataPtr->points[1].MthDerivative(0) -
            this->dataPtr->points[numPoints-2].MthDerivative(0));
      }
      else
      {
        pointNeighboursChord = (this->dataPtr->points[1].MthDerivative(0) -
                                this->dataPtr->points[0].MthDerivative(0));
      }
    }
    else if (i == numPoints-1)
    {
      // Special case end
      if (isClosed)
      {
        // Use same tangent as already calculated for [0]
        pointNeighboursChord = (
            this->dataPtr->points[1].MthDerivative(0) -
            this->dataPtr->points[numPoints-2].MthDerivative(0));
      }
      else
      {
        pointNeighboursChord = (this->dataPtr->points[i].MthDerivative(0) -
                                this->dataPtr->points[i-1].MthDerivative(0));
      }
    }
    else
    {
      pointNeighboursChord = (this->dataPtr->points[i+1].MthDerivative(0) -
                              this->dataPtr->points[i-1].MthDerivative(0));
    }

    const Vector3d &pointTangent = this->dataPtr->points[i].MthDerivative(1);

    noLoopTension = std::max(
        noLoopTension, (1 - 2 * pointTangent.Length()
                        / pointNeighboursChord.Length()));
  }
  this->dataPtr->tension = noLoopTension;

  this->RecalcTangents();
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
