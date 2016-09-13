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

#include "ignition/math/SplinePrivate.hh"
#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector4.hh"
#include "ignition/math/Spline.hh"

using namespace ignition;
using namespace math;

///////////////////////////////////////////////////////////
Spline::Spline()
: dataPtr(new SplinePrivate)
{
  // Set up matrix
  // Hermite polynomial
  this->dataPtr->coeffs(0, 0) = 2;
  this->dataPtr->coeffs(0, 1) = -2;
  this->dataPtr->coeffs(0, 2) = 1;
  this->dataPtr->coeffs(0, 3) = 1;

  this->dataPtr->coeffs(1, 0) = -3;
  this->dataPtr->coeffs(1, 1) = 3;
  this->dataPtr->coeffs(1, 2) = -2;
  this->dataPtr->coeffs(1, 3) = -1;

  this->dataPtr->coeffs(2, 0) = 0;
  this->dataPtr->coeffs(2, 1) = 0;
  this->dataPtr->coeffs(2, 2) = 1;
  this->dataPtr->coeffs(2, 3) = 0;

  this->dataPtr->coeffs(3, 0) = 1;
  this->dataPtr->coeffs(3, 1) = 0;
  this->dataPtr->coeffs(3, 2) = 0;
  this->dataPtr->coeffs(3, 3) = 0;

  this->dataPtr->autoCalc = true;
  this->dataPtr->tension = 0.0;
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
  this->RecalcTangents();
}

///////////////////////////////////////////////////////////
double Spline::Tension() const
{
  return this->dataPtr->tension;
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p)
{
  this->dataPtr->points.push_back(_p);
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(double _t) const
{
  // Currently assumes points are evenly spaced, will cause velocity
  // change where this is not the case
  // TODO: base on arclength?

  // Work out which segment this is in
  double fSeg = _t * (this->dataPtr->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;
  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(unsigned int _fromIndex, double _t) const
{
  // Bounds check
  if (_fromIndex >= this->dataPtr->points.size())
    throw IndexException();

  if ((_fromIndex + 1) == this->dataPtr->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->dataPtr->points[_fromIndex];
  }

  // Fast special cases
  if (equal(_t, 0.0))
    return this->dataPtr->points[_fromIndex];
  else if (equal(_t, 1.0))
    return this->dataPtr->points[_fromIndex + 1];

  // double interpolation
  // Form a vector of powers of t
  double t2, t3;
  t2 = _t * _t;
  t3 = t2 * _t;
  Vector4d powers(t3, t2, _t, 1);


  // Algorithm is ret = powers * this->dataPtr->coeffs * Matrix4(point1,
  // point2, tangent1, tangent2)
  const Vector3d &point1 = this->dataPtr->points[_fromIndex];
  const Vector3d &point2 = this->dataPtr->points[_fromIndex+1];
  const Vector3d &tan1 = this->dataPtr->tangents[_fromIndex];
  const Vector3d &tan2 = this->dataPtr->tangents[_fromIndex+1];
  Matrix4d pt;

  pt(0, 0) = point1.X();
  pt(0, 1) = point1.Y();
  pt(0, 2) = point1.Z();
  pt(0, 3) = 1.0f;
  pt(1, 0) = point2.X();
  pt(1, 1) = point2.Y();
  pt(1, 2) = point2.Z();
  pt(1, 3) = 1.0f;
  pt(2, 0) = tan1.X();
  pt(2, 1) = tan1.Y();
  pt(2, 2) = tan1.Z();
  pt(2, 3) = 1.0f;
  pt(3, 0) = tan2.X();
  pt(3, 1) = tan2.Y();
  pt(3, 2) = tan2.Z();
  pt(3, 3) = 1.0f;

  Vector4d ret = powers * this->dataPtr->coeffs * pt;

  return Vector3d(ret.X(), ret.Y(), ret.Z());
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
  if (this->dataPtr->points[0] == this->dataPtr->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  double t = 1.0 - this->dataPtr->tension;
  this->dataPtr->tangents.resize(numPoints);

  for (i = 0; i < numPoints; ++i)
  {
    if (i == 0)
    {
      // Special case start
      if (isClosed)
      {
        // Use this->dataPtr->points-2 since this->dataPtr->points-1
        // is the last point and == [0]
        this->dataPtr->tangents[i] =
          ((this->dataPtr->points[1] -
            this->dataPtr->points[numPoints-2]) * 0.5) * t;
      }
      else
      {
        this->dataPtr->tangents[i] =
          ((this->dataPtr->points[1] - this->dataPtr->points[0]) * 0.5) * t;
      }
    }
    else if (i == numPoints-1)
    {
      // Special case end
      if (isClosed)
      {
        // Use same tangent as already calculated for [0]
        this->dataPtr->tangents[i] = this->dataPtr->tangents[0];
      }
      else
      {
        this->dataPtr->tangents[i] =
          ((this->dataPtr->points[i] - this->dataPtr->points[i-1]) * 0.5) * t;
      }
    }
    else
    {
      this->dataPtr->tangents[i] =
        ((this->dataPtr->points[i+1] - this->dataPtr->points[i-1]) * 0.5) * t;
    }
  }
}

///////////////////////////////////////////////////////////
Vector3d Spline::Point(unsigned int _index) const
{
  if (_index >= this->dataPtr->points.size())
    throw IndexException();

  return this->dataPtr->points[_index];
}

///////////////////////////////////////////////////////////
Vector3d Spline::Tangent(unsigned int _index) const
{
  if (_index >= this->dataPtr->tangents.size())
    throw IndexException();

  return this->dataPtr->tangents[_index];
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
  this->dataPtr->tangents.clear();
}

///////////////////////////////////////////////////////////
void Spline::UpdatePoint(unsigned int _index, const Vector3d &_value)
{
  if (_index >= this->dataPtr->points.size())
    throw IndexException();

  this->dataPtr->points[_index] = _value;
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
void Spline::AutoCalculate(bool _autoCalc)
{
  this->dataPtr->autoCalc = _autoCalc;
}
