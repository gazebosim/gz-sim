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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector4.hh"
#include "ignition/math/Spline.hh"

using namespace ignition;
using namespace math;

///////////////////////////////////////////////////////////
Spline::Spline()
{
  // Set up matrix
  // Hermite polynomial
  this->coeffs(0, 0) = 2;
  this->coeffs(0, 1) = -2;
  this->coeffs(0, 2) = 1;
  this->coeffs(0, 3) = 1;

  this->coeffs(1, 0) = -3;
  this->coeffs(1, 1) = 3;
  this->coeffs(1, 2) = -2;
  this->coeffs(1, 3) = -1;

  this->coeffs(2, 0) = 0;
  this->coeffs(2, 1) = 0;
  this->coeffs(2, 2) = 1;
  this->coeffs(2, 3) = 0;

  this->coeffs(3, 0) = 1;
  this->coeffs(3, 1) = 0;
  this->coeffs(3, 2) = 0;
  this->coeffs(3, 3) = 0;

  this->autoCalc = true;
  this->tension = 0.0;
}

///////////////////////////////////////////////////////////
Spline::~Spline()
{
}

///////////////////////////////////////////////////////////
void Spline::Tension(double _t)
{
  this->tension = _t;
  this->RecalcTangents();
}

///////////////////////////////////////////////////////////
double Spline::Tension() const
{
  return this->tension;
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p)
{
  this->points.push_back(_p);
  if (this->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(double _t) const
{
  // Currently assumes points are evenly spaced, will cause velocity
  // change where this is not the case
  // TODO: base on arclength?

  // Work out which segment this is in
  double fSeg = _t * (this->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;
  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(unsigned int _fromIndex, double _t) const
{
  // Bounds check
  if (_fromIndex >= this->points.size())
    throw IndexException();

  if ((_fromIndex + 1) == this->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->points[_fromIndex];
  }

  // Fast special cases
  if (equal(_t, 0.0))
    return this->points[_fromIndex];
  else if (equal(_t, 1.0))
    return this->points[_fromIndex + 1];

  // double interpolation
  // Form a vector of powers of t
  double t2, t3;
  t2 = _t * _t;
  t3 = t2 * _t;
  Vector4d powers(t3, t2, _t, 1);


  // Algorithm is ret = powers * this->coeffs * Matrix4(point1,
  // point2, tangent1, tangent2)
  const Vector3d &point1 = this->points[_fromIndex];
  const Vector3d &point2 = this->points[_fromIndex+1];
  const Vector3d &tan1 = this->tangents[_fromIndex];
  const Vector3d &tan2 = this->tangents[_fromIndex+1];
  Matrix4d pt;

  pt(0, 0) = point1.x();
  pt(0, 1) = point1.y();
  pt(0, 2) = point1.z();
  pt(0, 3) = 1.0f;
  pt(1, 0) = point2.x();
  pt(1, 1) = point2.y();
  pt(1, 2) = point2.z();
  pt(1, 3) = 1.0f;
  pt(2, 0) = tan1.x();
  pt(2, 1) = tan1.y();
  pt(2, 2) = tan1.z();
  pt(2, 3) = 1.0f;
  pt(3, 0) = tan2.x();
  pt(3, 1) = tan2.y();
  pt(3, 2) = tan2.z();
  pt(3, 3) = 1.0f;

  Vector4d ret = powers * this->coeffs * pt;

  return Vector3d(ret.x(), ret.y(), ret.z());
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

  numPoints = this->points.size();
  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  // Closed or open?
  if (this->points[0] == this->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  double t = 1.0 - this->tension;
  this->tangents.resize(numPoints);

  for (i = 0; i < numPoints; ++i)
  {
    if (i == 0)
    {
      // Special case start
      if (isClosed)
      {
        // Use nuthis->points-2 since nuthis->points-1 is the last
        // point and == [0]
        this->tangents[i] =
          ((this->points[1] - this->points[numPoints-2]) * 0.5) * t;
      }
      else
      {
        this->tangents[i] =
          ((this->points[1] - this->points[0]) * 0.5) * t;
      }
    }
    else if (i == numPoints-1)
    {
      // Special case end
      if (isClosed)
      {
        // Use same tangent as already calculated for [0]
        this->tangents[i] = this->tangents[0];
      }
      else
      {
        this->tangents[i] =
          ((this->points[i] - this->points[i-1]) * 0.5) * t;
      }
    }
    else
    {
      this->tangents[i] =
        ((this->points[i+1] - this->points[i-1]) * 0.5) * t;
    }
  }
}

///////////////////////////////////////////////////////////
Vector3d Spline::Point(unsigned int _index) const
{
  if (_index >= this->points.size())
    throw IndexException();

  return this->points[_index];
}

///////////////////////////////////////////////////////////
Vector3d Spline::Tangent(unsigned int _index) const
{
  if (_index >= this->tangents.size())
    throw IndexException();

  return this->tangents[_index];
}

///////////////////////////////////////////////////////////
unsigned int Spline::PointCount() const
{
  return this->points.size();
}

///////////////////////////////////////////////////////////
void Spline::Clear()
{
  this->points.clear();
  this->tangents.clear();
}

///////////////////////////////////////////////////////////
void Spline::UpdatePoint(unsigned int _index, const Vector3d &_value)
{
  if (_index >= this->points.size())
    throw IndexException();

  this->points[_index] = _value;
  if (this->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
void Spline::AutoCalculate(bool _autoCalc)
{
  this->autoCalc = _autoCalc;
}
