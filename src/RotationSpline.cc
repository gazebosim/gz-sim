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
#include "ignition/math/Quaternion.hh"
#include "ignition/math/RotationSpline.hh"
#include "RotationSplinePrivate.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
RotationSpline::RotationSpline()
: dataPtr(new RotationSplinePrivate)
{
}

/////////////////////////////////////////////////
RotationSpline::~RotationSpline()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void RotationSpline::AddPoint(const Quaterniond &_p)
{
  this->dataPtr->points.push_back(_p);
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();
}

/////////////////////////////////////////////////
Quaterniond RotationSpline::Interpolate(double _t,
                                        const bool _useShortestPath)
{
  // Work out which segment this is in
  double fSeg = _t * (this->dataPtr->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;

  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t, _useShortestPath);
}

/////////////////////////////////////////////////
Quaterniond RotationSpline::Interpolate(const unsigned int _fromIndex,
    const double _t, const bool _useShortestPath)
{
  // Bounds check
  if (_fromIndex >= this->dataPtr->points.size())
    return Quaterniond(INF_D, INF_D, INF_D, INF_D);

  if ((_fromIndex + 1) == this->dataPtr->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->dataPtr->points[_fromIndex];
  }

  // Fast special cases
  if (math::equal(_t, 0.0))
    return this->dataPtr->points[_fromIndex];
  else if (math::equal(_t, 1.0))
    return this->dataPtr->points[_fromIndex + 1];

  // double interpolation
  // Use squad using tangents we've already set up
  Quaterniond &p = this->dataPtr->points[_fromIndex];
  Quaterniond &q = this->dataPtr->points[_fromIndex+1];
  Quaterniond &a = this->dataPtr->tangents[_fromIndex];
  Quaterniond &b = this->dataPtr->tangents[_fromIndex+1];

  // NB interpolate to nearest rotation
  return Quaterniond::Squad(_t, p, a, b, q, _useShortestPath);
}

/////////////////////////////////////////////////
void RotationSpline::RecalcTangents()
{
  // ShoeMake (1987) approach
  // Just like Catmull-Rom really, just more gnarly
  // And no, I don't understand how to derive this!
  //
  // let p = point[i], pInv = p.Inverse
  // tangent[i] = p * exp(-0.25 *
  // (log(pInv * point[i+1]) + log(pInv * point[i-1])))
  //
  // Assume endpoint tangents are parallel with line with neighbour

  unsigned int i;
  bool isClosed;

  size_t numPoints = this->dataPtr->points.size();

  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  this->dataPtr->tangents.resize(numPoints);

  if (this->dataPtr->points[0] == this->dataPtr->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  Quaterniond invp, part1, part2, preExp;
  for (i = 0; i < numPoints; ++i)
  {
    Quaterniond &p = this->dataPtr->points[i];
    invp = p.Inverse();

    if (i == 0)
    {
      // special case start
      part1 = (invp * this->dataPtr->points[i+1]).Log();
      if (isClosed)
      {
        // Use numPoints-2 since numPoints-1 == end == start == this one
        part2 = (invp * this->dataPtr->points[numPoints-2]).Log();
      }
      else
      {
        part2 = (invp * p).Log();
      }
    }
    else if (i == numPoints-1)
    {
      // special case end
      if (isClosed)
      {
        // Wrap to [1] (not [0], this is the same as end == this one)
        part1 = (invp * this->dataPtr->points[1]).Log();
      }
      else
      {
        part1 = (invp * p).Log();
      }
      part2 = (invp * this->dataPtr->points[i-1]).Log();
    }
    else
    {
      part1 = (invp * this->dataPtr->points[i+1]).Log();
      part2 = (invp * this->dataPtr->points[i-1]).Log();
    }

    preExp = (part1 + part2) * -0.25;
    this->dataPtr->tangents[i] = p * preExp.Exp();
  }
}

/////////////////////////////////////////////////
const Quaterniond &RotationSpline::Point(const unsigned int _index) const
{
  static Quaterniond inf(INF_D, INF_D, INF_D, INF_D);

  if (this->dataPtr->points.empty())
    return inf;

  return this->dataPtr->points[
    clamp(_index, 0u, static_cast<unsigned int>(
          this->dataPtr->points.size()-1))];
}

/////////////////////////////////////////////////
unsigned int RotationSpline::PointCount() const
{
  return static_cast<unsigned int>(this->dataPtr->points.size());
}

/////////////////////////////////////////////////
void RotationSpline::Clear()
{
  this->dataPtr->points.clear();
  this->dataPtr->tangents.clear();
}

/////////////////////////////////////////////////
bool RotationSpline::UpdatePoint(const unsigned int _index,
                                 const Quaterniond &_value)
{
  if (_index >= this->dataPtr->points.size())
    return false;

  this->dataPtr->points[_index] = _value;
  if (this->dataPtr->autoCalc)
    this->RecalcTangents();

  return true;
}

/////////////////////////////////////////////////
void RotationSpline::AutoCalculate(bool _autoCalc)
{
  this->dataPtr->autoCalc = _autoCalc;
}
