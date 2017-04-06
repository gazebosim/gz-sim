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
    : dataPtr(new SplinePrivate())
{
  // Set up matrix
  this->dataPtr->autoCalc = true;
  this->dataPtr->tension = 0.0;
  this->dataPtr->arc_length = 0.0;
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
  // TODO: Rebuild should call RecalcTangents, not the
  // other way around, BUT that isn't backwards compatible  
  this->Rebuild();
}

///////////////////////////////////////////////////////////
double Spline::Tension() const
{
  return this->dataPtr->tension;
}

///////////////////////////////////////////////////////////
double Spline::ArcLength() const
{
  return this->dataPtr->arc_length;
}

///////////////////////////////////////////////////////////
double Spline::ArcLength(const unsigned int _index) const
{
  if (_index >= this->dataPtr->segments.size())
    return INF_D;
  return this->dataPtr->segments[_index].ArcLength();
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p)
{
  this->AddPoint({_p, Vector3d(INF_D, INF_D, INF_D)}, false);
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3d &_p, const Vector3d &_t)
{
  this->AddPoint({_p, _t}, true);
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const ControlPoint &_cp, bool _fixed)
{
  this->dataPtr->points.push_back(_cp);
  this->dataPtr->fixings.push_back(_fixed);
  if (this->dataPtr->autoCalc)    
    this->RecalcTangents();
  // TODO: Rebuild should call RecalcTangents, not the
  // other way around, BUT that isn't backwards compatible  
  this->Rebuild();

}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateMthDerivative(const unsigned int _mth,
                                          const double _s) const
{
  auto it = std::lower_bound(this->dataPtr->cumulative_arc_lengths.begin(),
                             this->dataPtr->cumulative_arc_lengths.end(),
                             _s);
  int fromIndex = it - this->dataPtr->cumulative_arc_lengths.begin() - 1;
  double s_fraction = _s - this->dataPtr->cumulative_arc_lengths[fromIndex];
  return this->InterpolateMthDerivative(fromIndex, _mth, s_fraction);
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateMthDerivative(const unsigned int _fromIndex,
                                          const unsigned int _mth,
                                          const double _s) const
{
  // Bounds check
  if (_fromIndex >= this->dataPtr->points.size())
    return Vector3d(INF_D, INF_D, INF_D);
  
  if (_fromIndex == this->dataPtr->points.size() - 1)
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->dataPtr->points[_fromIndex].MthDerivative(_mth);
  }

  if (_mth > 3) {
    // M > 3 => p = 0 (as this is a cubic interpolator)
    return Vector3d(0.0f, 0.0f, 0.0f);
  }
  double t_s = this->dataPtr->params[_fromIndex].InterpolateMthDerivative(0, _s);
  if (_mth < 1) {
    // M = 0 => P(s) = Q(t(s))
    return this->dataPtr->segments[_fromIndex].InterpolateMthDerivative(0, t_s);
  }
  double t_prime_s = this->dataPtr->params[_fromIndex].InterpolateMthDerivative(1, _s);
  Vector3d q_prime_t = this->dataPtr->segments[_fromIndex].InterpolateMthDerivative(1, t_s);
  if (_mth < 2) {
    // M = 1 => P'(s) = Q'(t(s)) * t'(s)
    return q_prime_t * t_prime_s;
  }
  double t_prime_s_2 = t_prime_s * t_prime_s;
  double t_prime2_s = this->dataPtr->params[_fromIndex].InterpolateMthDerivative(2, _s);
  Vector3d q_prime2_t = this->dataPtr->segments[_fromIndex].InterpolateMthDerivative(2, t_s);
  if (_mth < 3) {
    // M = 2 => P''(s) = Q''(t(s)) * t'(s)^2 + Q'(t(s)) * t''(s)
    return q_prime2_t * t_prime_s_2 + q_prime_t * t_prime2_s;
  }
  double t_prime_s_3 = t_prime_s_2 * t_prime_s;
  double t_prime3_s = this->dataPtr->params[_fromIndex].InterpolateMthDerivative(3, _s);
  Vector3d q_prime3_t = this->dataPtr->segments[_fromIndex].InterpolateMthDerivative(3, t_s);
  // M = 3 => P'''(s) = Q'''(t(s)) * t'(s)^3
  ///                   + 3 * Q''(t(s)) * t'(s) * t''(s)
  ///                   + Q'(t(s)) * t'''(s)
  return (q_prime3_t * t_prime_s_3
          + 3 * q_prime2_t * t_prime_s * t_prime2_s
          + q_prime_t * t_prime3_s);
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(const double _t) const
{
  return this->InterpolateMthDerivative(
      0, _t * this->ArcLength());
}

///////////////////////////////////////////////////////////
Vector3d Spline::Interpolate(const unsigned int _fromIndex,
                             const double _t) const
{
  return this->InterpolateMthDerivative(
      _fromIndex, 0, _t * this->ArcLength(_fromIndex));
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateTangent(const double _t) const
{
  return this->InterpolateMthDerivative(
      1, _t * this->ArcLength());
}

///////////////////////////////////////////////////////////
Vector3d Spline::InterpolateTangent(const unsigned int _fromIndex,
                                    const double _t) const
{
  return this->InterpolateMthDerivative(
      _fromIndex, 1, _t * this->ArcLength(_fromIndex));
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
          this->dataPtr->points[i].MthDerivative(1) = this->dataPtr->points[0].MthDerivative(1);
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
  this->dataPtr->params.resize(numSegments);
  this->dataPtr->cumulative_arc_lengths.resize(numSegments);
  for (size_t i = 0 ; i < numSegments ; ++i)
  {
    this->dataPtr->segments[i].SetPoints(this->dataPtr->points[i],
                                         this->dataPtr->points[i+1]);
    this->dataPtr->params[i].Rescale(this->dataPtr->segments[i]);
    
    if (i > 0) {
      this->dataPtr->cumulative_arc_lengths[i] =
          (this->dataPtr->segments[i-1].ArcLength()
           + this->dataPtr->cumulative_arc_lengths[i-1]);
    }
    else {
      this->dataPtr->cumulative_arc_lengths[i] = 0.0;
    }
  }
  this->dataPtr->arc_length = (this->dataPtr->cumulative_arc_lengths.back()
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
  this->dataPtr->params.clear();
  this->dataPtr->fixings.clear();
}

///////////////////////////////////////////////////////////
bool Spline::UpdatePoint(const unsigned int _index,
                         const Vector3d &_point)
{
  return this->UpdatePoint(_index, {_point}, false);
}

///////////////////////////////////////////////////////////
bool Spline::UpdatePoint(const unsigned int _index,
                         const Vector3d &_point,
                         const Vector3d &_tangent)
{
  return this->UpdatePoint(_index, {_point, _tangent}, true);
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
  // TODO: Rebuild should call RecalcTangents, not the
  // other way around, BUT that isn't backwards compatible  
  this->Rebuild();
  return true;
}

///////////////////////////////////////////////////////////
void Spline::AutoCalculate(bool _autoCalc)
{
  this->dataPtr->autoCalc = _autoCalc;
}
