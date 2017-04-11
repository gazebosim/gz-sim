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

#include "ignition/math/Matrix4.hh"

#include "ignition/math/SplinePrivate.hh"

namespace ignition
{
namespace math
{

///////////////////////////////////////////////////////////
Vector4d GetPolynomialPowers(const unsigned int _order,
                             const double _t)
{
  // It is much faster to go over this table than
  // delving into factorials and power computations.
  double t2 = _t * _t;
  double t3 = t2 * _t;
  switch (_order) {
    case 0:
      return Vector4d(t3, t2, _t, 1.0);
    case 1:
      return Vector4d(3*t2, 2*_t, 1.0, 0.0);
    case 2:
      return Vector4d(6*_t, 2.0, 0.0, 0.0f);
    case 3:
      return Vector4d(6.0, 0.0, 0.0, 0.0);
    default:
      return Vector4d(0.0, 0.0, 0.0, 0.0);
  }
}

///////////////////////////////////////////////////////////
void ComputeCubicBernsteinHermiteCoeff(const ControlPoint &_startPoint,
                                       const ControlPoint &_endPoint,
                                       Matrix4d &_coeffs)
{
  // Get values and tangents
  const Vector3d &point0 = _startPoint.MthDerivative(0);
  const Vector3d &point1 = _endPoint.MthDerivative(0);
  const Vector3d &tan0 = _startPoint.MthDerivative(1);
  const Vector3d &tan1 = _endPoint.MthDerivative(1);

  // Bernstein polynomials matrix
  const Matrix4d bmatrix(2.0, -2.0,  1.0,  1.0,
                        -3.0,  3.0, -2.0, -1.0,
                         0.0,  0.0,  1.0,  0.0,
                         1.0,  0.0,  0.0,  0.0);

  // Control vectors matrix
  Matrix4d cmatrix(point0.X(), point0.Y(), point0.Z(), 1.0,
                   point1.X(), point1.Y(), point1.Z(), 1.0,
                   tan0.X(),   tan0.Y(),   tan0.Z(),   1.0,
                   tan1.X(),   tan1.Y(),   tan1.Z(),   1.0);

  // Compute coefficients
  _coeffs = bmatrix * cmatrix;
}

///////////////////////////////////////////////////////////
IntervalCubicSpline::IntervalCubicSpline()
    : startPoint({Vector3d::Zero, Vector3d::Zero}),
      endPoint({Vector3d::Zero, Vector3d::Zero}),
      coeffs(Matrix4d::Zero),
      arcLength(0.0f)
{
}

///////////////////////////////////////////////////////////
IntervalCubicSpline::IntervalCubicSpline(const ControlPoint &_startPoint,
                                         const ControlPoint &_endPoint)
    : startPoint(_startPoint), endPoint(_endPoint)
{
  ComputeCubicBernsteinHermiteCoeff(
      this->startPoint, this->endPoint, this->coeffs);

  this->arcLength = this->ComputeArcLength();
}

void IntervalCubicSpline::SetPoints(const ControlPoint &_startPoint,
                                    const ControlPoint &_endPoint)
{
  this->startPoint = _startPoint; this->endPoint = _endPoint;
  ComputeCubicBernsteinHermiteCoeff(
      this->startPoint, this->endPoint, this->coeffs);

  this->arcLength = this->ComputeArcLength();
}

double IntervalCubicSpline::ComputeArcLength() const
{
  // 5 Point Gauss-Legendre quadrature rule for numerical path integration
  // TODO: generalize into a numerical integration toolkit ?
  double arc_length = 0.2844444444444444 * this->InterpolateMthDerivative(
      1, 0.5000000000000000).Length();
  arc_length += 0.2393143352496833 * (
      this->InterpolateMthDerivative(1, 0.2307653449471584).Length()
      + this->InterpolateMthDerivative(1, 0.7692346550528415).Length());
  arc_length += 0.1184634425280946 * (
      this->InterpolateMthDerivative(1, 0.0469100770306680).Length()
      + this->InterpolateMthDerivative(1, 0.9530899229693319).Length());
  return arc_length;
}

///////////////////////////////////////////////////////////
Vector3d IntervalCubicSpline::InterpolateMthDerivative(const unsigned int _mth,
                                                       const double _t) const
{
  if (_t < 0.0 || _t > 1.0)
    return Vector3d(INF_D, INF_D, INF_D);

  if (equal(_t, 0.0))
    return this->startPoint.MthDerivative(_mth);
  else if (equal(_t, 1.0))
    return this->endPoint.MthDerivative(_mth);

  Vector4d powers = GetPolynomialPowers(_mth, _t);
  Vector4d interpolation = powers * this->coeffs;
  return Vector3d(interpolation.X(), interpolation.Y(), interpolation.Z());
}

///////////////////////////////////////////////////////////
InverseArcLengthInterpolator::InverseArcLengthInterpolator()
    : arcLength(0)
{
}

///////////////////////////////////////////////////////////
InverseArcLengthInterpolator::InverseArcLengthInterpolator(
    const CurveInterpolator &_curve)
{
  this->Rescale(_curve);
}

///////////////////////////////////////////////////////////
void InverseArcLengthInterpolator::Rescale(
    const CurveInterpolator &_curve)
{
  this->arcLength = _curve.ArcLength();
}

///////////////////////////////////////////////////////////
double InverseArcLengthInterpolator::InterpolateMthDerivative(
    const unsigned int _mth, const double _s) const
{
  // TODO: implement inverse arc length
  // function approximation for proper
  // parameterization
  if (_mth > 1)
    return 0.0f;
  if (_mth > 0)
    return 1.0f / this->arcLength;
  return _s / this->arcLength;
}

}
}
