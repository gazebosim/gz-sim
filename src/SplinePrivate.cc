#include "ignition/math/Matrix4.hh"

#include "ignition/math/SplinePrivate.hh"

namespace ignition
{
namespace math
{

///////////////////////////////////////////////////////////
Vector4d GetPolynomialPowers(const unsigned int _order,
                             const double _t) {  
  // It is much faster to go over this table than
  // delving into factorials and power computations.
  double t2 = _t * _t;
  double t3 = t2 * _t;
  switch(_order) {
    case 0:
      return Vector4d(t3, t2, _t, 1.0f);
    case 1:
      return Vector4d(3*t2, 2*_t, 1.0f, 0.0f);
    case 2:
      return Vector4d(6*_t, 2, 0.0f, 0.0f);
    case 3:
      return Vector4d(6.0f, 0.0f, 0.0f, 0.0f);
    default:
      return Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  };
}

///////////////////////////////////////////////////////////
void ComputeCubicBernsteinHermiteCoeff(const ControlPoint &_start,
                                       const ControlPoint &_end,
                                       Matrix4d &_coeffs)
{
  // Get values and tangents
  const Vector3d &point0 = _start.MthDerivative(0);
  const Vector3d &point1 = _end.MthDerivative(0);
  const Vector3d &tan0 = _start.MthDerivative(1);
  const Vector3d &tan1 = _end.MthDerivative(1);

  // Bernstein polynomials matrix
  const Matrix4d bmatrix( 2.0f, -2.0f,  1.0f,  1.0f,
                         -3.0f,  3.0f, -2.0f, -1.0f,
                          0.0f,  0.0f,  1.0f,  0.0f,
                          1.0f,  0.0f,  0.0f,  0.0f);
  
  // Control vectors matrix
  Matrix4d cmatrix (point0.X(), point0.Y(), point0.Z(), 1.0f,
                    point1.X(), point1.Y(), point1.Z(), 1.0f,
                    tan0.X(),   tan0.Y(),   tan0.Z(), 1.0f,
                    tan1.X(),   tan1.Y(),   tan1.Z(), 1.0f);

  // Compute coefficients
  _coeffs = bmatrix * cmatrix;
}

///////////////////////////////////////////////////////////
IntervalCubicSpline::IntervalCubicSpline()
    : start_({Vector3d::Zero, Vector3d::Zero}),
      end_({Vector3d::Zero, Vector3d::Zero}),
      coeffs_(Matrix4d::Zero),
      arc_length_(0.0f)
{
}

///////////////////////////////////////////////////////////
IntervalCubicSpline::IntervalCubicSpline(const ControlPoint &_start,
                                         const ControlPoint &_end)
    : start_(_start), end_(_end)
{
  ComputeCubicBernsteinHermiteCoeff(this->start_, this->end_, this->coeffs_);
  
  this->arc_length_ = this->ComputeArcLength();
}

void IntervalCubicSpline::SetPoints(const ControlPoint &_start,
                                    const ControlPoint &_end)
{
  this->start_ = _start; this->end_ = _end;
  ComputeCubicBernsteinHermiteCoeff(this->start_, this->end_, this->coeffs_);

  this->arc_length_ = this->ComputeArcLength();
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
  if ( _t < 0.0f || _t > 1.0f )
    return Vector3d(INF_D, INF_D, INF_D);

  if (equal(_t, 0.0))
    return this->start_.MthDerivative(_mth);
  else if (equal(_t, 1.0))
    return this->end_.MthDerivative(_mth);

  Vector4d powers = GetPolynomialPowers(_mth, _t);
  Vector4d interpolation = powers * this->coeffs_;
  return Vector3d(interpolation.X(), interpolation.Y(), interpolation.Z());
}

///////////////////////////////////////////////////////////
InverseArcLengthCubicInterpolator::InverseArcLengthCubicInterpolator()
    : arc_length_(0)
{
}

///////////////////////////////////////////////////////////
InverseArcLengthCubicInterpolator::InverseArcLengthCubicInterpolator(const IntervalCubicSpline &_spline)    
{
  this->Rescale(_spline);
}

///////////////////////////////////////////////////////////
void InverseArcLengthCubicInterpolator::Rescale(const IntervalCubicSpline &_spline)
{
  this->arc_length_ = _spline.ArcLength();
}

///////////////////////////////////////////////////////////
double InverseArcLengthCubicInterpolator::InterpolateMthDerivative(const unsigned int _mth,
                                                                   const double _s) const
{
  // TODO: implement inverse arc length
  // function approximation for proper
  // parameterization
  if (_mth > 1)
    return 0.0f;
  if (_mth > 0)
    return 1.0f / this->arc_length_;
  return _s / this->arc_length_;
}

}
}
