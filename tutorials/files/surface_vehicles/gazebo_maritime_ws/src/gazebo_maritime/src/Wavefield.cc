/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#include <gz/msgs/any.pb.h>
#include <gz/msgs/param.pb.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "Wavefield.hh"

using namespace gz;
using namespace maritime;

///////////////////////////////////////////////////////////////////////////////
// Utilities
std::ostream& operator<<(std::ostream &_os, const std::vector<double> &_vec)
{
  for (auto&& v : _vec ) // NOLINT
    _os << v << ", ";
  return _os;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Private data for the WavefieldParameters.
class Wavefield::Implementation
{
  /// \brief Constructor.
  public: Implementation():
    size({6000, 6000}),
    cellCount({300, 300}),
    model("PMS"),
    number(3),
    scale(1.1),
    angle(0.4),
    steepness(0.0),
    amplitude(0.0),
    period(5.0),
    phase(0.0),
    direction(0.0),
    angularFrequency(2.0*M_PI),
    wavelength(2 * M_PI / this->DeepWaterDispersionToWavenumber(2.0 * M_PI)),
    wavenumber(this->DeepWaterDispersionToWavenumber(2.0 * M_PI)),
    tau(2.0),
    gain(1.0)
  {
  }

  /// \brief Populate a msg::Param with the current wavefield parameters.
  public: void FillParameters();

  /// \brief Recalculate for constant wavelength-amplitude ratio.
  public: void RecalculateCwr();

  /// \brief Recalculate for Pierson-Moskowitz spectrum sampling model.
  public: void RecalculatePms();

  /// \brief Recalculate all derived quantities from inputs.
  public: void Recalculate();

  /// \brief Helper function.
  public: double DeepWaterDispersionToOmega(double _wavenumber) const;

  /// \brief Helper function.
  public: double DeepWaterDispersionToWavenumber(double _omega) const;

  /// \brief Pierson-Moskowitz wave spectrum.
  public: double Pm(double _omega, double _omegaP) const;

  /// \brief The size of the wavefield.
  public: math::Vector2d size;

  /// \brief The number of grid cells in the wavefield.
  public: math::Vector2d cellCount;

  /// \brief Name of wavefield model to use - must be "PMS" or "CWR"
  public: std::string model;

  /// \brief The number of component waves.
  public: size_t number;

  /// \brief Set the scale of the largest and smallest waves.
  public: double scale;

  /// \brief Set the angle between component waves and the mean direction.
  public: double angle;

  /// \brief Control the wave steepness. 0 is sine waves, 1 is Gerstner waves.
  public: double steepness;

  /// \brief The mean wave amplitude [m].
  public: double amplitude;

  /// \brief The mean wave period [s]
  public: double period;

  /// \brief The mean wave phase (not currently enabled).
  public: double phase;

  /// \brief The mean wave direction.
  public: double direction;

  /// \brief The time constant for exponential increasing waves on startup
  public: double tau;

  /// \brief The multiplier applied to PM spectra
  public: double gain;

  /// \brief The mean wave angular frequency (derived).
  public: double angularFrequency;

  /// \brief The mean wavelength (derived).
  public: double wavelength;

  /// \brief The mean wavenumber (derived).
  public: double wavenumber;

  /// \brief The component wave angular frequencies (derived).
  public: std::vector<double> angularFrequencies;

  /// \brief The component wave amplitudes (derived).
  public: std::vector<double> amplitudes;

  /// \brief The component wave phases (derived).
  public: std::vector<double> phases;

  /// \brief The component wave steepness factors (derived).
  public: std::vector<double> steepnesses;

  /// \brief The component wavenumbers (derived).
  public: std::vector<double> wavenumbers;

  /// \brief The component wave dirctions (derived).
  public: std::vector<math::Vector2d> directions;

  /// \brief True when waves are present.
  public: bool active = false;

  /// \brief Topic to publish/receive wavefield updates.
  public: std::string topic = "/wavefield/parameters";

  /// \brief A msgs::Param containing all parameters. This is useful to
  /// communicate all wavefield parameters via Transport.
  public: msgs::Param params;
};

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Implementation::RecalculateCwr()
{
  // Derived mean values
  this->angularFrequency = 2.0 * M_PI / this->period;
  this->wavenumber = \
    this->DeepWaterDispersionToWavenumber(this->angularFrequency);
  this->wavelength = 2.0 * M_PI / this->wavenumber;

  // Update components
  this->angularFrequencies.clear();
  this->amplitudes.clear();
  this->phases.clear();
  this->wavenumbers.clear();
  this->steepnesses.clear();
  this->directions.clear();

  for (size_t i = 0; i < this->number; ++i)
  {
    const int n = i - this->number / 2;
    const double scaleFactor = std::pow(this->scale, n);
    const double a = scaleFactor * this->amplitude;
    const double k = this->wavenumber / scaleFactor;
    const double omega = this->DeepWaterDispersionToOmega(k);
    const double phi = this->phase;
    double q = 0.0;
    if (!math::equal(a, 0.0))
    {
      q = std::min(1.0, this->steepness / (a * k * this->number));
    }

    this->amplitudes.push_back(a);
    this->angularFrequencies.push_back(omega);
    this->phases.push_back(phi);
    this->steepnesses.push_back(q);
    this->wavenumbers.push_back(k);

    // Direction
    const double c = std::cos((n * this->angle) + this->direction);
    const double s = std::sin((n * this->angle) + this->direction);

    const math::Vector2d d(c, s);

    directions.push_back(d);
  }
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Implementation::Pm(double _omega, double _omegaP) const
{
  double alpha = 0.0081;
  double g = 9.81;
  return alpha * std::pow(g, 2.0) / std::pow(_omega, 5.0) * \
    std::exp(-(5.0 / 4.0) * std::pow(_omegaP / _omega, 4.0));
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Implementation::RecalculatePms()
{
  // Derived mean values
  this->angularFrequency = 2.0 * M_PI / this->period;
  this->wavenumber = \
    this->DeepWaterDispersionToWavenumber(this->angularFrequency);
  this->wavelength = 2.0 * M_PI / this->wavenumber;

  // Update components
  this->angularFrequencies.clear();
  this->amplitudes.clear();
  this->phases.clear();
  this->wavenumbers.clear();
  this->steepnesses.clear();
  this->directions.clear();

  // Vector for spaceing
  std::vector<double> omegaSpacing;
  omegaSpacing.push_back(this->angularFrequency * (1.0 - 1.0 / this->scale));
  omegaSpacing.push_back(this->angularFrequency * \
                          (this->scale - 1.0 / this->scale) / 2.0);
  omegaSpacing.push_back(this->angularFrequency * (this->scale - 1.0));

  for (size_t i = 0; i < this->number; ++i)
  {
    const int n = i - 1;
    const double scaleFactor = std::pow(this->scale, n);
    const double omega = this->angularFrequency * scaleFactor;
    const double pms = this->Pm(omega, this->angularFrequency);
    const double a = this->gain * std::sqrt(2.0 * pms * omegaSpacing[i]);
    const double k = this->DeepWaterDispersionToWavenumber(omega);
    const double phi = this->phase;
    double q = 0.0;
    if (!math::equal(a, 0.0))
    {
      q = std::min(1.0, this->steepness / (a * k * this->number));
    }

    this->amplitudes.push_back(a);
    this->angularFrequencies.push_back(omega);
    this->phases.push_back(phi);
    this->steepnesses.push_back(q);
    this->wavenumbers.push_back(k);

    // Direction
    const double c = std::cos((n * this->angle) + this->direction);
    const double s = std::sin((n * this->angle) + this->direction);

    const math::Vector2d d(c, s);

    directions.push_back(d);
  }
}

/////////////////////////////////////////////////
void Wavefield::Implementation::Recalculate()
{
  if (!this->model.compare("PMS"))
  {
    this->RecalculatePms();
  }
  else if (!this->model.compare("CWR"))
  {
    this->RecalculateCwr();
  }
  else
  {
    gzwarn << "Wavefield model specified as <" << this->model
           << "> which is not one of the two supported wavefield models: "
           << "PMS or CWR!!!" << std::endl;
  }
}

/////////////////////////////////////////////////
double Wavefield::Implementation::DeepWaterDispersionToOmega(double _wavenumber)
  const
{
  const double g = std::fabs(-9.8);
  return std::sqrt(g * _wavenumber);
}

/////////////////////////////////////////////////
double Wavefield::Implementation::DeepWaterDispersionToWavenumber(double _omega)
   const
{
  const double g = std::fabs(-9.8);
  return _omega * _omega / g;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Implementation::FillParameters()
{
  this->params.mutable_params()->clear();
  auto *params = this->params.mutable_params();

  // size.
  gz::msgs::Any sizeValue;
  sizeValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_VECTOR3D);
  sizeValue.mutable_vector3d_value()->set_x(this->size.X());
  sizeValue.mutable_vector3d_value()->set_y(this->size.Y());
  (*params)["size"] = sizeValue;

  // cell_count
  gz::msgs::Any cellCountValue;
  cellCountValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_VECTOR3D);
  cellCountValue.mutable_vector3d_value()->set_x(this->cellCount.X());
  cellCountValue.mutable_vector3d_value()->set_y(this->cellCount.Y());
  (*params)["cell_count"] = cellCountValue;

  // number
  gz::msgs::Any numberValue;
  numberValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_INT32);
  numberValue.set_int_value(this->number);
  (*params)["number"] = numberValue;

  // scale
  gz::msgs::Any scaleValue;
  scaleValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  scaleValue.set_double_value(this->scale);
  (*params)["scale"] = scaleValue;

  // angle
  gz::msgs::Any angleValue;
  angleValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  angleValue.set_double_value(this->angle);
  (*params)["angle"] = angleValue;

  // steepness
  gz::msgs::Any steepnessValue;
  steepnessValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  steepnessValue.set_double_value(this->steepness);
  (*params)["steepness"] = steepnessValue;

  // amplitude
  gz::msgs::Any amplitudeValue;
  amplitudeValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  amplitudeValue.set_double_value(this->amplitude);
  (*params)["amplitude"] = amplitudeValue;

  // period
  gz::msgs::Any periodValue;
  periodValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  periodValue.set_double_value(this->period);
  (*params)["period"] = periodValue;

  // phase
  gz::msgs::Any phaseValue;
  phaseValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  phaseValue.set_double_value(this->phase);
  (*params)["phase"] = phaseValue;

  // direction.
  gz::msgs::Any directionValue;
  directionValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  directionValue.set_double_value(this->direction);
  (*params)["direction"] = directionValue;

  // model
  gz::msgs::Any modelValue;
  modelValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_STRING);
  modelValue.set_string_value(this->model);
  (*params)["model"] = modelValue;

  // gain
  gz::msgs::Any gainValue;
  gainValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  gainValue.set_double_value(this->gain);
  (*params)["gain"] = gainValue;

  // tau
  gz::msgs::Any tauValue;
  tauValue.set_type(gz::msgs::Any_ValueType::Any_ValueType_DOUBLE);
  tauValue.set_double_value(this->tau);
  (*params)["tau"] = tauValue;
}

/////////////////////////////////////////////////////////////////////////////
Wavefield::Wavefield()
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
Wavefield::~Wavefield()
{
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Load(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf->HasElement("wavefield"))
    return;

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  auto sdfWavefield = ptr->GetElement("wavefield");

  this->dataPtr->topic = sdfWavefield->Get<std::string>("topic",
    this->dataPtr->topic).first;
  this->dataPtr->size = sdfWavefield->Get<math::Vector2d>("size",
    this->dataPtr->size).first;
  this->dataPtr->cellCount = sdfWavefield->Get<math::Vector2d>("cell_count",
    this->dataPtr->cellCount).first;
  if (sdfWavefield->HasElement("wave"))
  {
    auto sdfWave = sdfWavefield->GetElement("wave");

    this->dataPtr->model = sdfWave->Get<std::string>("model", "PMS").first;
    this->dataPtr->number =
      sdfWave->Get<double>("number", this->dataPtr->number).first;
    this->dataPtr->amplitude =
      sdfWave->Get<double>("amplitude", this->dataPtr->amplitude).first;
    this->dataPtr->period =
      sdfWave->Get<double>("period", this->dataPtr->period).first;
    this->dataPtr->phase = sdfWave->Get<double>("phase", this->dataPtr->phase).first;
    this->dataPtr->direction =
      sdfWave->Get<double>("direction", this->dataPtr->direction).first;
    this->dataPtr->scale = sdfWave->Get<double>("scale", this->dataPtr->scale).first;
    this->dataPtr->angle = sdfWave->Get<double>("angle", this->dataPtr->angle).first;
    this->dataPtr->steepness =
      sdfWave->Get<double>("steepness", this->dataPtr->steepness).first;
    this->dataPtr->tau = sdfWave->Get<double>("tau", this->dataPtr->tau).first;
    this->dataPtr->gain = sdfWave->Get<double>("gain", this->dataPtr->gain).first;

    this->dataPtr->Recalculate();
  }

  this->dataPtr->FillParameters();
  this->DebugPrint();
  this->dataPtr->active = true;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Load(const msgs::Param &_msg)
{
  auto params = _msg.params();

  if (params.count("size") > 0)
  {
    this->dataPtr->size = {params["size"].vector3d_value().x(),
      params["size"].vector3d_value().y()};
  }
  if (params.count("cell_count") > 0)
  {
    this->dataPtr->cellCount = {params["cell_count"].vector3d_value().x(),
      params["cell_count"].vector3d_value().y()};
  }
  if (params.count("number") > 0)
  {
    this->dataPtr->number = params["number"].int_value();
  }
  if (params.count("scale") > 0)
  {
    this->dataPtr->scale = params["scale"].double_value();
  }
  if (params.count("angle") > 0)
  {
    this->dataPtr->angle = params["angle"].double_value();
  }
  if (params.count("steepness") > 0)
  {
    this->dataPtr->steepness = params["steepness"].double_value();
  }
  if (params.count("amplitude") > 0)
  {
    this->dataPtr->amplitude = params["amplitude"].double_value();
  }
  if (params.count("period") > 0)
  {
    this->dataPtr->period = params["period"].double_value();
  }
  if (params.count("phase") > 0)
  {
    this->dataPtr->phase = params["phase"].double_value();
  }
  if (params.count("direction") > 0)
  {
    this->dataPtr->direction = params["direction"].double_value();
  }
  if (params.count("model") > 0)
  {
    this->dataPtr->model = params["model"].string_value();
  }
  if (params.count("gain") > 0)
  {
    this->dataPtr->gain = params["gain"].double_value();
  }
  if (params.count("tau") > 0)
  {
    this->dataPtr->tau = params["tau"].double_value();
  }

  this->dataPtr->FillParameters();
  this->dataPtr->Recalculate();
  this->dataPtr->active = true;
}

///////////////////////////////////////////////////////////////////////////////
std::string Wavefield::Topic() const
{
  return this->dataPtr->topic;
}

///////////////////////////////////////////////////////////////////////////////
bool Wavefield::Active() const
{
  return this->dataPtr->active;
}

///////////////////////////////////////////////////////////////////////////////
size_t Wavefield::Number() const
{
  return this->dataPtr->number;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Angle() const
{
  return this->dataPtr->angle;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Scale() const
{
  return this->dataPtr->scale;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Steepness() const
{
  return this->dataPtr->steepness;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::AngularFrequency() const
{
  return this->dataPtr->angularFrequency;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Amplitude() const
{
  return this->dataPtr->amplitude;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Period() const
{
  return this->dataPtr->period;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Phase() const
{
  return this->dataPtr->phase;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Wavelength() const
{
  return this->dataPtr->wavelength;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Wavenumber() const
{
  return this->dataPtr->wavenumber;
}

///////////////////////////////////////////////////////////////////////////////
float Wavefield::Tau() const
{
  return this->dataPtr->tau;
}

///////////////////////////////////////////////////////////////////////////////
float Wavefield::Gain() const
{
  return this->dataPtr->gain;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Direction() const
{
  return this->dataPtr->direction;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetNumber(size_t _number)
{
  this->dataPtr->number = _number;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetAngle(double _angle)
{
  this->dataPtr->angle = _angle;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetScale(double _scale)
{
  this->dataPtr->scale = _scale;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetSteepness(double _steepness)
{
  this->dataPtr->steepness = _steepness;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetAmplitude(double _amplitude)
{
  this->dataPtr->amplitude = _amplitude;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetPeriod(double _period)
{
  this->dataPtr->period = _period;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetPhase(double _phase)
{
  this->dataPtr->phase = _phase;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetTau(double _tau)
{
  this->dataPtr->tau = _tau;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetGain(double _gain)
{
  this->dataPtr->gain = _gain;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetDirection(double _direction)
{
  this->dataPtr->direction = _direction;
  this->dataPtr->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::AngularFrequency_V() const
{
  return this->dataPtr->angularFrequencies;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Amplitude_V() const
{
  return this->dataPtr->amplitudes;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Phase_V() const
{
  return this->dataPtr->phases;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Steepness_V() const
{
  return this->dataPtr->steepnesses;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Wavenumber_V() const
{
  return this->dataPtr->wavenumbers;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<math::Vector2d> &Wavefield::Direction_V() const
{
  return this->dataPtr->directions;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::DebugPrint() const
{
  gzmsg << "Input Parameters:" << std::endl;
  gzmsg << "model:     " << this->dataPtr->model << std::endl;
  gzmsg << "number:     " << this->dataPtr->number << std::endl;
  gzmsg << "scale:      " << this->dataPtr->scale << std::endl;
  gzmsg << "angle:      " << this->dataPtr->angle << std::endl;
  gzmsg << "steepness:  " << this->dataPtr->steepness << std::endl;
  gzmsg << "amplitude:  " << this->dataPtr->amplitude << std::endl;
  gzmsg << "period:     " << this->dataPtr->period << std::endl;
  gzmsg << "direction:  " << this->dataPtr->direction << std::endl;
  gzmsg << "tau:  " << this->dataPtr->tau << std::endl;
  gzmsg << "gain:  " << this->dataPtr->gain << std::endl;
  gzmsg << "Derived Parameters:" << std::endl;
  gzmsg << "amplitudes:  " << this->dataPtr->amplitudes << std::endl;
  gzmsg << "wavenumbers: " << this->dataPtr->wavenumbers << std::endl;
  gzmsg << "omegas:      " << this->dataPtr->angularFrequencies << std::endl;
  gzmsg << "periods:     ";
  for (auto&& omega : this->dataPtr->angularFrequencies) // NOLINT
  {
    gzmsg << 2.0 * M_PI / omega <<", ";
  }
  gzmsg << std::endl;
  gzmsg << "phases:      " << this->dataPtr->phases << std::endl;
  gzmsg << "steepnesses: " << this->dataPtr->steepnesses << std::endl;
  gzmsg << "directions:  ";
  for (auto&& d : this->dataPtr->directions) // NOLINT
  {
    gzmsg << d << "; ";
  }
  gzmsg << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::ComputeDepthSimply(const math::Vector3d &_point,
  double _time, double _timeInit)
{
  double h = 0.0;
  for (std::size_t i = 0; i < this->Number(); ++i)
  {
    double k = this->Wavenumber_V()[i];
    double a = this->Amplitude_V()[i];
    double dx =  this->Direction_V()[i].X();
    double dy =  this->Direction_V()[i].Y();
    double dot = _point.X() * dx + _point.Y() * dy;
    double omega = this->AngularFrequency_V()[i];
    double theta = k * dot - omega * _time;
    double c = cos(theta);
    h += a*c;
  }

  // Exponentially grow the waves
  return h * (1 - exp(-1.0 * (_time - _timeInit) / this->Tau()));
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::ComputeDepthDirectly(const math::Vector3d &_point,
  double _time, double _timeInit)
{
  // Struture for passing wave parameters to lambdas
  struct WaveParams
  {
    WaveParams(
      const std::vector<double>& _a,
      const std::vector<double>& _k,
      const std::vector<double>& _omega,
      const std::vector<double>& _phi,
      const std::vector<double>& _q,
      const std::vector<math::Vector2d>& _dir) :
      a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

    const std::vector<double>& a;
    const std::vector<double>& k;
    const std::vector<double>& omega;
    const std::vector<double>& phi;
    const std::vector<double>& q;
    const std::vector<math::Vector2d>& dir;
  };

  // Compute the target function and Jacobian. Also calculate pz,
  // the z-component of the Gerstner wave, which we essentially get for free.
  // cppcheck-suppress constParameter
  auto wave_fdf = [=](auto x, auto p, auto t, auto &wp, auto &F, auto &J)
  {
    double pz = 0;
    F(0) = p.x() - x.x();
    F(1) = p.y() - x.y();
    J(0, 0) = -1;
    J(0, 1) =  0;
    J(1, 0) =  0;
    J(1, 1) = -1;
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double dx = wp.dir[i].X();
      const double dy = wp.dir[i].Y();
      const double q = wp.q[i];
      const double a = wp.a[i];
      const double k = wp.k[i];
      const double dot = x.x() * dx + x.y() * dy;
      const double theta = k * dot - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double c = std::cos(theta);
      const double qakc = q * a * k * c;
      const double df1x = qakc * dx * dx;
      const double df1y = qakc * dx * dy;
      const double df2x = df1y;
      const double df2y = qakc * dy * dy;
      pz += a * c;
      F(0) += a * dx * s;
      F(1) += a * dy * s;
      J(0, 0) += df1x;
      J(0, 1) += df1y;
      J(1, 0) += df2x;
      J(1, 1) += df2y;
    }
    // Exponentially grow the waves
    return pz * (1 - exp(-1.0 * (_time - _timeInit) / this->Tau()));
  };

  // Simple multi-variate Newton solver -
  // this version returns the z-component of the
  // wave field at the desired point p.
  // cppcheck-suppress constParameter
  auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, \
                    auto& wp, auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    double pz = 0;
    auto xn = x0;
    Eigen::Vector2d F;
    Eigen::Matrix2d J;
    while (std::abs(err) > tol && n < nmax)
    {
      pz = fdfunc(x0, p, t, wp, F, J);
      xn = x0 - J.inverse() * F;
      x0 = xn;
      err = F.norm();
      n++;
    }
    return pz;
  };

  // Set up parameter references
  WaveParams wp(
    this->Amplitude_V(),
    this->Wavenumber_V(),
    this->AngularFrequency_V(),
    this->Phase_V(),
    this->Steepness_V(),
    this->Direction_V());

  // Tolerances etc.
  const double tol = 1.0E-10;
  const double nmax = 30;

  // Use the target point as the initial guess
  // (this is within sum{amplitudes} of the solution)
  Eigen::Vector2d p2(_point.X(), _point.Y());
  const double pz = solver(wave_fdf, p2, p2, _time, wp, tol, nmax);
  // Removed so that height is reported relative to mean water level
  // const double h = pz - _point.Z();
  const double h = pz;
  return h;
}

///////////////////////////////////////////////////////////////////////////////
msgs::Param Wavefield::Parameters() const
{
  return this->dataPtr->params;
}
