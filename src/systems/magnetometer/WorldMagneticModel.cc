/*
 * Copyright (C) 2026 Rudis Laboratories
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

// Native C++ implementation of the World Magnetic Model spherical harmonic
// computation. The mathematical formulation follows the WMM Technical Report
// (NOAA/NCEI). Equations referenced by number correspond to that report.
//
// The WMM algorithm and coefficients are in the public domain
// (US Government work, 17 U.S.C. 403).

#include "WorldMagneticModel.hh"

#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include <gz/common/Console.hh>

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
WorldMagneticModel::WorldMagneticModel() = default;

//////////////////////////////////////////////////
WorldMagneticModel::~WorldMagneticModel() = default;

//////////////////////////////////////////////////
static double CurrentDecimalYear()
{
  std::time_t now = std::time(nullptr);
  std::tm *tm = std::gmtime(&now);
  int year = tm->tm_year + 1900;
  int yday = tm->tm_yday;
  int daysInYear = ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
                       ? 366
                       : 365;
  return year + static_cast<double>(yday) / daysInYear;
}

//////////////////////////////////////////////////
bool WorldMagneticModel::Load(const std::string &_cofFilePath,
                              double _decimalYear)
{
  std::ifstream file(_cofFilePath);
  if (!file.is_open())
  {
    gzerr << "WorldMagneticModel: Cannot open [" << _cofFilePath << "]"
           << std::endl;
    return false;
  }

  // Parse header line: "epoch  ModelName  releaseDate"
  std::string headerLine;
  if (!std::getline(file, headerLine))
  {
    gzerr << "WorldMagneticModel: Empty coefficient file." << std::endl;
    return false;
  }

  {
    std::istringstream hss(headerLine);
    std::string name;
    hss >> this->epoch >> name;
    this->modelName = name;
  }

  // First pass: read all coefficients to determine nMax and nMaxSecVar
  struct CoeffEntry
  {
    int n, m;
    double gnm, hnm, dgnm, dhnm;
  };
  std::vector<CoeffEntry> entries;
  int maxN = 0;
  int maxNSV = 0;

  std::string line;
  while (std::getline(file, line))
  {
    // Terminator line is all 9s
    if (line.find("9999") != std::string::npos)
      break;

    std::istringstream lss(line);
    CoeffEntry e;
    if (!(lss >> e.n >> e.m >> e.gnm >> e.hnm >> e.dgnm >> e.dhnm))
      continue;

    entries.push_back(e);

    if (e.n > maxN)
      maxN = e.n;
    if ((std::abs(e.dgnm) > 0.0 || std::abs(e.dhnm) > 0.0) && e.n > maxNSV)
      maxNSV = e.n;
  }
  file.close();

  if (entries.empty() || maxN == 0)
  {
    gzerr << "WorldMagneticModel: No valid coefficients found." << std::endl;
    return false;
  }

  this->nMax = maxN;
  this->nMaxSecVar = maxNSV;

  // Determine decimal year
  if (_decimalYear <= 0.0)
    this->decimalYear = CurrentDecimalYear();
  else
    this->decimalYear = _decimalYear;

  double dt = this->decimalYear - this->epoch;

  // Allocate and fill time-adjusted coefficients
  // Index: n*(n+1)/2 + m
  int numTerms = (this->nMax + 1) * (this->nMax + 2) / 2;
  this->gCoeff.assign(numTerms, 0.0);
  this->hCoeff.assign(numTerms, 0.0);

  for (const auto &e : entries)
  {
    int idx = e.n * (e.n + 1) / 2 + e.m;
    // Time-adjust: g(t) = g(epoch) + dt * dg/dt  (Equation 19)
    this->gCoeff[idx] = e.gnm + dt * e.dgnm;
    this->hCoeff[idx] = e.hnm + dt * e.dhnm;
  }

  this->loaded = true;

  gzmsg << "WorldMagneticModel: Loaded [" << this->modelName
        << "] epoch " << this->epoch
        << ", nMax=" << this->nMax
        << ", nMaxSecVar=" << this->nMaxSecVar
        << ", decimal year=" << this->decimalYear << std::endl;

  return true;
}

//////////////////////////////////////////////////
bool WorldMagneticModel::ComputeLegendre(
    double _sinLat, int _nMax,
    std::vector<double> &_Pcup,
    std::vector<double> &_dPcup)
{
  // Holmes & Featherstone (2002) algorithm for Schmidt semi-normalized
  // associated Legendre functions. Handles high degree (nMax >> 16)
  // by scaling with 10^280 to avoid underflow near poles.

  int numTerms = (_nMax + 1) * (_nMax + 2) / 2;
  _Pcup.assign(numTerms, 0.0);
  _dPcup.assign(numTerms, 0.0);

  double z = std::sqrt((1.0 - _sinLat) * (1.0 + _sinLat));

  _Pcup[0] = 1.0;
  _dPcup[0] = 0.0;

  if (_nMax == 0)
    return true;

  // At geographic poles, z==0 and derivatives can't be computed normally.
  // Return false to trigger the special polar summation path.
  if (z < 1.0e-10)
    return false;

  // Precompute sqrt(n) table
  std::vector<double> preSqr(2 * _nMax + 2);
  for (int n = 0; n <= 2 * _nMax + 1; ++n)
    preSqr[n] = std::sqrt(static_cast<double>(n));

  // Precompute recursion coefficients f1, f2
  std::vector<double> f1(numTerms + 1, 0.0);
  std::vector<double> f2(numTerms + 1, 0.0);

  int k = 2;
  for (int n = 2; n <= _nMax; n++)
  {
    k++;
    f1[k] = static_cast<double>(2 * n - 1) / static_cast<double>(n);
    f2[k] = static_cast<double>(n - 1) / static_cast<double>(n);
    for (int m = 1; m <= n - 2; m++)
    {
      k++;
      f1[k] = static_cast<double>(2 * n - 1) /
              (preSqr[n + m] * preSqr[n - m]);
      f2[k] = (preSqr[n - m - 1] * preSqr[n + m - 1]) /
              (preSqr[n + m] * preSqr[n - m]);
    }
    k += 2;
  }

  // Compute P(n,0) - the zonal terms
  double pm2 = 1.0;
  double pm1 = _sinLat;
  _Pcup[1] = pm1;
  _dPcup[1] = z;

  k = 1;
  for (int n = 2; n <= _nMax; n++)
  {
    k += n;
    double plm = f1[k] * _sinLat * pm1 - f2[k] * pm2;
    _Pcup[k] = plm;
    _dPcup[k] = static_cast<double>(n) * (pm1 - _sinLat * plm) / z;
    pm2 = pm1;
    pm1 = plm;
  }

  // Compute P(n,m) for m >= 1 using scaled recursion
  constexpr double scalef = 1.0e-280;
  double pmm = preSqr[2] * scalef;
  double rescalem = 1.0 / scalef;
  int kstart = 0;

  for (int m = 1; m <= _nMax - 1; ++m)
  {
    rescalem *= z;

    // P(m,m)
    kstart += m + 1;
    pmm *= preSqr[2 * m + 1] / preSqr[2 * m];
    _Pcup[kstart] = pmm * rescalem / preSqr[2 * m + 1];
    _dPcup[kstart] = -(static_cast<double>(m) * _sinLat *
                        _Pcup[kstart] / z);
    pm2 = pmm / preSqr[2 * m + 1];

    // P(m+1,m)
    k = kstart + m + 1;
    pm1 = _sinLat * preSqr[2 * m + 1] * pm2;
    _Pcup[k] = pm1 * rescalem;
    _dPcup[k] = ((pm2 * rescalem) * preSqr[2 * m + 1] -
                  _sinLat * static_cast<double>(m + 1) * _Pcup[k]) / z;

    // P(n,m) for n >= m+2
    for (int n = m + 2; n <= _nMax; ++n)
    {
      k += n;
      double plm = _sinLat * f1[k] * pm1 - f2[k] * pm2;
      _Pcup[k] = plm * rescalem;
      _dPcup[k] = (preSqr[n + m] * preSqr[n - m] * (pm1 * rescalem) -
                    static_cast<double>(n) * _sinLat * _Pcup[k]) / z;
      pm2 = pm1;
      pm1 = plm;
    }
  }

  // P(nMax, nMax)
  rescalem *= z;
  kstart += _nMax + 1;
  pmm /= preSqr[2 * _nMax];
  _Pcup[kstart] = pmm * rescalem;
  _dPcup[kstart] = -(static_cast<double>(_nMax) * _sinLat *
                      _Pcup[kstart] / z);

  return true;
}

//////////////////////////////////////////////////
double WorldMagneticModel::SummationSpecial(
    double _sinLat,
    const std::vector<double> &_relRadPower,
    const std::vector<double> &_sinMLambda,
    const std::vector<double> &_cosMLambda) const
{
  // Special calculation for By at geographic poles where cos(phi) ~ 0.
  // See WMM Technical Report Section 1.4.
  std::vector<double> PcupS(this->nMax + 1);
  PcupS[0] = 1.0;

  double schmidtQuasiNorm1 = 1.0;
  double By = 0.0;

  for (int n = 1; n <= this->nMax; n++)
  {
    int index = n * (n + 1) / 2 + 1;
    double schmidtQuasiNorm2 =
        schmidtQuasiNorm1 * static_cast<double>(2 * n - 1) /
        static_cast<double>(n);
    double schmidtQuasiNorm3 =
        schmidtQuasiNorm2 * std::sqrt(
            static_cast<double>(n * 2) / static_cast<double>(n + 1));
    schmidtQuasiNorm1 = schmidtQuasiNorm2;

    if (n == 1)
      PcupS[n] = PcupS[n - 1];
    else
    {
      double kk = static_cast<double>((n - 1) * (n - 1) - 1) /
                  static_cast<double>((2 * n - 1) * (2 * n - 3));
      PcupS[n] = _sinLat * PcupS[n - 1] - kk * PcupS[n - 2];
    }

    By += _relRadPower[n] *
          (this->gCoeff[index] * _sinMLambda[1] -
           this->hCoeff[index] * _cosMLambda[1]) *
          PcupS[n] * schmidtQuasiNorm3;
  }

  return By;
}

//////////////////////////////////////////////////
WMMFieldResult WorldMagneticModel::ComputeField(
    double _latDeg, double _lonDeg, double _altKm) const
{
  WMMFieldResult result;
  if (!this->loaded)
    return result;

  // Clamp latitude to avoid singularity issues
  constexpr double kPoleTol = 1e-5;
  if (_latDeg < -90.0 + kPoleTol) _latDeg = -90.0 + kPoleTol;
  if (_latDeg > 90.0 - kPoleTol) _latDeg = 90.0 - kPoleTol;

  constexpr double kDeg2Rad = GZ_PI / 180.0;
  constexpr double kRad2Deg = 180.0 / GZ_PI;

  // --- Step 1: Geodetic to Spherical conversion (Equations 17-18) ---
  double cosLat = std::cos(_latDeg * kDeg2Rad);
  double sinLat = std::sin(_latDeg * kDeg2Rad);

  // Local radius of curvature on WGS84
  double rc = kWGS84_A / std::sqrt(1.0 - kWGS84_ESQ * sinLat * sinLat);

  // ECEF Cartesian (for longitude=0)
  double xp = (rc + _altKm) * cosLat;
  double zp = (rc * (1.0 - kWGS84_ESQ) + _altKm) * sinLat;

  // Spherical radius and geocentric latitude
  double r = std::sqrt(xp * xp + zp * zp);
  double phigDeg = std::asin(zp / r) * kRad2Deg;  // geocentric latitude
  double phigRad = phigDeg * kDeg2Rad;

  // --- Step 2: Spherical harmonic variables (Equations 10-12) ---
  // (a/r)^(n+2) power series
  std::vector<double> relRadPower(this->nMax + 1);
  double ratio = kWGS84_RE / r;
  relRadPower[0] = ratio * ratio;
  for (int n = 1; n <= this->nMax; n++)
    relRadPower[n] = relRadPower[n - 1] * ratio;

  // cos(m*lambda), sin(m*lambda) via angle addition
  double cosLambda = std::cos(_lonDeg * kDeg2Rad);
  double sinLambda = std::sin(_lonDeg * kDeg2Rad);

  std::vector<double> cosMLambda(this->nMax + 1);
  std::vector<double> sinMLambda(this->nMax + 1);
  cosMLambda[0] = 1.0;
  sinMLambda[0] = 0.0;
  if (this->nMax >= 1)
  {
    cosMLambda[1] = cosLambda;
    sinMLambda[1] = sinLambda;
  }
  for (int m = 2; m <= this->nMax; m++)
  {
    cosMLambda[m] = cosMLambda[m - 1] * cosLambda -
                    sinMLambda[m - 1] * sinLambda;
    sinMLambda[m] = cosMLambda[m - 1] * sinLambda +
                    sinMLambda[m - 1] * cosLambda;
  }

  // --- Step 3: Associated Legendre polynomials ---
  std::vector<double> Pcup, dPcup;
  double sinPhig = std::sin(phigRad);
  bool legendreOK = ComputeLegendre(sinPhig, this->nMax, Pcup, dPcup);

  // --- Step 4: Spherical harmonic summation (Equations 10-12) ---
  double Bx_sph = 0.0;
  double By_sph = 0.0;
  double Bz_sph = 0.0;

  if (legendreOK)
  {
    for (int n = 1; n <= this->nMax; n++)
    {
      for (int m = 0; m <= n; m++)
      {
        int idx = n * (n + 1) / 2 + m;
        double gCos_hSin = this->gCoeff[idx] * cosMLambda[m] +
                           this->hCoeff[idx] * sinMLambda[m];
        double gSin_hCos = this->gCoeff[idx] * sinMLambda[m] -
                           this->hCoeff[idx] * cosMLambda[m];

        // Bz: derivative w.r.t. radius (Eq 12)
        Bz_sph -= relRadPower[n] *
                  static_cast<double>(n + 1) * gCos_hSin * Pcup[idx];

        // By: derivative w.r.t. longitude / radius (Eq 11)
        By_sph += relRadPower[n] *
                  static_cast<double>(m) * gSin_hCos * Pcup[idx];

        // Bx: derivative w.r.t. latitude / radius (Eq 10)
        Bx_sph -= relRadPower[n] * gCos_hSin * dPcup[idx];
      }
    }

    // Divide By by cos(geocentric latitude)
    double cosPhi = std::cos(phigRad);
    if (std::abs(cosPhi) > 1.0e-10)
    {
      By_sph /= cosPhi;
    }
    else
    {
      By_sph = SummationSpecial(
          sinPhig, relRadPower, sinMLambda, cosMLambda);
    }
  }
  else
  {
    // Legendre computation failed at poles - use special path for By
    By_sph = SummationSpecial(
        sinPhig, relRadPower, sinMLambda, cosMLambda);
  }

  // --- Step 5: Rotate from spherical to geodetic frame (Equation 16) ---
  double psi = (phigDeg - _latDeg) * kDeg2Rad;
  double sinPsi = std::sin(psi);
  double cosPsi = std::cos(psi);

  double Bx_geo = Bx_sph * cosPsi - Bz_sph * sinPsi;  // North
  double By_geo = By_sph;                                // East
  double Bz_geo = Bx_sph * sinPsi + Bz_sph * cosPsi;   // Down

  // --- Step 6: Compute derived elements ---
  result.fieldNED = math::Vector3d(Bx_geo, By_geo, Bz_geo);
  double H = std::sqrt(Bx_geo * Bx_geo + By_geo * By_geo);
  result.totalIntensity = std::sqrt(H * H + Bz_geo * Bz_geo);
  result.declination = std::atan2(By_geo, Bx_geo);
  result.inclination = std::atan2(Bz_geo, H);

  return result;
}

//////////////////////////////////////////////////
bool WorldMagneticModel::IsLoaded() const
{
  return this->loaded;
}

//////////////////////////////////////////////////
std::string WorldMagneticModel::ModelName() const
{
  return this->modelName;
}

//////////////////////////////////////////////////
int WorldMagneticModel::MaxDegree() const
{
  return this->nMax;
}

//////////////////////////////////////////////////
double WorldMagneticModel::DecimalYear() const
{
  return this->decimalYear;
}
