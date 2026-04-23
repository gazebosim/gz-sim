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

// The mathematical formulation follows the WMM Technical Report:
// "The US/UK World Magnetic Model for 2025-2030" (NOAA/NCEI).
// The WMM algorithm and coefficients are in the public domain
// (US Government work, 17 U.S.C. 403).

#ifndef GZ_SIM_SYSTEMS_WORLDMAGNETICMODEL_HH_
#define GZ_SIM_SYSTEMS_WORLDMAGNETICMODEL_HH_

#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Result of a WMM field computation.
  struct WMMFieldResult
  {
    /// \brief Magnetic field vector in geodetic NED frame (nanotesla).
    /// X = North, Y = East, Z = Down.
    math::Vector3d fieldNED;

    /// \brief Magnetic declination in radians.
    double declination{0.0};

    /// \brief Magnetic inclination in radians.
    double inclination{0.0};

    /// \brief Total field intensity in nanotesla.
    double totalIntensity{0.0};
  };

  /// \brief Native C++ implementation of the NOAA World Magnetic Model.
  ///
  /// Computes Earth's magnetic field at any geodetic location using
  /// spherical harmonic expansion. Loads coefficients from a standard
  /// WMM/WMMHR .COF file. Supports any spherical harmonic degree
  /// (standard WMM degree 12, WMMHR degree 133).
  ///
  /// After Load(), ComputeField() is thread-safe (read-only access to
  /// model data; all scratch buffers are allocated per-call on the stack
  /// or as local vectors).
  class WorldMagneticModel
  {
    /// \brief Constructor.
    public: WorldMagneticModel();

    /// \brief Destructor.
    public: ~WorldMagneticModel();

    /// \brief Load model coefficients from a .COF file.
    /// \param[in] _cofFilePath Path to the coefficient file.
    /// \param[in] _decimalYear Decimal year for computation (e.g. 2025.5).
    ///   If <= 0, the current system date is used.
    /// \return True on success.
    public: bool Load(const std::string &_cofFilePath,
                      double _decimalYear = -1.0);

    /// \brief Compute the magnetic field at a geodetic location.
    /// \param[in] _latDeg Geodetic latitude in degrees [-90, 90].
    /// \param[in] _lonDeg Geodetic longitude in degrees [-180, 360].
    /// \param[in] _altKm Height above WGS84 ellipsoid in kilometers.
    /// \return Computed field result, or zero if model not loaded.
    public: WMMFieldResult ComputeField(
                double _latDeg, double _lonDeg, double _altKm) const;

    /// \brief Check if the model is loaded.
    public: bool IsLoaded() const;

    /// \brief Get the model name (e.g. "WMMHR-2025").
    public: std::string ModelName() const;

    /// \brief Get the max spherical harmonic degree.
    public: int MaxDegree() const;

    /// \brief Get the decimal year used for computations.
    public: double DecimalYear() const;

    // -- WGS84 ellipsoid constants --
    /// \brief Semi-major axis in km.
    private: static constexpr double kWGS84_A = 6378.137;

    /// \brief Semi-minor axis in km.
    private: static constexpr double kWGS84_B = 6356.7523142;

    /// \brief First eccentricity squared.
    private: static constexpr double kWGS84_ESQ =
        (kWGS84_A * kWGS84_A - kWGS84_B * kWGS84_B) /
        (kWGS84_A * kWGS84_A);

    /// \brief Mean radius of the ellipsoid in km.
    private: static constexpr double kWGS84_RE = 6371.2;

    /// \brief Maximum spherical harmonic degree of the loaded model.
    private: int nMax{0};

    /// \brief Maximum degree with secular variation data.
    private: int nMaxSecVar{0};

    /// \brief Model epoch (decimal year).
    private: double epoch{0.0};

    /// \brief The decimal year for time-adjusted computations.
    private: double decimalYear{0.0};

    /// \brief Model name string.
    private: std::string modelName;

    /// \brief Whether model is loaded.
    private: bool loaded{false};

    /// \brief Time-adjusted Gauss coefficients g(n,m).
    /// Indexed as [n*(n+1)/2 + m].
    private: std::vector<double> gCoeff;

    /// \brief Time-adjusted Gauss coefficients h(n,m).
    private: std::vector<double> hCoeff;

    /// \brief Compute Schmidt semi-normalized associated Legendre
    /// polynomials and their derivatives with respect to latitude.
    /// Uses the Holmes & Featherstone (2002) algorithm for numerical
    /// stability at high degree.
    /// \param[in] _sinLat sin(geocentric latitude).
    /// \param[in] _nMax Maximum degree.
    /// \param[out] _Pcup Legendre function values.
    /// \param[out] _dPcup Derivatives w.r.t. latitude.
    /// \return True on success.
    private: static bool ComputeLegendre(
        double _sinLat, int _nMax,
        std::vector<double> &_Pcup,
        std::vector<double> &_dPcup);

    /// \brief Special summation for By at geographic poles.
    /// \param[in] _sinLat sin(geocentric latitude).
    /// \param[in] _relRadPower Relative radius power array.
    /// \param[in] _sinMLambda sin(m*lambda) array.
    /// \param[in] _cosMLambda cos(m*lambda) array.
    /// \return By component in nT.
    private: double SummationSpecial(
        double _sinLat,
        const std::vector<double> &_relRadPower,
        const std::vector<double> &_sinMLambda,
        const std::vector<double> &_cosMLambda) const;
  };
}
}
}
}
#endif
