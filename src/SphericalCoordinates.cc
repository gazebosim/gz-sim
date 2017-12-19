/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <string>

#include "ignition/math/Matrix3.hh"
#include "ignition/math/SphericalCoordinates.hh"

using namespace ignition;
using namespace math;

// Parameters for EARTH_WGS84 model
// wikipedia: World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84

// a: Equatorial radius. Semi-major axis of the WGS84 spheroid (meters).
const double g_EarthWGS84AxisEquatorial = 6378137.0;

// b: Polar radius. Semi-minor axis of the wgs84 spheroid (meters).
const double g_EarthWGS84AxisPolar = 6356752.314245;

// if: WGS84 inverse flattening parameter (no units)
const double g_EarthWGS84Flattening = 1.0/298.257223563;

// Radius of the Earth (meters).
const double g_EarthRadius = 6371000.0;

// Private data for the SphericalCoordinates class.
class ignition::math::SphericalCoordinatesPrivate
{
  /// \brief Type of surface being used.
  public: SphericalCoordinates::SurfaceType surfaceType;

  /// \brief Latitude of reference point.
  public: ignition::math::Angle latitudeReference;

  /// \brief Longitude of reference point.
  public: ignition::math::Angle longitudeReference;

  /// \brief Elevation of reference point relative to sea level in meters.
  public: double elevationReference;

  /// \brief Heading offset, expressed as angle from East to
  ///        gazebo x-axis, or equivalently from North to gazebo y-axis.
  public: ignition::math::Angle headingOffset;

  /// \brief Semi-major axis ellipse parameter
  public: double ellA;

  /// \brief Semi-minor axis ellipse parameter
  public: double ellB;

  /// \brief Flattening ellipse parameter
  public: double ellF;

  /// \brief First eccentricity ellipse parameter
  public: double ellE;

  /// \brief Second eccentricity ellipse parameter
  public: double ellP;

  /// \brief Rotation matrix that moves ECEF to GLOBAL
  public: ignition::math::Matrix3d rotECEFToGlobal;

  /// \brief Rotation matrix that moves GLOBAL to ECEF
  public: ignition::math::Matrix3d rotGlobalToECEF;

  /// \brief Cache the ECEF position of the the origin
  public: ignition::math::Vector3d origin;

  /// \brief Cache cosine head transform
  public: double cosHea;

  /// \brief Cache sine head transform
  public: double sinHea;
};

//////////////////////////////////////////////////
SphericalCoordinates::SurfaceType SphericalCoordinates::Convert(
  const std::string &_str)
{
  if ("EARTH_WGS84" == _str)
    return EARTH_WGS84;

  std::cerr << "SurfaceType string not recognized, "
    << "EARTH_WGS84 returned by default" << std::endl;
  return EARTH_WGS84;
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates()
  : dataPtr(new SphericalCoordinatesPrivate)
{
  this->SetSurface(EARTH_WGS84);
  this->SetElevationReference(0.0);
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _type)
  : dataPtr(new SphericalCoordinatesPrivate)
{
  this->SetSurface(_type);
  this->SetElevationReference(0.0);
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _type,
    const ignition::math::Angle &_latitude,
    const ignition::math::Angle &_longitude,
    const double _elevation,
    const ignition::math::Angle &_heading)
: dataPtr(new SphericalCoordinatesPrivate)
{
  // Set the reference and calculate ellipse parameters
  this->SetSurface(_type);

  // Set the coordinate transform parameters
  this->dataPtr->latitudeReference = _latitude;
  this->dataPtr->longitudeReference = _longitude;
  this->dataPtr->elevationReference = _elevation;
  this->dataPtr->headingOffset = _heading;

  // Generate transformation matrix
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SphericalCoordinates &_sc)
  : SphericalCoordinates()
{
  (*this) = _sc;
}

//////////////////////////////////////////////////
SphericalCoordinates::~SphericalCoordinates()
{
}

//////////////////////////////////////////////////
SphericalCoordinates::SurfaceType SphericalCoordinates::Surface() const
{
  return this->dataPtr->surfaceType;
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::LatitudeReference() const
{
  return this->dataPtr->latitudeReference;
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::LongitudeReference() const
{
  return this->dataPtr->longitudeReference;
}

//////////////////////////////////////////////////
double SphericalCoordinates::ElevationReference() const
{
  return this->dataPtr->elevationReference;
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::HeadingOffset() const
{
  return this->dataPtr->headingOffset;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetSurface(const SurfaceType &_type)
{
  this->dataPtr->surfaceType = _type;

  switch (this->dataPtr->surfaceType)
  {
    case EARTH_WGS84:
      {
      // Set the semi-major axis
      this->dataPtr->ellA = g_EarthWGS84AxisEquatorial;

      // Set the semi-minor axis
      this->dataPtr->ellB = g_EarthWGS84AxisPolar;

      // Set the flattening parameter
      this->dataPtr->ellF = g_EarthWGS84Flattening;

      // Set the first eccentricity ellipse parameter
      // https://en.wikipedia.org/wiki/Eccentricity_(mathematics)#Ellipses
      this->dataPtr->ellE = sqrt(1.0 -
          std::pow(this->dataPtr->ellB, 2) / std::pow(this->dataPtr->ellA, 2));

      // Set the second eccentricity ellipse parameter
      // https://en.wikipedia.org/wiki/Eccentricity_(mathematics)#Ellipses
      this->dataPtr->ellP = sqrt(
          std::pow(this->dataPtr->ellA, 2) / std::pow(this->dataPtr->ellB, 2) -
          1.0);

      break;
      }
    default:
      {
        std::cerr << "Unknown surface type["
          << this->dataPtr->surfaceType << "]\n";
      break;
      }
  }
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLatitudeReference(
    const ignition::math::Angle &_angle)
{
  this->dataPtr->latitudeReference = _angle;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLongitudeReference(
    const ignition::math::Angle &_angle)
{
  this->dataPtr->longitudeReference = _angle;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetElevationReference(const double _elevation)
{
  this->dataPtr->elevationReference = _elevation;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetHeadingOffset(const ignition::math::Angle &_angle)
{
  this->dataPtr->headingOffset.Radian(_angle.Radian());
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::SphericalFromLocalPosition(
    const ignition::math::Vector3d &_xyz) const
{
  ignition::math::Vector3d result =
    this->PositionTransform(_xyz, LOCAL, SPHERICAL);
  result.X(IGN_RTOD(result.X()));
  result.Y(IGN_RTOD(result.Y()));
  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::LocalFromSphericalPosition(
    const ignition::math::Vector3d &_xyz) const
{
  ignition::math::Vector3d result = _xyz;
  result.X(IGN_DTOR(result.X()));
  result.Y(IGN_DTOR(result.Y()));
  return this->PositionTransform(result, SPHERICAL, LOCAL);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::GlobalFromLocalVelocity(
    const ignition::math::Vector3d &_xyz) const
{
  return this->VelocityTransform(_xyz, LOCAL, GLOBAL);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::LocalFromGlobalVelocity(
    const ignition::math::Vector3d &_xyz) const
{
  return this->VelocityTransform(_xyz, GLOBAL, LOCAL);
}

//////////////////////////////////////////////////
/// Based on Haversine formula (http://en.wikipedia.org/wiki/Haversine_formula).
double SphericalCoordinates::Distance(const ignition::math::Angle &_latA,
                                      const ignition::math::Angle &_lonA,
                                      const ignition::math::Angle &_latB,
                                      const ignition::math::Angle &_lonB)
{
  ignition::math::Angle dLat = _latB - _latA;
  ignition::math::Angle dLon = _lonB - _lonA;

  double a = sin(dLat.Radian() / 2) * sin(dLat.Radian() / 2) +
             sin(dLon.Radian() / 2) * sin(dLon.Radian() / 2) *
             cos(_latA.Radian()) * cos(_latB.Radian());

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = g_EarthRadius * c;
  return d;
}

//////////////////////////////////////////////////
void SphericalCoordinates::UpdateTransformationMatrix()
{
  // Cache trig results
  double cosLat = cos(this->dataPtr->latitudeReference.Radian());
  double sinLat = sin(this->dataPtr->latitudeReference.Radian());
  double cosLon = cos(this->dataPtr->longitudeReference.Radian());
  double sinLon = sin(this->dataPtr->longitudeReference.Radian());

  // Create a rotation matrix that moves ECEF to GLOBAL
  // http://www.navipedia.net/index.php/
  // Transformations_between_ECEF_and_ENU_coordinates
  this->dataPtr->rotECEFToGlobal = ignition::math::Matrix3d(
                      -sinLon,           cosLon,          0.0,
                      -cosLon * sinLat, -sinLon * sinLat, cosLat,
                       cosLon * cosLat,  sinLon * cosLat, sinLat);

  // Create a rotation matrix that moves GLOBAL to ECEF
  // http://www.navipedia.net/index.php/
  // Transformations_between_ECEF_and_ENU_coordinates
  this->dataPtr->rotGlobalToECEF = ignition::math::Matrix3d(
                      -sinLon, -cosLon * sinLat, cosLon * cosLat,
                       cosLon, -sinLon * sinLat, sinLon * cosLat,
                       0,      cosLat,           sinLat);

  // Cache heading transforms -- note that we have to negate the heading in
  // order to preserve backward compatibility. ie. Gazebo has traditionally
  // expressed positive angle as a CLOCKWISE rotation that takes the GLOBAL
  // frame to the LOCAL frame. However, right hand coordinate systems require
  // this to be expressed as an ANTI-CLOCKWISE rotation. So, we negate it.
  this->dataPtr->cosHea = cos(-this->dataPtr->headingOffset.Radian());
  this->dataPtr->sinHea = sin(-this->dataPtr->headingOffset.Radian());

  // Cache the ECEF coordinate of the origin
  this->dataPtr->origin = ignition::math::Vector3d(
    this->dataPtr->latitudeReference.Radian(),
    this->dataPtr->longitudeReference.Radian(),
    this->dataPtr->elevationReference);
  this->dataPtr->origin =
    this->PositionTransform(this->dataPtr->origin, SPHERICAL, ECEF);
}

/////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::PositionTransform(
    const ignition::math::Vector3d &_pos,
    const CoordinateType &_in, const CoordinateType &_out) const
{
  ignition::math::Vector3d tmp = _pos;

  // Cache trig results
  double cosLat = cos(_pos.X());
  double sinLat = sin(_pos.X());
  double cosLon = cos(_pos.Y());
  double sinLon = sin(_pos.Y());

  // Radius of planet curvature (meters)
  double curvature = 1.0 -
    this->dataPtr->ellE * this->dataPtr->ellE * sinLat * sinLat;
  curvature = this->dataPtr->ellA / sqrt(curvature);

  // Convert whatever arrives to a more flexible ECEF coordinate
  switch (_in)
  {
    // East, North, Up (ENU), note no break at end of case
    case LOCAL:
      {
        tmp.X(-_pos.X() * this->dataPtr->cosHea + _pos.Y() *
            this->dataPtr->sinHea);
        tmp.Y(-_pos.X() * this->dataPtr->sinHea - _pos.Y() *
            this->dataPtr->cosHea);
      }

    case GLOBAL:
      {
        tmp = this->dataPtr->origin + this->dataPtr->rotGlobalToECEF * tmp;
        break;
      }

    case SPHERICAL:
      {
        tmp.X((_pos.Z() + curvature) * cosLat * cosLon);
        tmp.Y((_pos.Z() + curvature) * cosLat * sinLon);
        tmp.Z(((this->dataPtr->ellB * this->dataPtr->ellB)/
              (this->dataPtr->ellA * this->dataPtr->ellA) *
              curvature + _pos.Z()) * sinLat);
        break;
      }

    // Do nothing
    case ECEF:
      break;
    default:
      {
        std::cerr << "Invalid coordinate type[" << _in << "]\n";
        return _pos;
      }
  }

  // Convert ECEF to the requested output coordinate system
  switch (_out)
  {
    case SPHERICAL:
      {
        // Convert from ECEF to SPHERICAL
        double p = sqrt(tmp.X() * tmp.X() + tmp.Y() * tmp.Y());
        double theta = atan((tmp.Z() * this->dataPtr->ellA) /
            (p * this->dataPtr->ellB));

        // Calculate latitude and longitude
        double lat = atan(
            (tmp.Z() + std::pow(this->dataPtr->ellP, 2) * this->dataPtr->ellB *
             std::pow(sin(theta), 3)) /
            (p - std::pow(this->dataPtr->ellE, 2) *
             this->dataPtr->ellA * std::pow(cos(theta), 3)));

        double lon = atan2(tmp.Y(), tmp.X());

        // Recalculate radius of planet curvature at the current latitude.
        double nCurvature = 1.0 - std::pow(this->dataPtr->ellE, 2) *
          std::pow(sin(lat), 2);
        nCurvature = this->dataPtr->ellA / sqrt(nCurvature);

        tmp.X(lat);
        tmp.Y(lon);

        // Now calculate Z
        tmp.Z(p/cos(lat) - nCurvature);
        break;
      }

    // Convert from ECEF TO GLOBAL
    case GLOBAL:
      tmp = this->dataPtr->rotECEFToGlobal * (tmp - this->dataPtr->origin);
      break;

    // Convert from ECEF TO LOCAL
    case LOCAL:
      tmp = this->dataPtr->rotECEFToGlobal * (tmp - this->dataPtr->origin);

      tmp = ignition::math::Vector3d(
          tmp.X() * this->dataPtr->cosHea - tmp.Y() * this->dataPtr->sinHea,
          tmp.X() * this->dataPtr->sinHea + tmp.Y() * this->dataPtr->cosHea,
          tmp.Z());
      break;

    // Return ECEF (do nothing)
    case ECEF:
      break;

    default:
      std::cerr << "Unknown coordinate type[" << _out << "]\n";
      return _pos;
  }

  return tmp;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::VelocityTransform(
    const ignition::math::Vector3d &_vel,
    const CoordinateType &_in, const CoordinateType &_out) const
{
  // Sanity check -- velocity should not be expressed in spherical coordinates
  if (_in == SPHERICAL || _out == SPHERICAL)
  {
    return _vel;
  }

  // Intermediate data type
  ignition::math::Vector3d tmp = _vel;

  // First, convert to an ECEF vector
  switch (_in)
  {
    // ENU (note no break at end of case)
    case LOCAL:
      tmp.X(-_vel.X() * this->dataPtr->cosHea + _vel.Y() *
            this->dataPtr->sinHea);
      tmp.Y(-_vel.X() * this->dataPtr->sinHea - _vel.Y() *
            this->dataPtr->cosHea);
    // spherical
    case GLOBAL:
      tmp = this->dataPtr->rotGlobalToECEF * tmp;
      break;
    // Do nothing
    case ECEF:
      tmp = _vel;
      break;
    default:
      std::cerr << "Unknown coordinate type[" << _in << "]\n";
      return _vel;
  }

  // Then, convert to the request coordinate type
  switch (_out)
  {
    // ECEF, do nothing
    case ECEF:
      break;

    // Convert from ECEF to global
    case GLOBAL:
      tmp = this->dataPtr->rotECEFToGlobal * tmp;
      break;

    // Convert from ECEF to local
    case LOCAL:
      tmp = this->dataPtr->rotECEFToGlobal * tmp;
      tmp = ignition::math::Vector3d(
          tmp.X() * this->dataPtr->cosHea - tmp.Y() * this->dataPtr->sinHea,
          tmp.X() * this->dataPtr->sinHea + tmp.Y() * this->dataPtr->cosHea,
          tmp.Z());
      break;

    default:
      std::cerr << "Unknown coordinate type[" << _out << "]\n";
      return _vel;
  }

  return tmp;
}

//////////////////////////////////////////////////
bool SphericalCoordinates::operator==(const SphericalCoordinates &_sc) const
{
  return this->Surface() == _sc.Surface() &&
         this->LatitudeReference() == _sc.LatitudeReference() &&
         this->LongitudeReference() == _sc.LongitudeReference() &&
         this->ElevationReference() == _sc.ElevationReference() &&
         this->HeadingOffset() == _sc.HeadingOffset();
}

//////////////////////////////////////////////////
bool SphericalCoordinates::operator!=(const SphericalCoordinates &_sc) const
{
  return !(*this == _sc);
}

//////////////////////////////////////////////////
SphericalCoordinates &SphericalCoordinates::operator=(
  const SphericalCoordinates &_sc)
{
  this->SetSurface(_sc.Surface());
  this->SetLatitudeReference(_sc.LatitudeReference());
  this->SetLongitudeReference(_sc.LongitudeReference());
  this->SetElevationReference(_sc.ElevationReference());
  this->SetHeadingOffset(_sc.HeadingOffset());

  // Generate transformation matrix
  this->UpdateTransformationMatrix();

  return *this;
}
