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
#ifndef IGNITION_MATH_SPHERICALCOORDINATES_HH_
#define IGNITION_MATH_SPHERICALCOORDINATES_HH_

#include <memory>
#include <string>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    class SphericalCoordinatesPrivate;

    /// \brief Convert spherical coordinates for planetary surfaces.
    class IGNITION_MATH_VISIBLE SphericalCoordinates
    {
      /// \enum SurfaceType
      /// \brief Unique identifiers for planetary surface models.
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

      /// \enum CoordinateType
      /// \brief Unique identifiers for coordinate types.
      public: enum CoordinateType
              {
                /// \brief Latitude, Longitude and Altitude by SurfaceType
                SPHERICAL = 1,

                /// \brief Earth centered, earth fixed Cartesian
                ECEF = 2,

                /// \brief Local tangent plane (East, North, Up)
                GLOBAL = 3,

                /// \brief Heading-adjusted tangent plane (X, Y, Z)
                /// This has kept a bug for backwards compatibility, use
                /// LOCAL2 for the correct behaviour.
                LOCAL = 4,

                /// \brief Heading-adjusted tangent plane (X, Y, Z)
                LOCAL2 = 5
              };

      /// \brief Constructor.
      public: SphericalCoordinates();

      /// \brief Constructor with surface type input.
      /// \param[in] _type SurfaceType specification.
      public: explicit SphericalCoordinates(const SurfaceType _type);

      /// \brief Constructor with surface type, angle, and elevation inputs.
      /// \param[in] _type SurfaceType specification.
      /// \param[in] _latitude Reference latitude.
      /// \param[in] _longitude Reference longitude.
      /// \param[in] _elevation Reference elevation.
      /// \param[in] _heading Heading offset.
      public: SphericalCoordinates(const SurfaceType _type,
                                   const ignition::math::Angle &_latitude,
                                   const ignition::math::Angle &_longitude,
                                   const double _elevation,
                                   const ignition::math::Angle &_heading);

      /// \brief Copy constructor.
      /// \param[in] _sc Spherical coordinates to copy.
      public: SphericalCoordinates(const SphericalCoordinates &_sc);

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// This performs a `PositionTransform` from LOCAL to SPHERICAL.
      ///
      /// There's a known bug with this computation that can't be fixed on
      /// version 6 to avoid behaviour changes. Directly call
      /// `PositionTransform(_xyz, LOCAL2, SPHERICAL)` for correct results.
      /// Note that `PositionTransform` returns spherical coordinates in
      /// radians.
      ///
      /// \param[in] _xyz Cartesian position vector in the heading-adjusted
      /// world frame.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      public: ignition::math::Vector3d SphericalFromLocalPosition(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a Cartesian velocity vector in the local frame
      ///        to a global Cartesian frame with components East, North, Up.
      /// This is a wrapper around `VelocityTransform(_xyz, LOCAL, GLOBAL)`
      ///
      /// There's a known bug with this computation that can't be fixed on
      /// version 6 to avoid behaviour changes. Directly call
      /// `VelocityTransform(_xyz, LOCAL2, GLOBAL)` for correct results.
      ///
      /// \param[in] _xyz Cartesian velocity vector in the heading-adjusted
      /// world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      public: ignition::math::Vector3d GlobalFromLocalVelocity(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a string to a SurfaceType.
      /// Allowed values: ["EARTH_WGS84"].
      /// \param[in] _str String to convert.
      /// \return Conversion to SurfaceType.
      public: static SurfaceType Convert(const std::string &_str);

      /// \brief Convert a SurfaceType to a string.
      /// \param[in] _type Surface type
      /// \return Type as string
      public: static std::string Convert(SurfaceType _type);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _lonA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _lonB Longitude of point B.
      /// \return Distance in meters.
      public: static double Distance(const ignition::math::Angle &_latA,
                                     const ignition::math::Angle &_lonA,
                                     const ignition::math::Angle &_latB,
                                     const ignition::math::Angle &_lonB);

      /// \brief Get SurfaceType currently in use.
      /// \return Current SurfaceType value.
      public: SurfaceType Surface() const;

      /// \brief Get reference geodetic latitude.
      /// \return Reference geodetic latitude.
      public: ignition::math::Angle LatitudeReference() const;

      /// \brief Get reference longitude.
      /// \return Reference longitude.
      public: ignition::math::Angle LongitudeReference() const;

      /// \brief Get reference elevation in meters.
      /// \return Reference elevation.
      public: double ElevationReference() const;

      /// \brief Get heading offset for the reference frame, expressed as
      ///        angle from East to x-axis, or equivalently
      ///        from North to y-axis.
      /// \return Heading offset of reference frame.
      public: ignition::math::Angle HeadingOffset() const;

      /// \brief Set SurfaceType for planetary surface model.
      /// \param[in] _type SurfaceType value.
      public: void SetSurface(const SurfaceType &_type);

      /// \brief Set reference geodetic latitude.
      /// \param[in] _angle Reference geodetic latitude.
      public: void SetLatitudeReference(const ignition::math::Angle &_angle);

      /// \brief Set reference longitude.
      /// \param[in] _angle Reference longitude.
      public: void SetLongitudeReference(const ignition::math::Angle &_angle);

      /// \brief Set reference elevation above sea level in meters.
      /// \param[in] _elevation Reference elevation.
      public: void SetElevationReference(const double _elevation);

      /// \brief Set heading angle offset for the frame.
      /// \param[in] _angle Heading offset for the frame.
      public: void SetHeadingOffset(const ignition::math::Angle &_angle);

      /// \brief Convert a geodetic position vector to Cartesian coordinates.
      /// This performs a `PositionTransform` from SPHERICAL to LOCAL.
      /// \param[in] _latLonEle Geodetic position in the planetary frame of
      /// reference. X: latitude (deg), Y: longitude (deg), X: altitude.
      /// \return Cartesian position vector in the heading-adjusted world frame.
      public: ignition::math::Vector3d LocalFromSphericalPosition(
                  const ignition::math::Vector3d &_latLonEle) const;

      /// \brief Convert a Cartesian velocity vector with components East,
      /// North, Up to a local cartesian frame vector XYZ.
      /// This is a wrapper around `VelocityTransform(_xyz, GLOBAL, LOCAL)`
      /// \param[in] _xyz Vector with components (x,y,z): (East, North, Up).
      /// \return Cartesian vector in the world frame.
      public: ignition::math::Vector3d LocalFromGlobalVelocity(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Update coordinate transformation matrix with reference location
      public: void UpdateTransformationMatrix();

      /// \brief Convert between positions in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// Spherical coordinates use radians, while the other frames use meters.
      /// \param[in] _pos Position vector in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed coordinate using cached origin.
      public: ignition::math::Vector3d
              PositionTransform(const ignition::math::Vector3d &_pos,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \brief Convert between velocity in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// Spherical coordinates use radians, while the other frames use meters.
      /// \param[in] _vel Velocity vector in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed velocity vector
      public: ignition::math::Vector3d VelocityTransform(
                  const ignition::math::Vector3d &_vel,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \brief Equality operator, result = this == _sc
      /// \param[in] _sc Spherical coordinates to check for equality
      /// \return true if this == _sc
      public: bool operator==(const SphericalCoordinates &_sc) const;

      /// \brief Inequality
      /// \param[in] _sc Spherical coordinates to check for inequality
      /// \return true if this != _sc
      public: bool operator!=(const SphericalCoordinates &_sc) const;

      /// \brief Assignment operator
      /// \param[in] _sc The spherical coordinates to copy from.
      /// \return this
      public: SphericalCoordinates &operator=(
        const SphericalCoordinates &_sc);


#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \internal
      /// \brief Pointer to the private data
      private: std::unique_ptr<SphericalCoordinatesPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}
#endif
