/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module sphericalcoordinates
%{
#include <memory>
#include <string>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>
%}

namespace gz
{
  namespace math
  {
    class SphericalCoordinates
    {
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

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

      public: SphericalCoordinates();

      public: explicit SphericalCoordinates(const SurfaceType _type);

      public: SphericalCoordinates(const SurfaceType _type,
                                   const gz::math::Angle &_latitude,
                                   const gz::math::Angle &_longitude,
                                   const double _elevation,
                                   const gz::math::Angle &_heading);

      public: SphericalCoordinates(const SphericalCoordinates &_sc);

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      public: gz::math::Vector3<double> SphericalFromLocalPosition(
                  const gz::math::Vector3<double> &_xyz) const;

      public: gz::math::Vector3<double> GlobalFromLocalVelocity(
                  const gz::math::Vector3<double> &_xyz) const;

      public: static SurfaceType Convert(const std::string &_str);

      public: static std::string Convert(SurfaceType _type);

      public: static double Distance(const gz::math::Angle &_latA,
                                     const gz::math::Angle &_lonA,
                                     const gz::math::Angle &_latB,
                                     const gz::math::Angle &_lonB);

      public: SurfaceType Surface() const;

      public: gz::math::Angle LatitudeReference() const;

      public: gz::math::Angle LongitudeReference() const;

      public: double ElevationReference() const;

      public: gz::math::Angle HeadingOffset() const;

      public: void SetSurface(const SurfaceType &_type);

      public: void SetLatitudeReference(const gz::math::Angle &_angle);

      public: void SetLongitudeReference(const gz::math::Angle &_angle);

      public: void SetElevationReference(const double _elevation);

      public: void SetHeadingOffset(const gz::math::Angle &_angle);

      public: gz::math::Vector3<double> LocalFromSphericalPosition(
                  const gz::math::Vector3<double> &_latLonEle) const;

      public: gz::math::Vector3<double> LocalFromGlobalVelocity(
                  const gz::math::Vector3<double> &_xyz) const;

      public: void UpdateTransformationMatrix();

      public: gz::math::Vector3<double>
              PositionTransform(const gz::math::Vector3<double> &_pos,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \return Transformed velocity vector
      public: gz::math::Vector3<double> VelocityTransform(
                  const gz::math::Vector3<double> &_vel,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      public: bool operator==(const SphericalCoordinates &_sc) const;

      public: bool operator!=(const SphericalCoordinates &_sc) const;
    };
  }
}
