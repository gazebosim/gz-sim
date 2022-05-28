/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_MATH_ELLIPSOID_HH_
#define GZ_MATH_ELLIPSOID_HH_

#include <optional>
#include "gz/math/MassMatrix3.hh"
#include "gz/math/Material.hh"

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Ellipsoid Ellipsoid.hh gz/math/Ellipsoid.hh
    /// \brief A representation of a general ellipsoid.
    ///
    /// The ellipsoid class supports defining a ellipsoid with three radii and
    /// material properties. Radii are in meters. See Material for more on
    /// material properties.
    /// \tparam Precision Scalar numeric type.
    template<typename Precision>
    class Ellipsoid
    {
      /// \brief Default constructor. The default radius and length are both
      /// zero.
      public: Ellipsoid() = default;

      /// \brief Construct a ellipsoid with a Vector3 of three radii.
      /// \param[in] _radii The three radii (x, y, z) defining this ellipsoid
      public: explicit Ellipsoid(const Vector3<Precision> &_radii);

      /// \brief Construct a ellipsoid with three radii and a material.
      /// \param[in] _radii The three radii (x, y, z) defining this ellipsoid
      /// \param[in] _mat Material property for the ellipsoid.
      public: Ellipsoid(const Vector3<Precision> &_radii,
                  const Material &_mat);

      /// \brief Get the radius in meters.
      /// \return The radius of the ellipsoid in meters.
      public: Vector3<Precision> Radii() const;

      /// \brief Set the radius in meters.
      /// \param[in] _radii The radii of the ellipsoid in meters.
      public: void SetRadii(const Vector3<Precision> &_radii);

      /// \brief Get the material associated with this ellipsoid.
      /// \return The material assigned to this ellipsoid
      public: const Material &Mat() const;

      /// \brief Set the material associated with this ellipsoid.
      /// \param[in] _mat The material assigned to this ellipsoid
      public: void SetMat(const Material &_mat);

      /// \brief Get the mass matrix for this ellipsoid. This function
      /// is only meaningful if the ellipsoid's radii and material
      /// have been set.
      /// \return The computed mass matrix if parameters are valid
      /// (radius > 0), (length > 0), and (density > 0). Otherwise
      /// std::nullopt is returned.
      public: std::optional< MassMatrix3<Precision> > MassMatrix() const;

      /// \brief Check if this ellipsoid is equal to the provided ellipsoid.
      /// Radius, length, and material properties will be checked.
      public: bool operator==(const Ellipsoid &_ellipsoid) const;

      /// \brief Get the volume of the ellipsoid in m^3.
      /// \return Volume of the ellipsoid in m^3.
      public: Precision Volume() const;

      /// \brief Compute the ellipsoid's density given a mass value. The
      /// ellipsoid is assumed to be solid with uniform density. This
      /// function requires the ellipsoid's radius and length to be set to
      /// values greater than zero. The Material of the ellipsoid is ignored.
      /// \param[in] _mass Mass of the ellipsoid, in kg. This value should be
      /// greater than zero.
      /// \return Density of the ellipsoid in kg/m^3. A NaN is returned
      /// if radius, length or _mass is <= 0.
      public: Precision DensityFromMass(const Precision _mass) const;

      /// \brief Set the density of this ellipsoid based on a mass value.
      /// Density is computed using
      /// Precision DensityFromMass(const Precision _mass) const. The
      /// ellipsoid is assumed to be solid with uniform density. This
      /// function requires the ellipsoid's radius and length to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the ellipsoid, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// ellipsoid's radius, length, or the _mass value are <= 0.
      /// \sa Precision DensityFromMass(const Precision _mass) const
      public: bool SetDensityFromMass(const Precision _mass);

      /// \brief Radius of the ellipsoid.
      private: Vector3<Precision> radii = Vector3<Precision>::Zero;

      /// \brief the ellipsoid's material.
      private: Material material;
    };

    /// \typedef Ellipsoid<int> Ellipsoidi
    /// \brief Ellipsoid with integer precision.
    typedef Ellipsoid<int> Ellipsoidi;

    /// \typedef Ellipsoid<double> Ellipsoidd
    /// \brief Ellipsoid with double precision.
    typedef Ellipsoid<double> Ellipsoidd;

    /// \typedef Ellipsoid<float> Ellipsoidf
    /// \brief Ellipsoid with float precision.
    typedef Ellipsoid<float> Ellipsoidf;
    }
  }
}
#include "gz/math/detail/Ellipsoid.hh"

#endif
