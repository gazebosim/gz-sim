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
#ifndef GZ_MATH_CAPSULE_HH_
#define GZ_MATH_CAPSULE_HH_

#include <optional>
#include "gz/math/MassMatrix3.hh"
#include "gz/math/Material.hh"

namespace gz
{
  namespace math
  {
    // Foward declarations
    class CapsulePrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Capsule Capsule.hh gz/math/Capsule.hh
    /// \brief A representation of a capsule or sphere-capped cylinder.
    ///
    /// The capsule class supports defining a capsule with a radius,
    /// length, and material properties. The shape is equivalent to a cylinder
    /// aligned with the Z-axis and capped with hemispheres. Radius and
    /// length are in meters. See Material for more on material properties.
    /// \tparam Precision Scalar numeric type.
    template<typename Precision>
    class Capsule
    {
      /// \brief Default constructor. The default radius and length are both
      /// zero.
      public: Capsule() = default;

      /// \brief Construct a capsule with a length and radius.
      /// \param[in] _length Length of the capsule.
      /// \param[in] _radius Radius of the capsule.
      public: Capsule(const Precision _length, const Precision _radius);

      /// \brief Construct a capsule with a length, radius, and material.
      /// \param[in] _length Length of the capsule.
      /// \param[in] _radius Radius of the capsule.
      /// \param[in] _mat Material property for the capsule.
      public: Capsule(const Precision _length, const Precision _radius,
                  const Material &_mat);

      /// \brief Get the radius in meters.
      /// \return The radius of the capsule in meters.
      public: Precision Radius() const;

      /// \brief Set the radius in meters.
      /// \param[in] _radius The radius of the capsule in meters.
      public: void SetRadius(const Precision _radius);

      /// \brief Get the length in meters.
      /// \return The length of the capsule in meters.
      public: Precision Length() const;

      /// \brief Set the length in meters.
      /// \param[in] _length The length of the capsule in meters.
      public: void SetLength(const Precision _length);

      /// \brief Get the material associated with this capsule.
      /// \return The material assigned to this capsule
      public: const Material &Mat() const;

      /// \brief Set the material associated with this capsule.
      /// \param[in] _mat The material assigned to this capsule
      public: void SetMat(const Material &_mat);

      /// \brief Get the mass matrix for this capsule. This function
      /// is only meaningful if the capsule's radius, length, and material
      /// have been set.
      /// \return The computed mass matrix if parameters are valid
      /// (radius > 0), (length > 0), and (density > 0). Otherwise
      /// std::nullopt is returned.
      public: std::optional< MassMatrix3<Precision> > MassMatrix() const;

      /// \brief Check if this capsule is equal to the provided capsule.
      /// Radius, length, and material properties will be checked.
      public: bool operator==(const Capsule &_capsule) const;

      /// \brief Get the volume of the capsule in m^3.
      /// \return Volume of the capsule in m^3.
      public: Precision Volume() const;

      /// \brief Compute the capsule's density given a mass value. The
      /// capsule is assumed to be solid with uniform density. This
      /// function requires the capsule's radius and length to be set to
      /// values greater than zero. The Material of the capsule is ignored.
      /// \param[in] _mass Mass of the capsule, in kg. This value should be
      /// greater than zero.
      /// \return Density of the capsule in kg/m^3. A NaN is returned
      /// if radius, length or _mass is <= 0.
      public: Precision DensityFromMass(const Precision _mass) const;

      /// \brief Set the density of this capsule based on a mass value.
      /// Density is computed using
      /// Precision DensityFromMass(const Precision _mass) const. The
      /// capsule is assumed to be solid with uniform density. This
      /// function requires the capsule's radius and length to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the capsule, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// capsule's radius, length, or the _mass value are <= 0.
      /// \sa Precision DensityFromMass(const Precision _mass) const
      public: bool SetDensityFromMass(const Precision _mass);

      /// \brief Radius of the capsule.
      private: Precision radius = 0.0;

      /// \brief Length of the capsule.
      private: Precision length = 0.0;

      /// \brief the capsule's material.
      private: Material material;
    };

    /// \typedef Capsule<int> Capsulei
    /// \brief Capsule with integer precision.
    typedef Capsule<int> Capsulei;

    /// \typedef Capsule<double> Capsuled
    /// \brief Capsule with double precision.
    typedef Capsule<double> Capsuled;

    /// \typedef Capsule<float> Capsulef
    /// \brief Capsule with float precision.
    typedef Capsule<float> Capsulef;
    }
  }
}
#include "gz/math/detail/Capsule.hh"

#endif
