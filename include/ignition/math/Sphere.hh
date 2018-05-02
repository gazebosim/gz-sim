/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_SPHERE_HH_
#define IGNITION_MATH_SPHERE_HH_

#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Material.hh"
#include "ignition/math/Quaternion.hh"

namespace ignition
{
  namespace math
  {
    // Foward declarations
    class SpherePrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Sphere Sphere.hh ignition/math/Sphere.hh
    /// \brief A representation of a sphere.
    ///
    /// The sphere class supports defining a sphere with a radius and
    /// material properties. Radius is in meters.
    /// See Material for more on material properties.
    template<typename Precision>
    class Sphere
    {
      /// \brief Default constructor. The default radius is zero.
      public: Sphere() = default;

      /// \brief Construct a sphere with a radius.
      /// \param[in] _radius Radius of the sphere.
      public: explicit Sphere(const Precision _radius);

      /// \brief Construct a sphere with a radius, material
      /// \param[in] _radius Radius of the sphere.
      /// \param[in] _mat Material property for the sphere.
      public: Sphere(const Precision _radius, const Material &_mat);

      /// \brief Destructor
      public: ~Sphere() = default;

      /// \brief Get the radius in meters.
      /// \return The radius of the sphere in meters.
      public: Precision Radius() const;

      /// \brief Set the radius in meters.
      /// \param[in] _radius The radius of the sphere in meters.
      public: void SetRadius(const Precision _radius);

      /// \brief Get the material associated with this sphere.
      /// \return The material assigned to this sphere
      public: const ignition::math::Material &Material() const;

      /// \brief Set the material associated with this sphere.
      /// \param[in] _mat The material assigned to this sphere
      public: void SetMaterial(const ignition::math::Material &_mat);

      /// \brief Get the mass matrix for this sphere. This function
      /// is only meaningful if the sphere's radius and material have been set.
      /// \param[out] _massMatrix The computed mass matrix will be stored
      /// here.
      /// \return False if computation of the mass matrix failed, which
      /// could be due to an invalid radius (<=0) or density (<=0).
      public: bool MassMatrix(MassMatrix3d &_massMat) const;

      /// \brief Check if this sphere is equal to the provided sphere.
      /// Radius and material properties will be checked.
      public: bool operator==(const Sphere &_sphere) const;

      /// \brief Check if this sphere is not equal to the provided sphere.
      /// Radius and material properties will be checked.
      public: bool operator!=(const Sphere &_sphere) const;

      /// \brief Get the volume of the sphere in m^3.
      /// \return Volume of the sphere in m^3.
      public: Precision Volume() const;

      /// \brief Compute the sphere's density given a mass value. The
      /// sphere is assumed to be solid with uniform density. This
      /// function requires the sphere's radius to be set to a
      /// value greater than zero. The Material of the sphere is ignored.
      /// \param[in] _mass Mass of the sphere, in kg. This value should be
      /// greater than zero.
      /// \return Density of the sphere in kg/m^3. A negative value is
      /// returned if radius or _mass is <= 0.
      public: Precision DensityFromMass(const Precision _mass) const;

      /// \brief Set the density of this sphere based on a mass value.
      /// Density is computed using
      /// Precision DensityFromMass(const Precision _mass) const. The
      /// sphere is assumed to be solid with uniform density. This
      /// function requires the sphere's radius to be set to a
      /// value greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the sphere, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// sphere's radius or the _mass value are <= 0.
      /// \sa Precision DensityFromMass(const Precision _mass) const
      public: bool SetDensityFromMass(const Precision _mass);

      /// \brief Radius of the sphere.
      private: Precision radius = 0.0;

      /// \brief the sphere's material.
      private: ignition::math::Material material;
    };

    /// \typedef Sphere<int> Spherei
    /// \brief Sphere with integer precision.
    typedef Sphere<int> Spherei;

    /// \typedef Sphere<double> Sphered
    /// \brief Sphere with double precision.
    typedef Sphere<double> Sphered;

    /// \typedef Sphere<float> Spheref
    /// \brief Sphere with float precision.
    typedef Sphere<float> Spheref;
    }
  }
}
#include "ignition/math/detail/Sphere.hh"

#endif
