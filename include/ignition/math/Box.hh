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
#ifndef IGNITION_MATH_BOX_HH_
#define IGNITION_MATH_BOX_HH_

#include <ignition/math/config.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Box Box.hh ignition/math/Box.hh
    /// \brief A representation of a box. All units are in meters.
    ///
    /// The box class supports defining a size and material properties.
    /// See Material for more on material properties.
    ///
    /// By default, a box's size (length, width, and height)  is zero.
    ///
    /// See AxisAlignedBox for an axis aligned box implementation.
    template<typename Precision>
    class Box
    {
      /// \brief Default constructor.
      public: Box() = default;

      /// \brief Construct a box with specified dimensions.
      /// \param[in] _length Length of the box in meters.
      /// \param[in] _width Width of the box in meters.
      /// \param[in] _height Height of the box in meters.
      public: Box(const Precision _length,
                  const Precision _width,
                  const Precision _height);

      /// \brief Construct a box with specified dimensions and a material.
      /// \param[in] _length Length of the box in meters.
      /// \param[in] _width Width of the box in meters.
      /// \param[in] _height Height of the box.
      /// \param[in] _mat Material property for the box.
      public: Box(const Precision _length, const Precision _width,
                  const Precision _height,
                  const ignition::math::Material &_mat);

      /// \brief Construct a box with specified dimensions, in vector form.
      /// \param[in] _size Size of the box. The vector _size has the following
      /// mapping:
      ///
      /// * _size[0] == length in meters
      /// * _size[1] == width in meters
      /// * _size[2] == height in meters
      public: explicit Box(const Vector3<Precision> &_size);

      /// \brief Construct a box with specified dimensions, in vector form
      /// and a material.
      /// \param[in] _size Size of the box. The vector _size has the following
      /// mapping:
      ///
      /// * _size[0] == length in meters
      /// * _size[1] == width in meters
      /// * _size[2] == height in meters
      /// \param[in] _mat Material property for the box.
      public: Box(const Vector3<Precision> &_size,
                  const ignition::math::Material &_mat);

      /// \brief Copy Constructor.
      /// \param[in]  _b Box to copy.
      public: Box(const Box<Precision> &_b);

      /// \brief Destructor.
      public: virtual ~Box() = default;

      /// \brief Get the size of the box.
      /// \return Size of the box in meters.
      public: math::Vector3<Precision> Size() const;

      /// \brief Set the size of the box.
      /// \param[in] _size Size of the box. The vector _size has the following
      /// mapping:
      ///
      /// * _size[0] == lengt in metersh
      /// * _size[1] == widt in metersh
      /// * _size[2] == heigh in meterst
      public: void SetSize(const math::Vector3<Precision> &_size);

      /// \brief Set the size of the box.
      /// \param[in] _length Length of the box in meters.
      /// \param[in] _width Width of the box in meters.
      /// \param[in] _height Height of the box in meters.
      public: void SetSize(const Precision _length,
                           const Precision _width,
                           const Precision _height);

      /// \brief Equality test operator.
      /// \param[in] _b Box to test.
      /// \return True if equal.
      public: bool operator==(const Box<Precision> &_b) const;

      /// \brief Inequality test operator.
      /// \param[in] _b Box to test.
      /// \return True if not equal.
      public: bool operator!=(const Box<Precision> &_b) const;

      /// \brief Get the material associated with this box.
      /// \return The material assigned to this box.
      public: const ignition::math::Material &Material() const;

      /// \brief Set the material associated with this box.
      /// \param[in] _mat The material assigned to this box.
      public: void SetMaterial(const ignition::math::Material &_mat);

      /// \brief Get the volume of the box in m^3.
      /// \return Volume of the box in m^3.
      public: Precision Volume() const;

      /// \brief Compute the box's density given a mass value. The
      /// box is assumed to be solid with uniform density. This
      /// function requires the box's size to be set to
      /// values greater than zero. The Material of the box is ignored.
      /// \param[in] _mass Mass of the box, in kg. This value should be
      /// greater than zero.
      /// \return Density of the box in kg/m^3. A negative value is
      /// returned if the size or _mass is <= 0.
      public: Precision DensityFromMass(const Precision _mass) const;

      /// \brief Set the density of this box based on a mass value.
      /// Density is computed using
      /// double DensityFromMass(const double _mass) const. The
      /// box is assumed to be solid with uniform density. This
      /// function requires the box's size to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the box, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// box's size or the _mass value are <= 0.
      /// \sa double DensityFromMass(const double _mass) const
      public: bool SetDensityFromMass(const Precision _mass);

      /// \brief Get the mass matrix for this box. This function
      /// is only meaningful if the box's size and material
      /// have been set.
      /// \param[out] _massMatrix The computed mass matrix will be stored
      /// here.
      /// \return False if computation of the mass matrix failed, which
      /// could be due to an invalid size (<=0) or density (<=0).
      public: bool MassMatrix(MassMatrix3<Precision> &_massMat) const;

      /// \brief Size of the box.
      private: Vector3<Precision> size = Vector3<Precision>::Zero;

      /// \brief The box's material.
      private: ignition::math::Material material;
    };

    /// \typedef Box<int> Boxi
    /// \brief Box with integer precision.
    typedef Box<int> Boxi;

    /// \typedef Box<double> Boxd
    /// \brief Box with double precision.
    typedef Box<double> Boxd;

    /// \typedef Box<float> Boxf
    /// \brief Box with float precision.
    typedef Box<float> Boxf;
    }
  }
}
#include "ignition/math/detail/Box.hh"
#endif
