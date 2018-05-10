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
#ifndef IGNITION_MATH_MATERIAL_HH_
#define IGNITION_MATH_MATERIAL_HH_

#include <limits>
#include <map>
#include <string>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>
#include <ignition/math/MaterialType.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    // Forward declarations
    class MaterialPrivate;

    /// \brief Contains information about a single material.
    ///
    /// Steel, wood, and iron are examples of materials. This class
    /// allows you to create custom materials, or use built-in materials.
    /// The list of built-in materials can be found in the ::MaterialType
    /// enum.
    ///
    /// This class will replace the
    /// [MaterialDensity class](https://bitbucket.org/ignitionrobotics/ign-common/src/ign-common1/include/ignition/common/MaterialDensity.hh)
    /// found in the Ignition Common library, which was at version 1 at the
    /// time of this writing.
    ///
    /// **How to create a wood material:**
    ///
    /// ~~~
    /// Material mat(MaterialType::WOOD);
    /// std::cout << "The density of " << mat.Name() << " is "
    ///   << mat.Density() << std::endl;
    /// ~~~
    ///
    /// **How to create a custom material:**
    ///
    /// ~~~
    /// Material mat;
    /// mat.SetDensity(12.23);
    /// mat.SetName("my_material");
    /// std::cout << "The density of " << mat.Name() is "
    ///   << mat.Density() << std::endl;
    /// ~~~
    class IGNITION_MATH_VISIBLE Material
    {
      /// \brief Constructor.
      public: Material();

      /// \brief Construct a material based on a type.
      /// \param[in] _type Built-in type to create.
      public: explicit Material(const MaterialType _type);

      /// \brief Construct a material based on a type name.
      /// \param[in] _typename Name of the built-in type to create. String
      /// names are listed in the ::MaterialType documentation.
      public: explicit Material(const std::string &_typename);

      /// \brief Construct a material based on a density value.
      /// \param[in] _density Material density.
      public: explicit Material(const double _density);

      /// \brief Copy constructor.
      /// \param[in] _material Material to copy.
      public: Material(const Material &_material);

      /// \brief Move constructor.
      /// \param[in] _material Material to move. The _material object will
      /// have default values after the move.
      public: Material(Material &&_material);

      /// \brief Destructor.
      public: ~Material();

      /// \brief Get all the built-in materials.
      /// \return A map of all the materials. The map's key is
      /// material type and the map's value is the material object.
      public: static const std::map<MaterialType, Material> &Predefined();

      /// \brief Set this Material to the built-in Material with
      /// the nearest density value within _epsilon. If a built-in material
      /// could not be found, then this Material is not changed.
      /// \param[in] _value Density value of entry to match.
      /// \param[in] _epsilon Allowable range of difference between _value,
      /// and a material's density.
      public: void SetToNearestDensity(
                  const double _value,
                  const double _epsilon = std::numeric_limits<double>::max());

      /// \brief Copy operator.
      /// \param[in] _material Material to copy.
      /// \return Reference to this material.
      public: Material &operator=(const Material &_material);

      /// \brief Move operator.
      /// \param[in] _material Material to move. The _material object will
      /// contain default values after this move.
      /// \return Reference to this Material.
      public: Material &operator=(Material &&_material);

      /// \brief Equality operator. This compares type and density values.
      /// \param[in] _material Material to evaluate this object against.
      /// \return True if this material is equal to the given _material.
      public: bool operator==(const Material &_material) const;

      /// \brief Inequality operator. This compares type and density values.
      /// \param[in] _material Material to evaluate this object against.
      /// \return True if this material is not equal to the given _material.
      public: bool operator!=(const Material &_material) const;

      /// \brief Get the material's type.
      /// \return The material's type.
      public: MaterialType Type() const;

      /// \brief Set the material's type. This will only set the type value.
      /// Other properties, such as density, will not be changed.
      /// \param[in] _type The material's type.
      public: void SetType(const MaterialType _type);

      /// \brief Get the name of the material. This will match the enum type
      /// names used in ::MaterialType, but in lowercase, if a built-in
      /// material is used.
      /// \return The material's name.
      /// \sa void SetName(const std::string &_name)
      public: std::string Name() const;

      /// \brief Set the name of the material.
      /// \param[in] _name The material's name.
      /// \sa std::string Name() const
      public: void SetName(const std::string &_name);

      /// \brief Get the density value of the material in kg/m^3.
      /// \return The density of this material in kg/m^3.
      public: double Density() const;

      /// \brief Set the density value of the material in kg/m^3.
      /// \param[in] _density The density of this material in kg/m^3.
      public: void SetDensity(const double _density);

      /// \brief Private data pointer.
      private: MaterialPrivate *dataPtr = nullptr;
    };
    }
  }
}
#endif
