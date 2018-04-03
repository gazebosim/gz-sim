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
#ifndef IGNITION_MATH_MATERIALDENSITY_HH_
#define IGNITION_MATH_MATERIALDENSITY_HH_

#include <string>
#include <limits>
#include <map>
#include "ignition/math/Export.hh"
#include "ignition/math/config.hh"

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Remove this diagnostic push,ignore,pop after gcc 6. The issue is with
    // the visibility attributes in enum classes.
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43407
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#endif
    /// \enum MaterialType
    /// \brief This enum lists the supported material types. A value can be
    /// used to query the MaterialDensity class for a density value.
    /// Source: https://en.wikipedia.org/wiki/Density
    /// \sa MaterialDensity
    // Developer Note: When modifying this enum, make sure to also modify
    // the MaterialDensity::materials map, see below.
    enum class IGNITION_MATH_VISIBLE MaterialType
    {
      /// \brief Styrofoam, density = 75.0 kg/m^3
      /// String name = "styrofoam"
      STYROFOAM = 0,

      /// \brief Pine, density = 373.0 kg/m^3
      /// String name = "pine"
      PINE,

      /// \brief Wood, density = 700.0 kg/m^3
      /// String name = "wood"
      WOOD,

      /// \brief Oak, density = 710.0 kg/m^3
      /// String name = "oak"
      OAK,

      /// \brief Ice, density = 916.0 kg/m^3
      /// String name = "ice"
      ICE,

      /// \brief Water, density = 1000.0 kg/m^3
      /// String name = "water"
      WATER,

      /// \brief Plastic, density = 1175.0 kg/m^3
      /// String name = "plastic"
      PLASTIC,

      /// \brief Concrete, density = 2000.0 kg/m^3
      /// String name = "concrete"
      CONCRETE,

      /// \brief Aluminum, density = 2700.0 kg/m^3
      /// String name = "aluminum"
      ALUMINUM,

      /// \brief Steel alloy, density = 7600.0 kg/m^3
      /// String name = "steel_alloy"
      STEEL_ALLOY,

      /// \brief Stainless steel, density = 7800.0 kg/m^3
      /// String name = "steel_stainless"
      STEEL_STAINLESS,

      /// \brief Iron, density = 7870.0 kg/m^3
      /// String name = "iron"
      IRON,

      /// \brief Brass, density = 8600.0 kg/m^3
      /// String name = "brass"
      BRASS,

      /// \brief Copper, density = 8940.0 kg/m^3
      /// String name = "copper"
      COPPER,

      /// \brief Tungsten, density = 19300.0 kg/m^3
      /// String name = "tungsten"
      TUNGSTEN,

      /// \brief Represents an invalid or unknown material.
      // This value should always be last in the enum; it is used in
      // MaterialDensity_TEST.
      INVALID
    };
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

    /// \brief Contains information about a single material.
    struct IGNITION_MATH_VISIBLE Material
    {
      /// \brief The material type.
      public: MaterialType type = MaterialType::INVALID;

      /// \brief Name of the material. This will match the names
      /// used in MaterialType, but in lowercase.
      public: std::string name = "";

      /// \brief Density value of the material in kg/m^3.
      public: double density = -1;
    };

    /// \brief Encapsulates density types.
    class IGNITION_MATH_VISIBLE MaterialDensity
    {
      /// \brief Accessor for retrieving density entries
      /// \return List of entries.
      public: static const std::map<MaterialType, Material> &Materials();

      /// \brief Return the density of the given material name, or
      /// a negative value if the material is not found. Units are kg/m^3
      /// \param[in] _material Name of the material. The name must be a
      /// lowercase version of one value in the MaterialType enum.
      /// \return Matching density if found, otherwise a negative value.
      /// \sa MaterialType.
      public: static double Density(const std::string &_material);

      /// \brief Return the density of a material based on a type. Units are
      /// kg/m^3
      /// \param[in] _material Type of the material.
      /// \return Matching density if found, otherwise a negative value.
      /// \sa MaterialType
      public: static double Density(const MaterialType _material);

      /// \brief Return the material with the closest density value within
      /// _epsilon, or MATERIAL_TYPE_END if not found.
      /// \param[in] _value Density value of entry to match.
      /// \param[in] _epsilon Allowable range of difference between _value,
      /// and a material's density.
      /// \return The Material that has a density nearest to the given value.
      /// A default constructed Material will be returned if a matching
      /// material could not be found for the given value and epsilon.
      public: static Material Nearest(
                  const double _value,
                  const double _epsilon = std::numeric_limits<double>::max());

#ifdef _WIN32
// Disable warning C4251
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      // Developer Note: When modifying this map, make sure to also modify
      // the MaterialType enum, see above.
      private: static std::map<MaterialType, Material> materials;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}
#endif
