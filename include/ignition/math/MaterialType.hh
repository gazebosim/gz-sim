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
#ifndef IGNITION_MATH_MATERIALTYPES_HH_
#define IGNITION_MATH_MATERIALTYPES_HH_

#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \enum MaterialType
    /// \brief This enum lists the supported material types. A value can be
    /// used to create a Material instance.
    /// Source: https://en.wikipedia.org/wiki/Density
    /// \sa Material
    // Developer Note: When modifying this enum, make sure to also modify
    // the kMaterials map in src/MaterialTypes.hh.
    enum class MaterialType
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
      UNKNOWN_MATERIAL
    };
    }
  }
}
#endif
