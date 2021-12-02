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
#ifndef IGNITION_MATH_SRC_MATERIAL_TYPE_HH_
#define IGNITION_MATH_SRC_MATERIAL_TYPE_HH_

#include <array>
#include <utility>

using namespace ignition;
using namespace math;

// This class is used to curly-brace initialize kMaterialData
struct MaterialData
{
  // Name of the material
  // cppcheck-suppress unusedStructMember
  const char * const name;

  // Density of the material
  // cppcheck-suppress unusedStructMember
  const double density;
};

// The mapping of material type to name and density values.
// If you modify this map, make sure to also modify the MaterialType enum in
// include/ignition/math/MaterialTypes.hh. The compiler will also complain if
// you forget to change the std::array<..., N> compile-time size constant.
constexpr std::array<std::pair<MaterialType, MaterialData>, 13> kMaterialData =
{{
  {MaterialType::STYROFOAM, {"styrofoam", 75.0}},
  {MaterialType::PINE, {"pine", 373.0}},
  {MaterialType::WOOD, {"wood", 700.0}},
  {MaterialType::OAK, {"oak", 710.0}},
  {MaterialType::PLASTIC, {"plastic", 1175.0}},
  {MaterialType::CONCRETE, {"concrete", 2000.0}},
  {MaterialType::ALUMINUM, {"aluminum", 2700.0}},
  {MaterialType::STEEL_ALLOY, {"steel_alloy", 7600.0}},
  {MaterialType::STEEL_STAINLESS, {"steel_stainless", 7800.0}},
  {MaterialType::IRON, {"iron", 7870.0}},
  {MaterialType::BRASS, {"brass", 8600.0}},
  {MaterialType::COPPER, {"copper", 8940.0}},
  {MaterialType::TUNGSTEN, {"tungsten", 19300.0}}
}};
#endif
