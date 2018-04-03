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
#include "ignition/math/MaterialDensity.hh"
#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include "ignition/math/Helpers.hh"

using namespace ignition;
using namespace math;

// Initialize the map of materials.
// Make sure that this set of materials matches the MaterialType enum in
// MaterialDensity.hh
std::map<MaterialType, Material> MaterialDensity::materials =
{{
  {MaterialType::STYROFOAM,
    {MaterialType::STYROFOAM,       "styrofoam",       75.0}},
  {MaterialType::PINE,
    {MaterialType::PINE,            "pine",            373.0}},
  {MaterialType::WOOD,
    {MaterialType::WOOD,            "wood",            700.0}},
  {MaterialType::OAK,
    {MaterialType::OAK,             "oak",             710.0}},
  {MaterialType::ICE,
    {MaterialType::ICE,             "ice",             916.0}},
  {MaterialType::WATER,
    {MaterialType::WATER,           "water",           1000.0}},
  {MaterialType::PLASTIC,
    {MaterialType::PLASTIC,         "plastic",         1175.0}},
  {MaterialType::CONCRETE,
    {MaterialType::CONCRETE,        "concrete",        2000.0}},
  {MaterialType::ALUMINUM,
    {MaterialType::ALUMINUM,        "aluminum",        2700.0}},
  {MaterialType::STEEL_ALLOY,
    {MaterialType::STEEL_ALLOY,     "steel_alloy",     7600.0}},
  {MaterialType::STEEL_STAINLESS,
    {MaterialType::STEEL_STAINLESS, "steel_stainless", 7800.0}},
  {MaterialType::IRON,
    {MaterialType::IRON,            "iron",            7870.0}},
  {MaterialType::BRASS,
    {MaterialType::BRASS,           "brass",           8600.0}},
  {MaterialType::COPPER,
    {MaterialType::COPPER,          "copper",          8940.0}},
  {MaterialType::TUNGSTEN,
    {MaterialType::TUNGSTEN,        "tungsten",        19300.0}}
}};

/////////////////////////////////////////////////
const std::map<MaterialType, Material> &MaterialDensity::Materials()
{
  return materials;
}

/////////////////////////////////////////////////
double MaterialDensity::Density(const std::string &_material)
{
  // Convert to lowercase.
  std::string material = _material;
  std::transform(material.begin(), material.end(), material.begin(), ::tolower);

  for (const std::pair<MaterialType, Material> &mat : materials)
  {
    if (mat.second.name == material)
      return mat.second.density;
  }
  return -1;
}

/////////////////////////////////////////////////
double MaterialDensity::Density(const MaterialType _material)
{
  return materials[_material].density;
}

/////////////////////////////////////////////////
Material MaterialDensity::Nearest(const double _value, const double _epsilon)
{
  double min = MAX_D;
  Material result;

  for (const std::pair<MaterialType, Material> &mat : materials)
  {
    double diff = std::fabs(mat.second.density - _value);
    if (diff < min && diff < _epsilon)
    {
      min = diff;
      result = mat.second;
    }
  }

  return result;
}
