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

#include "gz/math/Material.hh"

#include <algorithm>
#include <memory>

#include "gz/math/Helpers.hh"

// Placing the kMaterialData in a separate file for conveniece and clarity.
#include "MaterialType.hh"

using namespace gz;
using namespace math;

// Private data for the Material class
class gz::math::Material::Implementation
{
  /// \brief Set from a kMaterialData constant.
  public: void SetFrom(const std::pair<MaterialType, MaterialData>& _input)
  {
    type = _input.first;
    name = _input.second.name;
    density = _input.second.density;
  }

  /// \brief The material type.
  public: MaterialType type = MaterialType::UNKNOWN_MATERIAL;

  /// \brief Name of the material. This will match the names
  /// used in MaterialType, but in lowercase.
  public: std::string name = "";

  /// \brief Density value of the material in kg/m^3.
  public: double density = -1;
};

///////////////////////////////
Material::Material()
: dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

///////////////////////////////
Material::Material(const MaterialType _type)
: Material()
{
  auto iter = std::find_if(std::begin(kMaterialData), std::end(kMaterialData),
                           [_type](const auto& item) {
                             return item.first == _type;
                           });
  if (iter != std::end(kMaterialData))
  {
    this->dataPtr->SetFrom(*iter);
  }
}

///////////////////////////////
Material::Material(const std::string &_typename)
: Material()
{
  // Convert to lowercase.
  std::string material = _typename;
  std::transform(material.begin(), material.end(), material.begin(), ::tolower);

  auto iter = std::find_if(std::begin(kMaterialData), std::end(kMaterialData),
                           [&material](const auto& item) {
                             return item.second.name == material;
                           });
  if (iter != std::end(kMaterialData))
  {
    this->dataPtr->SetFrom(*iter);
  }
}

///////////////////////////////
Material::Material(const double _density)
: Material()
{
  this->dataPtr->density = _density;
}

///////////////////////////////
const std::map<MaterialType, Material> &Material::Predefined()
{
  using Map = std::map<MaterialType, Material>;

  // Initialize the static map of Material objects based on the kMaterialData.
  // We construct upon first use and never destroy it, in order to avoid the
  // [Static Initialization Order Fiasco](https://en.cppreference.com/w/cpp/language/siof).
  static const Map * const kMaterials = []()
  {
    auto temporary = std::make_unique<Map>();
    for (const auto &item : kMaterialData)
    {
      MaterialType type = item.first;
      Material& material = (*temporary)[type];
      material.dataPtr->SetFrom(item);
    }
    return temporary.release();
  }();

  return *kMaterials;
}

///////////////////////////////
bool Material::operator==(const Material &_material) const
{
  // Not checking the name, because type should be enough.
  return this->dataPtr->type == _material.dataPtr->type &&
         equal(this->dataPtr->density, _material.dataPtr->density);
}

///////////////////////////////
bool Material::operator!=(const Material &_material) const
{
  return !(*this == _material);
}

///////////////////////////////
MaterialType Material::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Material::SetType(const MaterialType _type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
std::string Material::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Material::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
double Material::Density() const
{
  return this->dataPtr->density;
}

//////////////////////////////////////////////////
void Material::SetDensity(const double _density)
{
  this->dataPtr->density = _density;
}

//////////////////////////////////////////////////
void Material::SetToNearestDensity(const double _value, const double _epsilon)
{
  double min = MAX_D;
  Material result;

  for (const auto &item : kMaterialData)
  {
    double diff = std::fabs(item.second.density - _value);
    if (diff < min && diff < _epsilon)
    {
      min = diff;
      result.dataPtr->SetFrom(item);
    }
  }

  if (result.Type() != MaterialType::UNKNOWN_MATERIAL)
    *this = result;
}
